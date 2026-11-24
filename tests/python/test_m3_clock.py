from dataclasses import replace

import pytest
from aerotwin_experiments.m3_clock import (
    MAX_NS,
    Mapping,
    Observation,
    State,
    correlate,
    microseconds_to_nanoseconds,
    normalize_reconnection,
)


def observation(**changes):
    return replace(Observation("epoch1", 1, "publisher", "message1", 10, "wire", 1000), **changes)


def valid():
    mapping = Mapping("epoch1")
    mapping.begin("publisher", "wire")
    mapping.validate(epoch="epoch1", generation=1, offset_ns=-1000, uncertainty_ns=100)
    return mapping


@pytest.mark.parametrize("value", [0, -1, True, 1.5, MAX_NS // 1000 + 1])
def test_invalid_conversion(value):
    with pytest.raises(ValueError):
        microseconds_to_nanoseconds(value)


def test_exact_conversion():
    assert microseconds_to_nanoseconds(1795552167234198) == 1795552167234198000


def test_initialization_required():
    with pytest.raises(ValueError):
        Mapping("epoch1").convert(observation())


def test_conversion_and_overflow():
    mapping = valid()
    assert mapping.convert(observation()) == 9000
    mapping.begin("publisher", "wire")
    mapping.validate(epoch="epoch1", generation=2, offset_ns=MAX_NS, uncertainty_ns=0)
    with pytest.raises(ValueError):
        mapping.convert(observation(generation=2))
    assert mapping.state == State.INVALIDATED


@pytest.mark.parametrize(
    "changes",
    [
        {"publisher": "other"},
        {"domain": "other"},
        {"units": "ns"},
    ],
)
def test_identity_changes_invalidate(changes):
    mapping = valid()
    with pytest.raises(ValueError):
        mapping.convert(observation(**changes))
    assert mapping.state == State.INVALIDATED


@pytest.mark.parametrize("stamp,identity", [(10, "message1"), (10, "message2"), (9, "message2")])
def test_duplicate_reordered_or_backward_invalidates(stamp, identity):
    mapping = valid()
    mapping.convert(observation())
    with pytest.raises(ValueError):
        mapping.convert(observation(timestamp_us=stamp, identity=identity))
    assert mapping.state == State.INVALIDATED


def test_epoch_isolation_and_generation_revalidation():
    mapping = valid()
    mapping.restart("epoch2")
    mapping.begin("publisher2", "wire")
    generation = mapping.generation
    mapping.validate(epoch="epoch2", generation=generation, offset_ns=0, uncertainty_ns=0)
    with pytest.raises(ValueError):
        mapping.convert(observation())
    with pytest.raises(ValueError):
        mapping.convert(observation(epoch="epoch2", generation=1, publisher="publisher2"))
    assert (
        mapping.convert(observation(epoch="epoch2", generation=generation, publisher="publisher2"))
        == 10000
    )


@pytest.mark.parametrize("operation", ["reconnect", "synchronization_changed"])
def test_reconnection_and_sync_require_new_mapping(operation):
    mapping = valid()
    getattr(mapping, operation)()
    assert mapping.epoch == "epoch1"
    with pytest.raises(ValueError):
        mapping.convert(observation())
    mapping.begin("new", "wire")
    with pytest.raises(ValueError):
        mapping.validate(epoch="epoch1", generation=1, offset_ns=0, uncertainty_ns=0)
    mapping.validate(epoch="epoch1", generation=mapping.generation, offset_ns=0, uncertainty_ns=0)
    assert mapping.convert(observation(publisher="new", generation=mapping.generation)) == 10000


def test_unsupported_reset_invalidates():
    mapping = valid()
    with pytest.raises(ValueError):
        mapping.in_place_reset()
    assert mapping.state == State.INVALIDATED


def test_ambiguous_identity_and_unverified_match_rejected():
    source = [observation(domain="uorb")]
    assert correlate(source, [observation()])["matched"] == 0
    assert correlate(source * 2, [observation()], identity_verified=True)["matched"] == 0
    result = correlate(source, [observation(timestamp_us=15)], identity_verified=True)
    assert result["offset_ns"] == 5000
    assert result["uncertainty_ns"] is None


def test_normalization_preserves_raw_schema():
    raw = {
        "experiment": "Agent interruption and recovery; Gazebo reset was not requested",
        "clocks": {"time_only_reset": {"count": 3}},
        "transport": "FAIL",
        "cleanup": "PASS",
    }
    derived = normalize_reconnection(raw)
    assert derived["operation"] == "agent_reconnection"
    assert not derived["simulation_reset_requested"]
    assert "time_only_reset" in raw["clocks"]


def test_timestamp_semantics_cannot_be_mixed():
    mapping = valid()
    with pytest.raises(ValueError):
        mapping.convert(observation(semantics="publication"))
    assert mapping.state == State.INVALIDATED


def test_duplicate_wire_identity_is_ambiguous():
    assert (
        correlate([observation()], [observation(), observation()], identity_verified=True)[
            "matched"
        ]
        == 0
    )
