import pytest
from aerotwin_experiments.m3_source_wire import fingerprint, match


def row(**changes):
    return dict(
        timestamp_us=10,
        timestamp_sample_us=9,
        position_m=[0.0, 1.0, 2.0],
        velocity_mps=[1.0, 2.0, 3.0],
        attitude_wxyz=[1.0, 0.0, 0.0, 0.0],
        pose_frame=1,
        velocity_frame=1,
        reset_counter=0,
        **changes,
    )


def test_timestamp_independence():
    source = row()
    wire = source | {"timestamp_us": 100, "timestamp_sample_us": 99}
    assert fingerprint(source) == fingerprint(wire)
    result = match([source], [wire])
    assert result["matched"] == 1
    assert result["pairs"][0]["publication_offset_ns"] == 90000
    assert result["pairs"][0]["acquisition_offset_ns"] == 90000
    assert not result["mapping_valid"]


def test_duplicates_on_either_side_rejected():
    value = row()
    assert match([value, value], [value])["matched"] == 0
    assert match([value], [value, value])["matched"] == 0


def test_signed_zero_and_payload_difference():
    value = row()
    assert fingerprint(value) != fingerprint(value | {"position_m": [-0.0, 1.0, 2.0]})
    assert fingerprint(value) != fingerprint(value | {"reset_counter": 1})


def test_invalid_nan_and_missing_fields():
    with pytest.raises(ValueError):
        fingerprint(row() | {"position_m": [float("nan"), 0, 0]})
    assert match([{}], [row()])["matched"] == 0


def test_float32_canonicalization_and_bad_shape():
    value = row()
    assert fingerprint(value | {"position_m": [1.0 + 1e-9, 1.0, 2.0]}) == fingerprint(
        value | {"position_m": [1.0, 1.0, 2.0]}
    )
    with pytest.raises(ValueError):
        fingerprint(value | {"position_m": [1.0]})


def test_hash_collision_rejects_ambiguous_matches(monkeypatch):
    from aerotwin_experiments import m3_source_wire

    monkeypatch.setattr(m3_source_wire, "fingerprint", lambda _: "collision")
    first = row()
    second = first | {"position_m": [4.0, 5.0, 6.0]}
    assert m3_source_wire.match([first, second], [first])["matched"] == 0


def test_cross_epoch_and_generation_pairs_are_rejected():
    value = row() | {"epoch": "first", "generation": 1}
    assert match([value], [value | {"epoch": "second"}])["matched"] == 0
    assert match([value], [value | {"generation": 2}])["matched"] == 0


def test_hash_collision_with_different_bytes_cannot_match(monkeypatch):
    from aerotwin_experiments import m3_source_wire

    monkeypatch.setattr(m3_source_wire, "fingerprint", lambda _: "collision")
    first = row()
    second = first | {"position_m": [4.0, 5.0, 6.0]}
    assert m3_source_wire.match([first], [second])["matched"] == 0


def test_invalid_frame_and_orientation_rejected():
    assert match([row() | {"pose_frame": 0}], [row()])["matched"] == 0
    assert match([row() | {"attitude_wxyz": [2.0, 0.0, 0.0, 0.0]}], [row()])["matched"] == 0


def test_summary_retains_integer_epoch_offsets():
    from aerotwin_experiments.m3_source_wire import summarize

    source = row()
    wire = source | {"timestamp_us": 1795552167234198, "timestamp_sample_us": 1795552167234197}
    report = summarize(match([source], [wire]))
    expected = (1795552167234198 - 10) * 1000
    assert report["publication"]["median_ns"] == expected
    assert report["acquisition"]["median_ns"] == expected
    assert report["publication_acquisition_offset_difference_ns"] == [0]
    assert report["arrival_ordering_violations"] == 0
