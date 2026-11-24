"""Validation-only clock correlation and temporal-boundary enforcement."""

from dataclasses import dataclass
from enum import Enum

MAX_NS = (1 << 63) - 1


def microseconds_to_nanoseconds(value):
    if type(value) is not int or value <= 0 or value > MAX_NS // 1000:
        raise ValueError("Invalid or overflowing microsecond timestamp")
    return value * 1000


@dataclass(frozen=True)
class Observation:
    epoch: str
    generation: int
    publisher: str
    identity: str
    timestamp_us: int
    domain: str
    receipt_monotonic_ns: int
    semantics: str = "acquisition"
    units: str = "us"


class State(Enum):
    UNINITIALIZED = "UNINITIALIZED"
    VALIDATING = "VALIDATING"
    VALID = "VALID"
    INVALIDATED = "INVALIDATED"


class Mapping:
    """An explicitly validated mapping scoped to one simulation epoch/generation."""

    def __init__(self, epoch):
        if not epoch:
            raise ValueError("An explicit simulation epoch is required")
        self.epoch = epoch
        self.generation = 0
        self.state = State.UNINITIALIZED
        self.publisher = None
        self.last = None
        self.seen = set()
        self.reason = None

    def invalidate(self, reason):
        self.generation += 1
        self.state = State.INVALIDATED
        self.last = None
        self.seen.clear()
        self.reason = reason

    def begin(self, publisher, domain, semantics="acquisition"):
        if not publisher or not domain:
            raise ValueError("Publisher and clock domain require explicit validation")
        self.invalidate("Mapping initialization requires new evidence")
        self.publisher = publisher
        self.domain = domain
        self.semantics = semantics
        self.state = State.VALIDATING

    def validate(self, *, epoch, generation, offset_ns, uncertainty_ns):
        if self.state != State.VALIDATING or epoch != self.epoch or generation != self.generation:
            raise ValueError("Stale or uninitialized mapping validation")
        if (
            type(offset_ns) is not int
            or abs(offset_ns) > MAX_NS
            or type(uncertainty_ns) is not int
            or not 0 <= uncertainty_ns <= MAX_NS
        ):
            raise ValueError("Invalid mapping parameters")
        self.offset_ns = offset_ns
        self.uncertainty_ns = uncertainty_ns
        self.state = State.VALID

    def convert(self, observation):
        if self.state != State.VALID:
            raise ValueError("Mapping is not valid")
        if observation.epoch != self.epoch or observation.generation != self.generation:
            raise ValueError("Stale epoch or mapping generation")
        if observation.publisher != self.publisher or observation.domain != self.domain:
            self.invalidate("Publisher or clock domain changed")
            raise ValueError(self.reason)
        if observation.units != "us" or observation.semantics != self.semantics:
            self.invalidate("Unsupported timestamp representation")
            raise ValueError(self.reason)
        try:
            value = microseconds_to_nanoseconds(observation.timestamp_us) + self.offset_ns
        except ValueError:
            self.invalidate("Invalid source timestamp")
            raise
        if not 0 <= value <= MAX_NS:
            self.invalidate("Mapped timestamp outside signed nanosecond range")
            raise ValueError(self.reason)
        if observation.identity in self.seen or (self.last is not None and value <= self.last):
            self.invalidate("Duplicate, reordered or backward observation")
            raise ValueError(self.reason)
        self.last = value
        self.seen.add(observation.identity)
        return value

    def reconnect(self):
        self.invalidate("Agent reconnection requires publisher and mapping revalidation")

    def synchronization_changed(self):
        self.invalidate("Synchronization assumptions changed")

    def restart(self, epoch):
        if not epoch or epoch == self.epoch:
            raise ValueError("A full restart requires a new epoch identity")
        self.invalidate("Runtime restarted")
        self.epoch = epoch

    def in_place_reset(self):
        self.invalidate("In-place reset is unsupported")
        raise ValueError(self.reason)


def correlate(source, wire, *, identity_verified=False):
    """Only externally verified message identities qualify for exact pairing.

    Timestamp proximity or timestamp_sample alone cannot supply identity.
    Residuals describe the observed affine offset, not synchronization accuracy.
    """
    pairs = []
    rejected = []
    for observation in wire:
        candidates = [
            item
            for item in source
            if item.identity == observation.identity
            and item.epoch == observation.epoch
            and item.generation == observation.generation
            and item.semantics == observation.semantics
        ]
        same_wire = [
            item
            for item in wire
            if item.identity == observation.identity
            and item.epoch == observation.epoch
            and item.generation == observation.generation
        ]
        if (
            not identity_verified
            or len(candidates) != 1
            or len(same_wire) != 1
            or observation.units != "us"
            or candidates[0].units != "us"
        ):
            rejected.append(observation.identity)
            continue
        item = candidates[0]
        pairs.append(
            microseconds_to_nanoseconds(observation.timestamp_us)
            - microseconds_to_nanoseconds(item.timestamp_us)
        )
    offset = sorted(pairs)[len(pairs) // 2] if pairs else None
    return {
        "status": "PASS" if pairs and not rejected else "BLOCKED",
        "matched": len(pairs),
        "unmatched": len(rejected),
        "offset_ns": offset,
        "residuals_ns": [value - offset for value in pairs],
        "offset_span_ns": max(pairs) - min(pairs) if pairs else None,
        "ordering_violations": sum(
            b.timestamp_us <= a.timestamp_us for a, b in zip(wire, wire[1:])
        ),
        "uncertainty_ns": None,
        "limitation": "Requires independently verified unique message identity; residuals do not bound clock error",
    }


def normalize_reconnection(report):
    """Return a derived schema without modifying retained raw evidence."""
    if (
        report.get("experiment")
        != "Agent interruption and recovery; Gazebo reset was not requested"
    ):
        raise ValueError("Missing explicit experiment provenance")
    return {
        "operation": "agent_reconnection",
        "raw_phase": "time_only_reset",
        "observation": report["clocks"]["time_only_reset"],
        "simulation_reset_requested": False,
        "mapping_policy": "INVALIDATE",
        "raw_transport_continuity": report["transport"],
        "cleanup": report["cleanup"],
    }
