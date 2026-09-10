"""Dependency-free parser for the finite Intel FLASER dataset.

CARMEN FLASER: count, ranges, laser pose (3), odometry pose (3),
acquisition timestamp, host, logger timestamp. Original timestamps are preserved.
"""
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import math


@dataclass(frozen=True)
class LaserRecord:
    timestamp_ns: int
    ranges: tuple
    odometry: tuple
    line_number: int


def parse_flaser(line, line_number=0):
    tokens = line.split()
    if not tokens or tokens[0] != "FLASER":
        return None
    try:
        count = int(tokens[1])
        if count < 2 or len(tokens) < count + 11:
            raise ValueError("incomplete FLASER record")
        timestamp = Decimal(tokens[count + 8])
        if not timestamp.is_finite() or timestamp < 0:
            raise ValueError("invalid acquisition timestamp")
        timestamp_ns = int((timestamp * 1_000_000_000).to_integral_value())
        ranges = tuple(float(value) for value in tokens[2:count + 2])
        odometry = tuple(float(value) for value in tokens[count + 5:count + 8])
        if not all(math.isfinite(value) for value in odometry):
            raise ValueError("non-finite odometry")
    except (IndexError, ValueError, InvalidOperation, OverflowError) as error:
        raise ValueError(f"Invalid FLASER at line {line_number}: {error}") from error
    return LaserRecord(timestamp_ns, ranges, odometry, line_number)


def load_ordered_flaser(path):
    records = []
    with open(path, encoding="utf-8") as source:
        for number, line in enumerate(source, 1):
            record = parse_flaser(line, number)
            if record is not None:
                records.append(record)
    if not records:
        raise ValueError("Dataset contains no FLASER records")
    backwards = sum(a.timestamp_ns > b.timestamp_ns for a, b in zip(records, records[1:]))
    records.sort(key=lambda record: record.timestamp_ns)  # stable; never change time
    for a, b in zip(records, records[1:]):
        if a.timestamp_ns == b.timestamp_ns:
            raise ValueError(f"Ambiguous duplicate acquisition time at lines {a.line_number}, {b.line_number}")
    return records, backwards


def scan_angles(count, start_degrees=-90.0, increment_degrees=1.0):
    """Explicit Intel beam convention; max is the angle of the last real beam."""
    if count < 2 or not math.isfinite(start_degrees) or not math.isfinite(increment_degrees) or increment_degrees <= 0:
        raise ValueError("Invalid scan angle configuration")
    start, increment = math.radians(start_degrees), math.radians(increment_degrees)
    return start, start + (count - 1) * increment, increment
