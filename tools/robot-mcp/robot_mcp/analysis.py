"""Compact summaries of a hardware recording, so an agent sees the shape before reading the CSV."""

from __future__ import annotations

import csv
import io
import math
from typing import Any


def summarize_csv(text: str, max_columns: int = 80) -> dict[str, Any]:
    """Per-column count/min/max/mean/first/last, sample timing, and which columns never changed.

    Columns that are empty or constant are listed rather than summarized, so the summary stays
    short on a robot with many idle devices.
    """
    reader = csv.reader(io.StringIO(text))
    try:
        header = next(reader)
    except StopIteration:
        return {"rows": 0}
    cols: list[list[float]] = [[] for _ in header]
    for row in reader:
        for i, cell in enumerate(row[: len(header)]):
            if cell != "":
                try:
                    cols[i].append(float(cell))
                except ValueError:
                    pass

    times = cols[0]
    out: dict[str, Any] = {"rows": len(times)}
    if len(times) >= 2:
        dts = sorted(b - a for a, b in zip(times, times[1:]))
        out["timing"] = {
            "duration_s": round((times[-1] - times[0]) / 1000.0, 3),
            "mean_dt_ms": round(sum(dts) / len(dts), 2),
            "p95_dt_ms": dts[min(len(dts) - 1, int(0.95 * len(dts)))],
            "max_dt_ms": dts[-1],
        }

    varying: dict[str, Any] = {}
    constant: dict[str, float] = {}
    empty: list[str] = []
    for name, values in zip(header[1:], cols[1:]):
        if not values:
            empty.append(name)
        elif min(values) == max(values):
            constant[name] = values[0]
        else:
            varying[name] = _stats(values)
    if len(varying) > max_columns:
        out["note"] = f"{len(varying) - max_columns} varying columns omitted; read the CSV"
        varying = dict(list(varying.items())[:max_columns])
    out["varying"] = varying
    out["constant"] = constant
    if empty:
        out["never_read"] = empty
    return out


def _stats(values: list[float]) -> dict[str, float]:
    mean = sum(values) / len(values)
    var = sum((v - mean) ** 2 for v in values) / len(values)
    return {
        "n": len(values),
        "min": _round(min(values)),
        "max": _round(max(values)),
        "mean": _round(mean),
        "std": _round(math.sqrt(var)),
        "first": _round(values[0]),
        "last": _round(values[-1]),
    }


def _round(v: float) -> float:
    return float(f"{v:.6g}")
