"""Range and type checks for live config sets, from the generated config-schema.json.

The robot only checks types; min/max live in config-docs.yaml, which the build compiles into
config-schema.json. Checking here keeps an agent from setting a gain the docs forbid.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

SCHEMA_RELATIVE = (
    "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/generated/config-schema.json"
)


@dataclass(frozen=True)
class Leaf:
    path: str
    type: str  # number, integer, boolean, string, array, object
    minimum: float | None
    maximum: float | None
    description: str


class ConfigSchema:
    def __init__(self, schema: dict):
        self._schema = schema

    @classmethod
    def load(cls, repo_root: Path) -> "ConfigSchema":
        return cls(json.loads((repo_root / SCHEMA_RELATIVE).read_text()))

    def leaf(self, path: str) -> Leaf:
        node: dict = self._schema
        for part in path.split("."):
            props = node.get("properties")
            if not props or part not in props:
                raise ValueError(f"no config key '{path}' (at '{part}')")
            node = props[part]
        kind, lo, hi = _type_and_range(node)
        return Leaf(path, kind, lo, hi, node.get("description", ""))

    def check(self, path: str, value: Any) -> Leaf:
        """Raises ValueError if value has the wrong type or is outside the documented range."""
        leaf = self.leaf(path)
        if leaf.type == "object":
            raise ValueError(f"{path} is a section; set one of its leaves")
        if leaf.type == "boolean":
            if not isinstance(value, bool):
                raise ValueError(f"{path} is true or false")
            return leaf
        if leaf.type == "string":
            if not isinstance(value, str):
                raise ValueError(f"{path} is a string")
            return leaf
        values = value if leaf.type == "array" else [value]
        if leaf.type == "array" and not isinstance(value, list):
            raise ValueError(f"{path} is a list of numbers")
        for v in values:
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f"{path} needs finite numbers, got {v!r}")
            if leaf.type == "integer" and v != int(v):
                raise ValueError(f"{path} is an integer, got {v}")
            if leaf.minimum is not None and v < leaf.minimum:
                raise ValueError(f"{path} = {v} is below its documented minimum {leaf.minimum}")
            if leaf.maximum is not None and v > leaf.maximum:
                raise ValueError(f"{path} = {v} is above its documented maximum {leaf.maximum}")
        return leaf


def _type_and_range(node: dict) -> tuple[str, float | None, float | None]:
    """The leaf's value type and range; numeric leaves may be oneOf [number, mirror-string]."""
    candidates = node.get("oneOf") or node.get("anyOf") or [node]
    for c in candidates:
        t = c.get("type")
        if t in ("number", "integer"):
            return t, c.get("minimum"), c.get("maximum")
    for c in candidates:
        t = c.get("type")
        if t == "array":
            items = c.get("items", {})
            return "array", items.get("minimum"), items.get("maximum")
        if t in ("boolean", "string", "object"):
            return t, None, None
    return ("object" if "properties" in node else "string"), None, None
