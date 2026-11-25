#!/usr/bin/env python3
r"""
Pinger filter parameters: `config/pinger_filter.yaml` (every parameter, its
range and, in plain words, what it does), its presets, and a tuning file.

ROS-FREE (stdlib + PyYAML). The station (BlueBoat-MCS) reads the same YAML as
data for its filter dialog.

    params = load_params(preset='slow_rov', yaml_path='my_tuning.yaml')
    params.values['latency_s'], params.hash      # 16 hex digits of the set

A tuning file is `params: {name: value, ...}` (or a flat mapping), optionally
with `preset:`; only the values it names change. Order: defaults <- preset <-
tuning file <- overrides. Every set is validated whole: a wrong name, type or
range is refused with the reason.
"""

import hashlib
import json
import os

import yaml

SCHEMA_NAME = "pinger_filter.yaml"
SCHEMA_ID = "blueboat_pinger_filter/1"
DEFAULT_PRESET = "slow_rov"
CUSTOM = "custom"


class ParamError(ValueError):
    """An invalid parameter set; the message names every problem."""


def schema_path():
    """The schema file: the source tree's (beside this file's package) or the
    installed share's (beside the installed lib/blueboat_control/)."""
    here = os.path.dirname(os.path.abspath(__file__))
    for c in (os.path.join(here, "..", "..", "config", SCHEMA_NAME),
              os.path.join(here, "..", "..", "share", "blueboat_control", "config", SCHEMA_NAME)):
        if os.path.isfile(c):
            return os.path.abspath(c)
    raise FileNotFoundError(f"pinger filter parameters ({SCHEMA_NAME}) not found from {here}")


_SCHEMA = None


def load_schema():
    """The parsed schema {schema, groups, parameters, presets, ...} (read once; do not modify)."""
    global _SCHEMA
    if _SCHEMA is None:
        path = schema_path()
        with open(path, "r") as handle:
            data = yaml.safe_load(handle)
        if not isinstance(data, dict) or data.get("schema") != SCHEMA_ID:
            raise ParamError(f"{path}: not a {SCHEMA_ID} file")
        _SCHEMA = data
    return _SCHEMA


def _coerce(name, value, spec):
    """`value` converted to the parameter's type, or ParamError."""
    kind = spec["type"]
    try:
        if kind == "bool":
            if isinstance(value, str):
                low = value.strip().lower()
                if low in ("true", "1", "yes", "on"):
                    return True
                if low in ("false", "0", "no", "off"):
                    return False
                raise ValueError(value)
            return bool(value)
        if kind == "enum":
            return str(value)
        if isinstance(value, bool):
            raise ValueError(value)
        if kind == "int":
            f = float(value)
            if f != int(f):
                raise ValueError(value)
            return int(f)
        if kind == "float":
            return float(value)
    except (TypeError, ValueError, OverflowError):
        raise ParamError(f"{name}: {value!r} is not a {kind}") from None
    raise ParamError(f"{name}: unknown type {kind!r} in the schema")


def _problem(name, value, spec):
    """None if `value` (already coerced) is allowed, else the reason."""
    if spec["type"] == "enum":
        if value not in spec["choices"]:
            return f"{name}: {value!r} is not one of {', '.join(spec['choices'])}"
        return None
    if spec["type"] in ("int", "float"):
        if value != value or value in (float("inf"), float("-inf")):
            return f"{name}: {value} is not a finite number"
        lo, hi, unit = spec.get("min"), spec.get("max"), spec.get("unit", "")
        if lo is not None and value < lo:
            return f"{name}: {value} is below the minimum {lo} {unit}".rstrip()
        if hi is not None and value > hi:
            return f"{name}: {value} is above the maximum {hi} {unit}".rstrip()
    if name in ("y_sign", "z_sign") and value == 0:
        return f"{name}: must be +1 or -1"
    return None


def validate(values):
    """The coerced values, or ParamError naming every problem. An unknown name
    is an error (a typo must never be silently ignored)."""
    specs = load_schema()["parameters"]
    problems = [f"{name}: unknown parameter" for name in values if name not in specs]
    out = {}
    for name, spec in specs.items():
        if name not in values:
            problems.append(f"{name}: missing")
            continue
        try:
            out[name] = _coerce(name, values[name], spec)
        except ParamError as exc:
            problems.append(str(exc))
            continue
        why = _problem(name, out[name], spec)
        if why:
            problems.append(why)
    if not problems:
        if out["gate_hard"] < out["gate_soft"]:
            problems.append("gate_hard must be >= gate_soft")
        if out["stale_timeout_s"] < out["coast_after_s"]:
            problems.append("stale_timeout_s must be >= coast_after_s")
    if problems:
        raise ParamError("; ".join(problems))
    return out


class Params:
    """A validated parameter set: `values`, the `preset` it builds on, its `hash`."""

    def __init__(self, values, preset):
        self.values = values
        self.preset = preset
        canon = json.dumps({k: values[k] for k in sorted(values)}, sort_keys=True, separators=(",", ":"))
        self.hash = hashlib.sha256(canon.encode("utf-8")).hexdigest()[:16]

    def __getitem__(self, key):
        return self.values[key]


def read_tuning_file(path):
    """(preset or None, {name: value}) of a tuning file."""
    with open(os.path.expanduser(path), "r") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ParamError(f"{path}: not a mapping")
    preset = str(data["preset"]) if data.get("preset") else None
    if "params" in data:
        values = data["params"] or {}             # every line commented out: nothing changed
        if not isinstance(values, dict):
            raise ParamError(f"{path}: `params:` is not a mapping")
        return preset, dict(values)
    return preset, {k: v for k, v in data.items() if k not in ("schema", "preset", "hash", "description")}


def write_tuning_template(path, params):
    """Create a tuning file listing every parameter, commented out, with its
    current value, range and what it changes. Never overwrites a file."""
    schema = load_schema()
    lines = ["# Pinger filter tuning. Uncomment a line, change the value, save: the running",
             "# pinger_estimator applies it within a second, keeping its estimate. A bad value is",
             "# refused (the reason is in the log) and the previous set stays. Presets: "
             + ", ".join(schema["presets"]) + ".",
             f"preset: {params.preset}",
             "params:"]
    for group in schema["groups"]:
        lines.append(f"  # ---- {group}")
        for name, spec in schema["parameters"].items():
            if spec["group"] != group:
                continue
            value = params.values[name]
            shown = str(value).lower() if isinstance(value, bool) else value
            span = (f"[{spec['min']}, {spec['max']}] {spec.get('unit', '')}".rstrip()
                    if "min" in spec else " | ".join(spec.get("choices", [])) or "true | false")
            lines.append(f"  # {name}: {shown}    # {span}. {spec['description']}")
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "x") as handle:
        handle.write("\n".join(lines) + "\n")


def load_params(preset=DEFAULT_PRESET, overrides=None, yaml_path=None):
    """Defaults <- preset <- tuning file <- overrides, validated. A tuning file
    that names a preset builds on it, whatever `preset` says."""
    schema = load_schema()
    values = {name: spec["default"] for name, spec in schema["parameters"].items()}
    file_preset, file_values = read_tuning_file(yaml_path) if yaml_path else (None, {})
    if file_preset in schema["presets"]:
        preset = file_preset
    if preset not in (CUSTOM, None, ""):
        if preset not in schema["presets"]:
            raise ParamError(f"unknown preset {preset!r} (one of {', '.join(schema['presets'])}, custom)")
        values.update(schema["presets"][preset].get("values") or {})
    values.update(file_values)
    values.update(overrides or {})
    return Params(validate(values), preset or CUSTOM)
