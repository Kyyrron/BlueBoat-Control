#!/usr/bin/env python3
r"""
Pinger filter parameters: the schema file, presets, validation and the hash.

ROS-FREE BY CONSTRUCTION (stdlib + PyYAML). Read by the pinger_estimator node, the
pinger_filter_replay CLI, poslog_report and the log reviewer; the Mission Control
Station reads the SAME data file (`config/pinger_filter.yaml`) without importing
this module.

    params = load_params(preset='slow_rov', overrides={'latency_s': 1.2})
    params.values['latency_s']    -> 1.2
    params.hash                   -> 16 hex digits of the applied set

The schema file is found, in order: an explicit path; $BLUEBOAT_PINGER_SCHEMA;
the source tree (`<pkg>/config/` two levels above this file); the installed share
directory (`<install>/share/blueboat_control/config/`, relative to the installed
`lib/blueboat_control/`); ament_index if importable.
"""

import copy
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


def schema_path(explicit=None):
    """Absolute path of the schema file, or FileNotFoundError naming every place tried."""
    here = os.path.dirname(os.path.abspath(__file__))
    tried = []
    candidates = []
    if explicit:
        candidates.append(explicit)
    if os.environ.get("BLUEBOAT_PINGER_SCHEMA"):
        candidates.append(os.environ["BLUEBOAT_PINGER_SCHEMA"])
    candidates.append(os.path.join(here, "..", "..", "config", SCHEMA_NAME))           # source tree
    candidates.append(os.path.join(here, "..", "..", "share", "blueboat_control", "config", SCHEMA_NAME))
    for c in candidates:
        tried.append(os.path.abspath(c))
        if os.path.isfile(c):
            return os.path.abspath(c)
    try:
        from ament_index_python.packages import get_package_share_directory
        c = os.path.join(get_package_share_directory("blueboat_control"), "config", SCHEMA_NAME)
        tried.append(c)
        if os.path.isfile(c):
            return c
    except Exception:                                   # noqa: BLE001 - optional lookup
        pass
    raise FileNotFoundError("pinger filter schema not found; tried: " + ", ".join(tried))


_SCHEMA_CACHE = {}


def load_schema(path=None):
    """The parsed schema dict ({schema, groups, parameters, presets})."""
    p = schema_path(path)
    if p not in _SCHEMA_CACHE:
        with open(p, "r") as handle:
            data = yaml.safe_load(handle)
        if not isinstance(data, dict) or data.get("schema") != SCHEMA_ID:
            raise ParamError(f"{p}: not a {SCHEMA_ID} file")
        _SCHEMA_CACHE[p] = data
    return copy.deepcopy(_SCHEMA_CACHE[p])


def defaults(schema=None):
    schema = schema or load_schema()
    return {name: spec["default"] for name, spec in schema["parameters"].items()}


def preset_names(schema=None):
    schema = schema or load_schema()
    return list(schema["presets"].keys())


def coerce(name, value, spec):
    """`value` converted to the parameter's type, or ParamError with the reason."""
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
        if kind == "int":
            if isinstance(value, bool):
                raise ValueError(value)
            f = float(value)
            if f != int(f):
                raise ValueError(value)
            return int(f)
        if kind == "float":
            if isinstance(value, bool):
                raise ValueError(value)
            return float(value)
        if kind == "enum":
            return str(value)
    except (TypeError, ValueError, OverflowError):
        raise ParamError(f"{name}: {value!r} is not a {kind}") from None
    raise ParamError(f"{name}: unknown type {kind!r} in the schema")


def check_value(name, value, spec):
    """None if `value` (already coerced) is allowed, else the reason (one line)."""
    kind = spec["type"]
    if kind == "enum":
        if value not in spec["choices"]:
            return f"{name}: {value!r} is not one of {', '.join(spec['choices'])}"
        return None
    if kind in ("int", "float"):
        if value != value or value in (float("inf"), float("-inf")):
            return f"{name}: {value} is not a finite number"
        lo, hi = spec.get("min"), spec.get("max")
        if lo is not None and value < lo:
            return f"{name}: {value} is below the minimum {lo} {spec.get('unit', '')}".rstrip()
        if hi is not None and value > hi:
            return f"{name}: {value} is above the maximum {hi} {spec.get('unit', '')}".rstrip()
    if name in ("y_sign", "z_sign") and value == 0:
        return f"{name}: must be +1 or -1"
    return None


def cross_checks(values):
    """Reasons that involve several parameters."""
    out = []
    if values["gate_hard"] < values["gate_soft"]:
        out.append("gate_hard must be >= gate_soft")
    if values["stale_timeout_s"] < values["coast_after_s"]:
        out.append("stale_timeout_s must be >= coast_after_s")
    return out


class Params:
    """An applied, validated parameter set."""

    def __init__(self, values, preset, schema):
        self.values = dict(values)
        self.preset = preset
        self.schema_id = schema["schema"]
        # Once: read with every fix and estimate, and `values` is never
        # changed after validation.
        self.hash = params_hash(self.values)

    def __getitem__(self, key):
        return self.values[key]

    def as_dict(self):
        return {"schema": self.schema_id, "preset": self.preset, "params": dict(self.values),
                "hash": self.hash}


def params_hash(values):
    canon = json.dumps({k: values[k] for k in sorted(values)}, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canon.encode("utf-8")).hexdigest()[:16]


def validate(values, schema=None):
    """(coerced values, []) or raise ParamError listing every problem. Unknown
    names are errors (a typo must not be silently ignored)."""
    schema = schema or load_schema()
    specs = schema["parameters"]
    problems, out = [], {}
    for name in values:
        if name not in specs:
            problems.append(f"{name}: unknown parameter")
    for name, spec in specs.items():
        if name not in values:
            problems.append(f"{name}: missing")
            continue
        try:
            v = coerce(name, values[name], spec)
        except ParamError as exc:
            problems.append(str(exc))
            continue
        why = check_value(name, v, spec)
        if why:
            problems.append(why)
        out[name] = v
    if not problems:
        problems.extend(cross_checks(out))
    if problems:
        raise ParamError("; ".join(problems))
    return out


def load_params(preset=DEFAULT_PRESET, overrides=None, yaml_path=None, schema=None):
    """Defaults <- preset <- YAML file (`params:` mapping or a flat mapping) <- overrides.

    preset `custom` means "the YAML file / overrides on top of the defaults".
    """
    schema = schema or load_schema()
    values = defaults(schema)
    file_values = read_param_yaml(yaml_path) if yaml_path else None
    file_preset = read_file_preset(yaml_path) if yaml_path else None
    if file_preset and file_preset in schema["presets"]:
        # A saved set builds on the preset it was saved from (as the station
        # reads it), whatever preset the launch names.
        preset = file_preset
    if preset not in (CUSTOM, None, ""):
        if preset not in schema["presets"]:
            raise ParamError(f"unknown preset {preset!r} (one of {', '.join(schema['presets'])}, custom)")
        values.update(schema["presets"][preset].get("values") or {})
    if file_values:
        values.update(file_values)
    if overrides:
        values.update(overrides)
    return Params(validate(values, schema), preset or CUSTOM, schema)


def read_param_yaml(path):
    """The parameter mapping of a saved file ({params: {...}} or flat)."""
    with open(os.path.expanduser(path), "r") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ParamError(f"{path}: not a mapping")
    if "params" in data and isinstance(data["params"], dict):
        return dict(data["params"])
    return {k: v for k, v in data.items() if k not in ("schema", "preset", "hash", "description")}


def read_file_preset(path):
    """The `preset:` a saved set names, or None."""
    with open(os.path.expanduser(path), "r") as handle:
        data = yaml.safe_load(handle) or {}
    p = data.get("preset") if isinstance(data, dict) else None
    return str(p) if p else None


def write_param_yaml(path, params):
    """Write an applied set (the format MCS saves too)."""
    with open(os.path.expanduser(path), "w") as handle:
        yaml.safe_dump(params.as_dict(), handle, sort_keys=False)


def parse_overrides(pairs, schema=None):
    """['latency_s=1.2', 'robust=chi2'] -> {name: coerced value}."""
    schema = schema or load_schema()
    out = {}
    for pair in pairs or []:
        name, sep, raw = pair.partition("=")
        name = name.strip()
        if not sep:
            raise ParamError(f"{pair!r}: expected name=value")
        if name not in schema["parameters"]:
            raise ParamError(f"{name}: unknown parameter")
        out[name] = coerce(name, raw.strip(), schema["parameters"][name])
    return out
