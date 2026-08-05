#!/usr/bin/env python3
"""
protoc plugin: generates ROS2 .msg files from .proto definitions.

Invoked by protoc as a subprocess; reads CodeGeneratorRequest from stdin,
writes CodeGeneratorResponse to stdout (standard protoc plugin protocol).

Type mappings:
  Proto scalar           → ROS2 primitive
  message Foo            → Foo.msg (separate file, referenced by name)
  enum Foo               → Foo.msg (constants-only message)
  repeated T             → T[] field
  oneof foo              → uint8 foo_case + constants + all arm fields
  enum field             → int32 (ROS2 has no enum type; constants in Foo.msg)
  nested message Foo.Bar → Bar.msg with flat name Foo_Bar
  map<K,V> field         → skipped (no ROS2 map type)

Options (via --ros2msg_opt=key=value,key=value):
  optional_submsg=has_field  (default) Emit bool has_<field> before each
                             non-oneof message-type field.
  optional_submsg=error      Reject any non-oneof message-type field as a
                             build error; forces schema authors to be explicit.
  sidecar=<path>             Path to a JSON annotation file. Fields listed in
                             the sidecar are consumed and emitted as annotated
                             ROS types; remaining fields pass through normally.

Sidecar annotation format (per message):
  "MsgName": {
    "fields": {
      "proto_field": { "ros_type": "...", "ros_field": "...", "scale": ... }
    },
    "outputs": [
      {
        "ros_field": "name", "ros_type": "geometry_msgs/Point32",
        "from": { "ros_component": "proto_field" },
        "scale": 1e-3
      },
      {
        "ros_field": "pose", "ros_type": "geometry_msgs/Pose",
        "position":    { "from": { "x": "tx", ... }, "scale": 1e-3 },
        "orientation": { "from": { "x": "q0", ... } }
      }
    ]
  }

  Consumed fields (right-hand side of all "from" maps, plus all "fields" keys)
  are excluded from passthrough. Generator errors on unknown field references
  or user-supplied conversion_func values (only intrinsic conversions supported).

Intrinsic conversions (triggered by ros_type only, no conversion_func needed):
  builtin_interfaces/Time     float/double proto field → from_seconds()
                              int/uint proto field     → from_microseconds()
  builtin_interfaces/Duration same rules as Time
"""

import json
import sys
from pathlib import Path
from google.protobuf.compiler import plugin_pb2
from google.protobuf import descriptor_pb2

sys.path.insert(0, str(Path(__file__).parent))
from ateam_proto_shared import (  # noqa: E402
    parse_options,
    flatten_type_name,
    build_map_entry_type_names,
    iter_messages,
)

# Public alias: tests may import plugin.strip_package.
strip_package = flatten_type_name

FD = descriptor_pb2.FieldDescriptorProto

SCALAR_TYPE_MAP = {
    FD.TYPE_DOUBLE:   "float64",
    FD.TYPE_FLOAT:    "float32",
    FD.TYPE_INT64:    "int64",
    FD.TYPE_UINT64:   "uint64",
    FD.TYPE_INT32:    "int32",
    FD.TYPE_FIXED64:  "uint64",
    FD.TYPE_FIXED32:  "uint32",
    FD.TYPE_BOOL:     "bool",
    FD.TYPE_STRING:   "string",
    FD.TYPE_BYTES:    "uint8[]",
    FD.TYPE_UINT32:   "uint32",
    FD.TYPE_SINT32:   "int32",
    FD.TYPE_SINT64:   "int64",
    FD.TYPE_SFIXED32: "int32",
    FD.TYPE_SFIXED64: "int64",
}

_FLOAT_TYPES = frozenset({FD.TYPE_FLOAT, FD.TYPE_DOUBLE})
_INT_TYPES = frozenset({
    FD.TYPE_INT32, FD.TYPE_INT64, FD.TYPE_UINT32, FD.TYPE_UINT64,
    FD.TYPE_SINT32, FD.TYPE_SINT64, FD.TYPE_FIXED32, FD.TYPE_FIXED64,
    FD.TYPE_SFIXED32, FD.TYPE_SFIXED64,
})

_INTRINSIC_ROS_TYPES = frozenset({
    "builtin_interfaces/Time",
    "builtin_interfaces/Duration",
})


def ros2_field_type(field: descriptor_pb2.FieldDescriptorProto) -> str:
    if field.type in SCALAR_TYPE_MAP:
        return SCALAR_TYPE_MAP[field.type]
    if field.type == FD.TYPE_ENUM:
        return "int32"
    if field.type == FD.TYPE_MESSAGE:
        return flatten_type_name(field.type_name)
    raise ValueError(f"unhandled proto field type {field.type} in field '{field.name}'")


def iter_enums(fd):
    """Yield (flat_name, enum) for all enums in fd, including those nested in messages."""
    for enum in fd.enum_type:
        yield enum.name, enum

    def _walk(msg, parent_flat: str):
        flat = f"{parent_flat}_{msg.name}" if parent_flat else msg.name
        for enum in msg.enum_type:
            yield f"{flat}_{enum.name}", enum
        for nested in msg.nested_type:
            yield from _walk(nested, flat)

    for msg in fd.message_type:
        yield from _walk(msg, "")


def generate_enum_msg(enum: descriptor_pb2.EnumDescriptorProto) -> str:
    lines = [f"# Generated from proto enum {enum.name}"]
    for v in enum.value:
        lines.append(f"int32 {v.name}={v.number}")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# Sidecar helpers
# ---------------------------------------------------------------------------

def load_sidecar(path: str | None) -> dict:
    if not path:
        return {}
    with open(path) as f:
        return json.load(f)


def _output_proto_fields(out: dict):
    """Yield proto field names consumed by one outputs entry."""
    for v in out.get("from", {}).values():
        yield v
    for sub in ("position", "orientation"):
        if sub in out:
            for v in out[sub].get("from", {}).values():
                yield v


def _sidecar_consumed(sidecar_entry: dict) -> frozenset:
    """Return the set of proto field names consumed by a message's sidecar entry."""
    consumed = set(sidecar_entry.get("fields", {}).keys())
    for out in sidecar_entry.get("outputs", []):
        consumed.update(_output_proto_fields(out))
    return frozenset(consumed)


def _validate_sidecar_entry(sidecar_entry: dict, proto_field_map: dict, flat_name: str, errors: list):
    for proto_name, ann in sidecar_entry.get("fields", {}).items():
        if proto_name not in proto_field_map:
            errors.append(f"{flat_name}: sidecar 'fields' references unknown proto field '{proto_name}'")
        if "conversion_func" in ann:
            errors.append(
                f"{flat_name}.{proto_name}: user-supplied 'conversion_func' is not supported; "
                f"only intrinsic conversions (builtin_interfaces/Time, /Duration) are available."
            )
    for out in sidecar_entry.get("outputs", []):
        ros_field = out.get("ros_field", "<unnamed>")
        for proto_name in _output_proto_fields(out):
            if proto_name not in proto_field_map:
                errors.append(
                    f"{flat_name}: sidecar output '{ros_field}' references unknown proto field '{proto_name}'"
                )
        if "conversion_func" in out:
            errors.append(
                f"{flat_name}: output '{ros_field}': user-supplied 'conversion_func' is not supported."
            )


def _emit_sidecar_outputs(sidecar_entry: dict, lines: list):
    """Emit ROS struct fields from outputs entries."""
    for out in sidecar_entry.get("outputs", []):
        lines.append(f"{out['ros_type']} {out['ros_field']}")


def _emit_sidecar_fields(
    sidecar_entry: dict,
    proto_field_map: dict,
    lines: list,
    proto2: bool,
    errors: list,
    flat_name: str,
):
    """Emit annotated single-field overrides (ros_type, ros_field, scale)."""
    for proto_name, ann in sidecar_entry.get("fields", {}).items():
        pf = proto_field_map.get(proto_name)
        if pf is None:
            continue  # already captured in validation

        ros_name = ann.get("ros_field", proto_name)

        if "ros_type" in ann:
            ros_type = ann["ros_type"]
            if ros_type in _INTRINSIC_ROS_TYPES and pf.type not in (_FLOAT_TYPES | _INT_TYPES):
                # Non-scalar source for Time/Duration — still emit, bridge layer will handle
                pass
        elif "scale" in ann and pf.type not in _FLOAT_TYPES:
            errors.append(
                f"{flat_name}.{proto_name}: 'scale' on non-float field "
                f"({SCALAR_TYPE_MAP.get(pf.type, f'type={pf.type}')}) requires an explicit "
                f"'ros_type' — scaling an integer without type promotion loses precision."
            )
            continue
        else:
            ros_type = ros2_field_type(pf)

        is_repeated = pf.label == FD.LABEL_REPEATED
        in_oneof = pf.HasField("oneof_index")
        is_proto2_optional = proto2 and pf.label == FD.LABEL_OPTIONAL and not in_oneof

        if is_repeated or is_proto2_optional:
            lines.append(f"{ros_type}[] {ros_name}")
        else:
            lines.append(f"{ros_type} {ros_name}")


# ---------------------------------------------------------------------------
# Main message generator
# ---------------------------------------------------------------------------

def generate_message_msg(
    msg: descriptor_pb2.DescriptorProto,
    flat_name: str,
    optional_submsg: str,
    errors: list,
    map_entry_type_names: frozenset,
    proto2: bool = False,
    sidecar_entry: dict | None = None,
) -> str:
    if sidecar_entry is None:
        sidecar_entry = {}

    lines = [f"# Generated from proto message {flat_name}"]

    proto_field_map = {f.name: f for f in msg.field}

    # Validate sidecar references before emitting anything.
    _validate_sidecar_entry(sidecar_entry, proto_field_map, flat_name, errors)

    consumed = _sidecar_consumed(sidecar_entry)

    # 1. Sidecar outputs (multi-field → ROS struct).
    _emit_sidecar_outputs(sidecar_entry, lines)

    # 2. Sidecar field overrides (single-field with ros_type / ros_field / scale).
    _emit_sidecar_fields(sidecar_entry, proto_field_map, lines, proto2, errors, flat_name)

    # 3. Passthrough: proto fields not consumed by the sidecar.
    emitted_oneofs: set = set()

    for field in msg.field:
        if field.name in consumed:
            continue
        if field.type == FD.TYPE_MESSAGE and field.type_name in map_entry_type_names:
            continue

        is_repeated = field.label == FD.LABEL_REPEATED
        in_oneof = field.HasField("oneof_index")
        is_proto2_optional = (
            proto2
            and field.label == FD.LABEL_OPTIONAL
            and not in_oneof
        )

        if in_oneof:
            oi = field.oneof_index
            if oi in emitted_oneofs:
                continue
            emitted_oneofs.add(oi)

            oneof_name = msg.oneof_decl[oi].name
            oneof_fields = [
                f for f in msg.field
                if f.HasField("oneof_index") and f.oneof_index == oi
            ]

            lines.append("")
            lines.append(f"# oneof {oneof_name}")
            lines.append(f"# case constants use proto field numbers (stable across reordering)")
            lines.append(f"uint8 ONEOF_{oneof_name.upper()}_NONE=0")
            for of in oneof_fields:
                if of.number > 255:
                    errors.append(
                        f"{flat_name}: oneof '{oneof_name}' field '{of.name}' has field "
                        f"number {of.number} which exceeds the uint8 range (max 255) used "
                        f"for the case discriminant. Use field numbers ≤ 255 in oneof "
                        f"declarations, or file a request to widen the discriminant type."
                    )
                    continue
                const = f"ONEOF_{oneof_name.upper()}_{of.name.upper()}"
                lines.append(f"uint8 {const}={of.number}")
            lines.append(f"uint8 {oneof_name}_case")
            for of in oneof_fields:
                lines.append(f"{ros2_field_type(of)} {of.name}")
            continue

        ros2_type = ros2_field_type(field)

        if is_repeated or is_proto2_optional:
            lines.append(f"{ros2_type}[] {field.name}")
        elif field.type == FD.TYPE_MESSAGE and not is_repeated:
            if proto2:
                lines.append(f"{ros2_type} {field.name}")
            elif optional_submsg == "error":
                errors.append(
                    f"{flat_name}.{field.name}: non-oneof message-type field has "
                    f"implicit proto3 presence — set optional_submsg=has_field to "
                    f"auto-generate a bool presence flag, or move into a oneof."
                )
                continue
            else:
                lines.append(f"bool has_{field.name}")
                lines.append(f"{ros2_type} {field.name}")
        else:
            lines.append(f"{ros2_type} {field.name}")

    return "\n".join(lines) + "\n"


def main() -> None:
    data = sys.stdin.buffer.read()
    request = plugin_pb2.CodeGeneratorRequest()
    request.ParseFromString(data)

    response = plugin_pb2.CodeGeneratorResponse()
    response.supported_features = (
        plugin_pb2.CodeGeneratorResponse.FEATURE_PROTO3_OPTIONAL
    )

    opts = parse_options(request.parameter)
    optional_submsg = opts.get("optional_submsg", "has_field")
    if optional_submsg not in ("has_field", "error"):
        response.error = (
            f"Unknown optional_submsg={optional_submsg!r}. "
            f"Valid values: 'has_field', 'error'."
        )
        sys.stdout.buffer.write(response.SerializeToString())
        return

    sidecar_path = opts.get("sidecar", None)
    sidecar = load_sidecar(sidecar_path)

    map_entry_type_names = build_map_entry_type_names(request)
    all_files = {f.name: f for f in request.proto_file}
    errors: list = []

    for file_name in request.file_to_generate:
        fd = all_files[file_name]

        for flat_name, enum in iter_enums(fd):
            out = response.file.add()
            out.name = f"{flat_name}.msg"
            out.content = generate_enum_msg(enum)

        proto2 = (fd.syntax != "proto3")
        for flat_name, msg in iter_messages(fd):
            out = response.file.add()
            out.name = f"{flat_name}.msg"
            out.content = generate_message_msg(
                msg, flat_name, optional_submsg, errors, map_entry_type_names,
                proto2, sidecar_entry=sidecar.get(flat_name, {})
            )

    if errors:
        response.error = "\n".join(errors)

    sys.stdout.buffer.write(response.SerializeToString())


if __name__ == "__main__":
    main()
