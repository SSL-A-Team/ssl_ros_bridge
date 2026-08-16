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
  enum field             → uint8 (ROS2 has no enum type; matches the uint8
                            constants emitted for the enum itself)
  nested message Foo.Bar → Bar.msg with flat name Foo_Bar
  map<K,V> field         → skipped (no ROS2 map type)

Options (via --ros2msg_opt=key=value,key=value):
  optional_submsg=has_field  (default) Emit bool has_<field> before each
                             non-oneof message-type field.
  optional_submsg=error      Reject any non-oneof message-type field as a
                             build error; forces schema authors to be explicit.
  sidecar=<path>             Path to a JSON annotation file. Fields listed in
                             it are consumed and emitted as annotated ROS
                             types; remaining fields pass through normally.

Annotation file format (per message):
  "MsgName": {
    "fields": {
      "proto_field": { "ros_type": "...", "ros_field": "...", "scale": ... },
      "unsupported_field": { "skip": true },
      "field_with_default": { "default": 1.0 }
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
      },
      {
        "ros_field": "designated_position", "ros_type": "geometry_msgs/Point32",
        "from": { "x": "nested_field.x", "y": "nested_field.y" },
        "optional": true
      }
    ]
  },
  "_skip_types": ["MsgNameToOmitEntirely", ...]

  A "from" value may be a dotted path (e.g. "nested_field.x") to reach one
  level into a nested message-type field, instead of a scalar field on the
  containing message directly. Only the top-level name (before the dot) is
  treated as consumed for passthrough-exclusion purposes.

  An "outputs" entry normally emits a plain (non-array) ROS field, matching
  a required or always-present proto source. Set "optional": true when the
  source is a proto2 "optional" message-type field, to preserve presence:
  the ROS field is emitted as a 0/1-element array instead, and the bridge
  layer guards the assignment on the source field's own has_<field>() check.

  Consumed fields (right-hand side of all "from" maps, plus all "fields" keys)
  are excluded from passthrough. The generator errors on unknown field
  references or user-supplied conversion_func values (only intrinsic
  conversions are supported).

  Some proto constructs have no sane ROS translation (e.g. google.protobuf.Any
  has no fixed schema to map to a ROS type). The generator fails by default on
  these; the annotation file can opt in to dropping them:
    - A "skip": true field annotation (must be the only key) omits that field
      from the generated .msg, like any other consumed field.
    - A top-level "_skip_types" list omits an entire message (no .msg file
      emitted). Any field referencing a skipped type must itself carry a
      "skip" annotation, or generation errors — skipping a type never
      silently skips its referencing fields.

  Recursive message types (a message referencing itself, directly or through
  other messages) are rejected up front, with the full reference chain, since
  ROS2 .msg files are fixed-layout structs and cannot be self-referential. Use
  a "skip" field annotation on one field in the cycle to break it. See
  find_type_cycles().

Intrinsic conversions (triggered by ros_type only, no conversion_func needed):
  builtin_interfaces/Time     float/double proto field → from_seconds()
                              int/uint proto field     → from_microseconds()
  builtin_interfaces/Duration same rules as Time
"""

from pathlib import Path
import sys
from typing import Iterable, Iterator

from google.protobuf import descriptor_pb2
from google.protobuf.compiler import plugin_pb2

sys.path.insert(0, str(Path(__file__).parent))
# Must follow the sys.path.insert() above, so this can't sort before the
# google.protobuf imports the way import-order linting wants.
from proto_shared import (  # noqa: E402, I100
    Annotations,
    build_map_entry_type_names,
    classify_field_shape,
    consumed_annotation_fields,
    FieldAnnotation,
    flatten_type_name,
    HAS_FIELD_PREFIX,
    iter_messages,
    load_annotations,
    MessageAnnotationEntry,
    output_proto_fields,
    OutputEntry,
    parse_options,
)

FD = descriptor_pb2.FieldDescriptorProto

SCALAR_TYPE_MAP = {
    FD.TYPE_DOUBLE:   'float64',
    FD.TYPE_FLOAT:    'float32',
    FD.TYPE_INT64:    'int64',
    FD.TYPE_UINT64:   'uint64',
    FD.TYPE_INT32:    'int32',
    FD.TYPE_FIXED64:  'uint64',
    FD.TYPE_FIXED32:  'uint32',
    FD.TYPE_BOOL:     'bool',
    FD.TYPE_STRING:   'string',
    FD.TYPE_BYTES:    'uint8[]',
    FD.TYPE_UINT32:   'uint32',
    FD.TYPE_SINT32:   'int32',
    FD.TYPE_SINT64:   'int64',
    FD.TYPE_SFIXED32: 'int32',
    FD.TYPE_SFIXED64: 'int64',
}

_FLOAT_TYPES = frozenset({FD.TYPE_FLOAT, FD.TYPE_DOUBLE})
_INT_TYPES = frozenset({
    FD.TYPE_INT32, FD.TYPE_INT64, FD.TYPE_UINT32, FD.TYPE_UINT64,
    FD.TYPE_SINT32, FD.TYPE_SINT64, FD.TYPE_FIXED32, FD.TYPE_FIXED64,
    FD.TYPE_SFIXED32, FD.TYPE_SFIXED64,
})

_INTRINSIC_ROS_TYPES = frozenset({
    'builtin_interfaces/Time',
    'builtin_interfaces/Duration',
})

# Populated by main() from the annotation file's "_skip_types" before generation runs.
_SKIP_TYPES: frozenset = frozenset()


def map_field_to_ros2_type(field: descriptor_pb2.FieldDescriptorProto) -> str:
    if field.type in SCALAR_TYPE_MAP:
        return SCALAR_TYPE_MAP[field.type]

    if field.type == FD.TYPE_ENUM:
        return 'uint8'

    if field.type == FD.TYPE_MESSAGE:
        if field.type_name == '.google.protobuf.Any':
            raise ValueError(
                "Fields with type 'Any' are not supported (no fixed schema to map "
                "to a ROS type). Add a 'skip' field annotation in the annotation file."
            )

        flat = flatten_type_name(field.type_name).replace('SSL_', '')
        if flat in _SKIP_TYPES:
            raise ValueError(
                f"references type '{flat}', which is skipped via the annotation "
                f"file's '_skip_types'. Add a 'skip' field annotation for this field, "
                f"or remove '{flat}' from '_skip_types'."
            )

        return flat

    raise ValueError(f"unhandled proto field type {field.type} in field '{field.name}'")


# ---------------------------------------------------------------------------
# Annotation entry validation (error accumulation is specific to this
# script's protoc-plugin response.error mechanism, so this stays local;
# load_annotations/consumed_annotation_fields/output_proto_fields are
# shared — see proto_shared.py)
# ---------------------------------------------------------------------------


def _validate_annotation_entry(
    annotation_entry: MessageAnnotationEntry,
    proto_field_map: dict[str, descriptor_pb2.FieldDescriptorProto],
    flat_name: str,
    errors: list[str],
) -> None:
    for proto_name, ann in annotation_entry.get('fields', {}).items():
        if proto_name not in proto_field_map:
            errors.append(
                f"{flat_name}: annotation 'fields' references unknown proto field '{proto_name}'"
            )

            continue

        if ann.get('skip'):
            extra_keys = sorted(
                k for k in ann if k != 'skip' and not k.startswith('_')
            )
            if extra_keys:
                errors.append(
                    f"{flat_name}.{proto_name}: 'skip' must be the only key in a "
                    f'skip annotation (found {extra_keys})'
                )

            continue

        if 'conversion_func' in ann:
            errors.append(
                f"{flat_name}.{proto_name}: user-supplied 'conversion_func' is not supported; "
                f'only intrinsic conversions (builtin_interfaces/Time, /Duration) are available.'
            )

        if 'default' in ann:
            pf = proto_field_map[proto_name]
            if pf.label == FD.LABEL_REPEATED:
                errors.append(
                    f"{flat_name}.{proto_name}: 'default' is not supported on a "
                    f"genuinely repeated proto field — a single default value can't "
                    f'stand in for a whole list.'
                )

    for out in annotation_entry.get('outputs', []):
        ros_field = out.get('ros_field', '<unnamed>')
        for proto_name in output_proto_fields(out):
            if proto_name not in proto_field_map:
                errors.append(
                    f"{flat_name}: annotation output '{ros_field}' references unknown "
                    f"proto field '{proto_name}'"
                )

        if 'conversion_func' in out:
            errors.append(
                f"{flat_name}: output '{ros_field}': user-supplied 'conversion_func' "
                f'is not supported.'
            )


def _iter_message_type_edges(
    msg: descriptor_pb2.DescriptorProto,
    consumed: frozenset[str],
    map_entry_type_names: frozenset[str],
) -> Iterator[tuple[str, str]]:
    """
    Yield (proto_field_name, target_flat_type_name) for msg's message-type fields.

    Only fields that will survive into the generated .msg — not consumed by
    an annotation, not a map entry, not Any, not a skipped type.
    """
    for field in msg.field:
        if field.name in consumed:
            continue
        if field.type != FD.TYPE_MESSAGE:
            continue
        if field.type_name in map_entry_type_names:
            continue
        if field.type_name == '.google.protobuf.Any':
            continue
        target = flatten_type_name(field.type_name).replace('SSL_', '')
        if target in _SKIP_TYPES:
            continue
        yield field.name, target


def find_type_cycles(
    all_files: dict[str, descriptor_pb2.FileDescriptorProto],
    file_to_generate_names: Iterable[str],
    annotations: Annotations,
    map_entry_type_names: frozenset[str],
) -> list[str]:
    """
    Detect cycles in the message-type reference graph across every generated message.

    Considers the graph post annotation consumption/skip. ROS2 .msg files are
    fixed-layout structs and cannot be self-referential, even indirectly —
    a cycle always breaks generation downstream (rosidl_generator_type_description
    has a latent bug: it crashes with an opaque KeyError instead of a clean
    error; see calculate_type_hash's double-delete of a cycled type's
    'default_value' key after deepcopy aliasing). Detect it here instead,
    with the concrete chain, so the annotation file can 'skip' a field to break it.
    """
    graph: dict[str, list[tuple[str, str]]] = {}
    for file_name in file_to_generate_names:
        fd = all_files[file_name]
        for flat_name, _cpp_name, msg in iter_messages(fd):
            if flat_name in _SKIP_TYPES:
                continue

            consumed = consumed_annotation_fields(annotations.get(flat_name, {}))
            graph[flat_name] = list(_iter_message_type_edges(msg, consumed, map_entry_type_names))

    found: set[tuple[tuple[str, ...], tuple[str, ...]]] = set()  # canonicalized (nodes, fields)

    def dfs(node: str, path: list[str], path_fields: list[str]) -> None:
        if node in path:
            i = path.index(node)
            nodes = tuple(path[i:]) + (node,)
            fields = tuple(path_fields[i:])
            # Canonicalize so the same cycle found from different start
            # nodes collapses to one report.
            j = nodes[:-1].index(min(nodes[:-1]))
            canon_nodes = nodes[j:-1] + nodes[:j] + (nodes[j],)
            canon_fields = fields[j:] + fields[:j]
            found.add((canon_nodes, canon_fields))

            return

        for field_name, target in graph.get(node, []):
            if target in graph:
                dfs(target, path + [node], path_fields + [field_name])

    for start in graph:
        dfs(start, [], [])

    errors: list[str] = []
    for nodes, fields in sorted(found):
        chain = ''.join(
            f'{n} --[{f}]--> ' for n, f in zip(nodes, fields)
        ) + nodes[-1]
        errors.append(
            f'Recursive message type reference detected: {chain}. ROS2 '
            f'messages are fixed-layout structs and cannot be '
            f"self-referential. Add a 'skip' field annotation in the "
            f'annotation file on one of the fields in the cycle to break it.'
        )

    return errors


def _emit_one_output(out: OutputEntry, lines: list[str]) -> None:
    """Emit the single ROS struct field for one annotation 'outputs' entry."""
    suffix = '[]' if out.get('optional') else ''
    lines.append(f"{out['ros_type']}{suffix} {out['ros_field']}")


def _format_default_literal(value: bool | str | int | float) -> str:
    """Format an annotation 'default' JSON value as a ROS2 .msg default-value literal."""
    if isinstance(value, bool):
        return 'true' if value else 'false'

    if isinstance(value, str):
        escaped = value.replace('\\', '\\\\').replace('"', '\\"')
        return f'"{escaped}"'

    return repr(value)


def _emit_annotation_field_override(
    proto_name: str,
    ann: FieldAnnotation,
    pf: descriptor_pb2.FieldDescriptorProto,
    lines: list[str],
    proto2: bool,
    errors: list[str],
    flat_name: str,
) -> None:
    """Emit one annotated single-field override (ros_type, ros_field, scale, default)."""
    if ann.get('skip'):
        return  # already excluded from passthrough via `consumed`

    ros_name = ann.get('ros_field', proto_name)

    if 'ros_type' in ann:
        ros_type = ann['ros_type']
        if ros_type in _INTRINSIC_ROS_TYPES and pf.type not in (_FLOAT_TYPES | _INT_TYPES):
            # Non-scalar source for Time/Duration; still emit — the bridge layer handles it
            pass
    elif 'scale' in ann and pf.type not in _FLOAT_TYPES:
        errors.append(
            f"{flat_name}.{proto_name}: 'scale' on non-float field "
            f"({SCALAR_TYPE_MAP.get(pf.type, f'type={pf.type}')}) requires an explicit "
            f"'ros_type' — scaling an integer without type promotion loses precision."
        )

        return
    else:
        try:
            ros_type = map_field_to_ros2_type(pf)
        except ValueError as e:
            errors.append(f'{flat_name}.{proto_name}: {e}')

            return

    shape = classify_field_shape(pf, proto2)

    if 'default' not in ann:
        if shape.is_repeated or shape.is_proto2_optional:
            lines.append(f'{ros_type}[] {ros_name}')
        else:
            lines.append(f'{ros_type} {ros_name}')

        return

    # 'default' was already rejected at validation time for genuinely repeated
    # fields (a single value cannot stand in for a whole list). A proto2
    # "optional" scalar is modeled as a 0/1-element ROS array (see
    # map_field_to_ros2_type), so its default is a 1-element array literal, matching
    # the presence convention: unset proto field -> empty array, default ROS
    # field -> that one value present.
    default_lit = _format_default_literal(ann['default'])
    if shape.is_proto2_optional:
        lines.append(f'{ros_type}[] {ros_name} [{default_lit}]')
    else:
        lines.append(f'{ros_type} {ros_name} {default_lit}')


# ---------------------------------------------------------------------------
# Main message generator
# ---------------------------------------------------------------------------

def generate_message_definition(
    msg: descriptor_pb2.DescriptorProto,
    flat_name: str,
    source_file: str,
    optional_submsg: str,
    errors: list[str],
    map_entry_type_names: frozenset[str],
    proto2: bool = False,
    annotation_entry: MessageAnnotationEntry | None = None,
) -> str:
    if annotation_entry is None:
        annotation_entry = {}

    lines = [
        f'# Generated from proto message {flat_name}',
        f'# Source: {source_file}',
        '',
    ]

    proto_field_map = {f.name: f for f in msg.field}

    # Validate annotation references before emitting anything.
    _validate_annotation_entry(annotation_entry, proto_field_map, flat_name, errors)

    consumed = consumed_annotation_fields(annotation_entry)
    field_overrides = annotation_entry.get('fields', {})

    # 1. Enum value constants, upfront per ROS2 convention — one blank line
    # after each enum's cluster.
    for enum in msg.enum_type:
        lines.append(f'# {enum.name}')
        for value in enum.value:
            constant_name = enum.name.upper() + '_' + value.name.upper()
            constant_value = value.number
            lines.append(f'uint8 {constant_name} = {constant_value}')

        lines.append('')

    # 2. Fields, walked in proto declaration order — this holds even for
    # annotated fields/outputs, so the .msg reads in the same order
    # as the .proto. An annotation 'outputs' entry (several proto fields
    # collapsing into one ROS field, e.g. x/y/z -> a Point32) is emitted once,
    # at the position of the first proto field it consumes; the entry's other
    # consumed fields are skipped where they would otherwise fall, rather
    # than splitting the composite field's data across multiple positions.
    pending_outputs: list[tuple[OutputEntry, frozenset[str]]] = [
        (out, frozenset(output_proto_fields(out)))
        for out in annotation_entry.get('outputs', [])
    ]
    output_trigger: dict[str, OutputEntry] = {}
    for field in msg.field:
        for i, (out, out_consumed) in enumerate(pending_outputs):
            if field.name in out_consumed:
                output_trigger[field.name] = out
                del pending_outputs[i]

                break

    emitted_oneofs: set[int] = set()

    for field in msg.field:
        if field.name in output_trigger:
            _emit_one_output(output_trigger[field.name], lines)
            continue

        if field.name in consumed:
            if field.name in field_overrides:
                _emit_annotation_field_override(
                    field.name, field_overrides[field.name], field, lines,
                    proto2, errors, flat_name,
                )
            # else: consumed by an 'outputs' entry, already emitted at
            # that entry's trigger field position above.
            continue

        if field.type == FD.TYPE_MESSAGE and field.type_name in map_entry_type_names:
            errors.append(
                f'{flat_name}.{field.name}: proto map<K,V> fields have no ROS2 '
                f"equivalent and are not supported. Add a 'skip' field annotation "
                f'in the annotation file for this field to drop it explicitly.'
            )
            continue

        shape = classify_field_shape(field, proto2)
        is_repeated, in_oneof, is_proto2_optional = shape

        if in_oneof:
            oi = field.oneof_index
            if oi in emitted_oneofs:
                continue

            emitted_oneofs.add(oi)

            oneof_name = msg.oneof_decl[oi].name
            oneof_fields = [
                f for f in msg.field
                if f.HasField('oneof_index') and f.oneof_index == oi
            ]

            lines.append('')
            lines.append(f'# oneof {oneof_name}')
            lines.append('# case constants use proto field numbers (stable across reordering)')
            lines.append(f'uint8 ONEOF_{oneof_name.upper()}_NONE=0')
            for of in oneof_fields:
                if of.number > 255:
                    errors.append(
                        f"{flat_name}: oneof '{oneof_name}' field '{of.name}' has field "
                        f'number {of.number} which exceeds the uint8 range (max 255) used '
                        f'for the case discriminant. Use field numbers ≤ 255 in oneof '
                        f'declarations, or file a request to widen the discriminant type.'
                    )
                    continue

                const = f'ONEOF_{oneof_name.upper()}_{of.name.upper()}'
                lines.append(f'uint8 {const}={of.number}')
            lines.append(f'uint8 {oneof_name}_case')
            for of in oneof_fields:
                try:
                    lines.append(f'{map_field_to_ros2_type(of)} {of.name}')
                except ValueError as e:
                    errors.append(f'{flat_name}.{of.name}: {e}')

            continue

        try:
            ros2_type = map_field_to_ros2_type(field)
        except ValueError as e:
            errors.append(f'{flat_name}.{field.name}: {e}')
            continue

        if is_repeated or is_proto2_optional:
            lines.append(f'{ros2_type}[] {field.name}')
        elif field.type == FD.TYPE_MESSAGE and not is_repeated:
            if proto2:
                lines.append(f'{ros2_type} {field.name}')
            elif optional_submsg == 'error':
                errors.append(
                    f'{flat_name}.{field.name}: non-oneof message-type field has '
                    f'implicit proto3 presence — set optional_submsg=has_field to '
                    f'auto-generate a bool presence flag, or move into a oneof.'
                )
                continue
            else:
                lines.append(f'bool {HAS_FIELD_PREFIX}{field.name}')
                lines.append(f'{ros2_type} {field.name}')
        else:
            lines.append(f'{ros2_type} {field.name}')

    return '\n'.join(lines) + '\n'


def generate_enum_definition(
    enum: descriptor_pb2.EnumDescriptorProto, flat_name: str, source_file: str,
) -> str:
    """
    Constants-only .msg for a top-level proto enum.

    Nested enums are handled separately, inline in their containing message
    (see the 'Enum value constants' step in generate_message_definition) — this
    covers only enums declared at file scope, which have no containing
    message to attach constants to.
    """
    lines = [
        f'# Generated from proto enum {flat_name}',
        f'# Source: {source_file}',
        '',
    ]
    for value in enum.value:
        lines.append(f'uint8 {value.name.upper()}={value.number}')
    return '\n'.join(lines) + '\n'


def main() -> None:
    data = sys.stdin.buffer.read()
    request = plugin_pb2.CodeGeneratorRequest()
    request.ParseFromString(data)

    response = plugin_pb2.CodeGeneratorResponse()
    response.supported_features = (
        plugin_pb2.CodeGeneratorResponse.FEATURE_PROTO3_OPTIONAL
    )

    opts = parse_options(request.parameter)
    optional_submsg = opts.get('optional_submsg', 'has_field')
    if optional_submsg not in ('has_field', 'error'):
        response.error = (
            f'Unknown optional_submsg={optional_submsg!r}. '
            f"Valid values: 'has_field', 'error'."
        )
        sys.stdout.buffer.write(response.SerializeToString())

        return

    annotations_path = opts.get('sidecar', None)
    annotations: Annotations = load_annotations(annotations_path)

    global _SKIP_TYPES
    _SKIP_TYPES = frozenset(annotations.get('_skip_types', []))
    seen_skip_types: set[str] = set()

    map_entry_type_names = build_map_entry_type_names(request.proto_file)
    all_files: dict[str, descriptor_pb2.FileDescriptorProto] = {
        f.name: f for f in request.proto_file
    }
    errors: list[str] = []

    errors.extend(find_type_cycles(
        all_files, request.file_to_generate, annotations, map_entry_type_names
    ))

    emitted_names: set[str] = set()

    for file_name in request.file_to_generate:
        fd = all_files[file_name]
        source_file = Path(file_name).name

        proto2 = (fd.syntax != 'proto3')
        for flat_name, _cpp_name, msg in iter_messages(fd):
            if flat_name in _SKIP_TYPES:
                seen_skip_types.add(flat_name)
                continue

            if flat_name in emitted_names:
                errors.append(f"duplicate generated message name '{flat_name}'")
                continue

            emitted_names.add(flat_name)
            out = response.file.add()
            out.name = f'{flat_name}.msg'
            out.content = generate_message_definition(
                msg, flat_name, source_file, optional_submsg, errors, map_entry_type_names,
                proto2, annotation_entry=annotations.get(flat_name, {})
            )

        for enum in fd.enum_type:
            flat_name = enum.name.replace('SSL_', '')
            if flat_name in _SKIP_TYPES:
                seen_skip_types.add(flat_name)
                continue

            if flat_name in emitted_names:
                errors.append(
                    f"duplicate generated message name '{flat_name}' (from top-level enum)"
                )
                continue

            emitted_names.add(flat_name)
            out = response.file.add()
            out.name = f'{flat_name}.msg'
            out.content = generate_enum_definition(enum, flat_name, source_file)

    unused_skip_types = _SKIP_TYPES - seen_skip_types
    if unused_skip_types:
        errors.append(
            f"annotation file '_skip_types' references message(s) never encountered "
            f'during generation: {sorted(unused_skip_types)}'
        )

    if errors:
        response.error = '\n'.join(errors)

    sys.stdout.buffer.write(response.SerializeToString())


if __name__ == '__main__':
    main()
