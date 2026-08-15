"""
Shared helpers for protoc_gen_ros2msg.py and gen_message_conversion.py.

protoc_gen_ros2msg.py (ssl_league_msgs) is the .msg generator;
gen_message_conversion.py (ssl_ros_bridge) is the C++ proto<->ROS bridge
generator. Both must agree on naming, map-entry detection, and annotation
semantics; that logic lives here once instead of two copies.

Import with:
    from ateam_proto_shared import (
        parse_options, flatten_type_name, strip_package,
        build_map_entry_type_names, iter_messages,
        load_annotations, consumed_annotation_fields, output_proto_fields,
        classify_field_shape, FieldShape,
        FieldAnnotation, OutputEntry, MessageAnnotationEntry, Annotations,
        HAS_FIELD_PREFIX,
    )
"""

import json
from typing import Any, Iterable, Iterator, NamedTuple, TypedDict

from google.protobuf import descriptor_pb2

FD = descriptor_pb2.FieldDescriptorProto

# ---------------------------------------------------------------------------
# Annotation JSON shapes
# ---------------------------------------------------------------------------
#
# Typed as TypedDict, not a dataclass: this is JSON config with no behavior,
# and every call site already uses plain dict access (.get(...), [...]).
# TypedDict types that shape and catches a typo'd key at type-check time
# without changing the runtime value. See the annotation file format in
# protoc_gen_ros2msg.py's module docstring.
#
# OutputEntry uses the functional TypedDict form because one of its keys is
# literally "from", a Python keyword. Class-based TypedDict syntax cannot
# declare an attribute named `from` — the functional constructor is
# required, not just an alternative spelling.


class FieldAnnotation(TypedDict, total=False):
    """One entry under a message's annotation entry 'fields' map."""

    ros_type: str
    ros_field: str
    scale: float
    skip: bool
    default: bool | str | int | float
    conversion_func: str  # unsupported; present only so it can be rejected


# The 'position' or 'orientation' sub-object of a Pose-shaped output entry.
OutputComponentSpec = TypedDict(
    'OutputComponentSpec',
    {'from': dict[str, str], 'scale': float},
    total=False,
)

OutputEntry = TypedDict(
    'OutputEntry',
    {
        'ros_field': str,
        'ros_type': str,
        'from': dict[str, str],
        'scale': float,
        'position': OutputComponentSpec,
        'orientation': OutputComponentSpec,
        'conversion_func': str,
    },
    total=False,
)


class MessageAnnotationEntry(TypedDict, total=False):
    """The annotation entry for one message, keyed by the message's flat_name."""

    fields: dict[str, FieldAnnotation]
    outputs: list[OutputEntry]


# The annotation file's top level mixes message-name keys (->
# MessageAnnotationEntry) with reserved keys ("_skip_types": list[str],
# documentation-only "_comment"/"_schema") that aren't message entries.
# TypedDict cannot express "arbitrary keys of type A except these keys of
# type B", so the top level stays loosely typed; only entries returned by
# annotations.get(flat_name, {}) are typed as MessageAnnotationEntry.
Annotations = dict[str, Any]

# Prefix for the ROS-side presence-sentinel bool emitted alongside a
# non-oneof message-type field under optional_submsg=has_field (see
# protoc_gen_ros2msg.py's module docstring) — e.g. "bool has_foo" next to
# "Foo foo". Shared so the .msg generator (emits the field) and the C++
# generator (emits "ros_msg.has_foo = true;") cannot drift on the name.
# Not protobuf's own has_foo() accessor — that naming is protobuf's
# convention, not ours.
HAS_FIELD_PREFIX = 'has_'


def parse_options(parameter: str) -> dict[str, str]:
    if not parameter:
        return {}
    return dict(kv.split('=', 1) for kv in parameter.split(',') if '=' in kv)


def flatten_type_name(type_name: str) -> str:
    """
    Convert a fully-qualified proto type name to a flat ROS2-compatible name.

    Package components (conventionally lowercase) are stripped; nested type
    components (CamelCase) are concatenated with no separator. ROS2 message
    type names must match '^[A-Z][A-Za-z0-9]*$' — rosidl_adapter rejects
    underscores — so unlike protobuf's own C++ class names (which join
    nested names with '_', e.g. 'Referee_Point'), the ROS-facing name
    cannot use '_' as a nesting separator.

    For example: .ateam.BasicControl -> BasicControl,
    .GameEvent.BallLeftField -> GameEventBallLeftField,
    .ateam_test.OuterMessage.Inner -> OuterMessageInner.
    """
    parts = [p for p in type_name.lstrip('.').split('.') if p and p[0].isupper()]
    return ''.join(parts)


# Alias preserved for callers that import the name directly.
strip_package = flatten_type_name


def build_map_entry_type_names(
    proto_files: Iterable[descriptor_pb2.FileDescriptorProto],
) -> frozenset[str]:
    """
    Return field.type_name values that are synthetic map-entry types.

    Recurses to arbitrary depth across every file in proto_files. proto_files
    is any iterable of descriptor_pb2.FileDescriptorProto — e.g. a
    CodeGeneratorRequest's `.proto_file`, or a FileDescriptorSet's `.file`.
    """
    result: set[str] = set()

    def _walk(msg: descriptor_pb2.DescriptorProto, parent_fqn: str) -> None:
        fqn = f'{parent_fqn}.{msg.name}'
        if msg.options.map_entry:
            result.add(fqn)
        for nested in msg.nested_type:
            _walk(nested, fqn)

    for fd in proto_files:
        pkg_prefix = f'.{fd.package}' if fd.package else ''
        for msg in fd.message_type:
            _walk(msg, pkg_prefix)

    return frozenset(result)


def iter_messages(
    fd: descriptor_pb2.FileDescriptorProto,
) -> Iterator[tuple[str, str, descriptor_pb2.DescriptorProto]]:
    """
    Yield (flat_name, cpp_name, msg) for all non-map-entry messages in fd.

    Includes nested messages. cpp_name is protobuf's own generated C++
    class name: nested components joined with '_' (e.g. 'Referee_Point').
    Protobuf flattens nested messages to global-scope C++ classes rather
    than nesting C++ namespaces or classes; see Referee_Point in a
    generated .pb.h.

    flat_name is the ROS-facing name: nested components concatenated with
    no separator (e.g. 'RefereePoint'), then any 'SSL_' prefix stripped.
    ROS2 message type names must match '^[A-Z][A-Za-z0-9]*$' (no
    underscores), so this cannot reuse cpp_name's '_'-joined form. See
    flatten_type_name for the same logic applied to field references.
    """
    def _walk(
        msg: descriptor_pb2.DescriptorProto, parent_cpp: str, parent_flat: str,
    ) -> Iterator[tuple[str, str, descriptor_pb2.DescriptorProto]]:
        cpp = f'{parent_cpp}_{msg.name}' if parent_cpp else msg.name
        flat = f'{parent_flat}{msg.name}' if parent_flat else msg.name
        flat = flat.replace('SSL_', '')
        if not msg.options.map_entry:
            yield flat, cpp, msg
        for nested in msg.nested_type:
            yield from _walk(nested, cpp, flat)

    for msg in fd.message_type:
        yield from _walk(msg, '', '')


class FieldShape(NamedTuple):
    """
    Classification of a proto field's ROS2 emission shape.

    One of: repeated (genuinely `repeated` in the proto), in a oneof, or a
    proto2 `optional` scalar/message (modeled as a 0/1-element ROS array;
    see map_field_to_ros2_type). This 3-line computation was duplicated across
    both generators; one typed helper replaces it.
    """

    is_repeated: bool
    in_oneof: bool
    is_proto2_optional: bool


def classify_field_shape(field: descriptor_pb2.FieldDescriptorProto, proto2: bool) -> FieldShape:
    in_oneof = field.HasField('oneof_index')
    return FieldShape(
        is_repeated=field.label == FD.LABEL_REPEATED,
        in_oneof=in_oneof,
        is_proto2_optional=proto2 and field.label == FD.LABEL_OPTIONAL and not in_oneof,
    )


# ---------------------------------------------------------------------------
# Annotation file helpers
# ---------------------------------------------------------------------------
#
# Annotation file format is documented in protoc_gen_ros2msg.py's module
# docstring (gen_message_conversion.py's docstring points there rather than
# duplicating it). Both generators consume the same JSON file and must
# agree on what a "consumed" field is, so that logic lives here once.

def load_annotations(path: str | None) -> Annotations:
    if not path:
        return {}
    with open(path) as f:
        return json.load(f)


def output_proto_fields(out: OutputEntry) -> Iterator[str]:
    """Yield proto field names consumed by one annotation 'outputs' entry."""
    for v in out.get('from', {}).values():
        yield v
    for sub in ('position', 'orientation'):
        if sub in out:
            for v in out[sub].get('from', {}).values():
                yield v


def consumed_annotation_fields(annotation_entry: MessageAnnotationEntry) -> frozenset[str]:
    """
    Return the proto field names consumed by a message's annotation entry.

    Right-hand side of all 'from' maps, plus all 'fields' keys. Excluded
    from passthrough emission by both generators.
    """
    consumed = set(annotation_entry.get('fields', {}).keys())
    for out in annotation_entry.get('outputs', []):
        consumed.update(output_proto_fields(out))
    return frozenset(consumed)
