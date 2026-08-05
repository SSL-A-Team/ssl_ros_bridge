"""
Shared helpers used by both protoc_gen_ros2msg.py and protoc_gen_ros2cpp.py.

Factored out to avoid duplication; import with:
    from ateam_proto_shared import (
        parse_options, flatten_type_name, strip_package,
        build_map_entry_type_names, iter_messages,
    )
"""

from google.protobuf import descriptor_pb2


def parse_options(parameter: str) -> dict:
    if not parameter:
        return {}
    return dict(kv.split("=", 1) for kv in parameter.split(",") if "=" in kv)


def flatten_type_name(type_name: str) -> str:
    """Convert a fully-qualified proto type name to a flat ROS2/C++-compatible name.

    Package components (conventionally all-lowercase) are stripped; nested type
    components (CamelCase, start with uppercase) are joined with '_'.

    Examples:
        .ateam.BasicControl               → BasicControl
        .GameEvent.BallLeftField          → GameEvent_BallLeftField
        .ateam_test.OuterMessage.Inner    → OuterMessage_Inner
    """
    parts = type_name.lstrip(".").split(".")
    type_parts = [p for p in parts if p and p[0].isupper()]
    return "_".join(type_parts) if type_parts else parts[-1]


# Alias preserved for callers that import the name directly.
strip_package = flatten_type_name


def build_map_entry_type_names(request) -> frozenset:
    """Return fully-qualified field.type_name values that are synthetic map-entry types."""
    result: set = set()

    def _walk(msg, parent_fqn: str) -> None:
        fqn = f"{parent_fqn}.{msg.name}"
        if msg.options.map_entry:
            result.add(fqn)
        for nested in msg.nested_type:
            _walk(nested, fqn)

    for fd in request.proto_file:
        pkg_prefix = f".{fd.package}" if fd.package else ""
        for msg in fd.message_type:
            _walk(msg, pkg_prefix)

    return frozenset(result)


def iter_messages(fd):
    """Yield (flat_name, msg) for all non-map-entry messages in fd, including nested."""
    def _walk(msg, parent_flat: str):
        flat = f"{parent_flat}_{msg.name}" if parent_flat else msg.name
        if not msg.options.map_entry:
            yield flat, msg
        for nested in msg.nested_type:
            yield from _walk(nested, flat)

    for msg in fd.message_type:
        yield from _walk(msg, "")
