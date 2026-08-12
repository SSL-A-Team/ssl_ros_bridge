#!/usr/bin/env python3
r"""
Generate C++ fromProto bridge functions.

Reads SSL league proto files + sidecar JSON, emits two files:
  <output-dir>/message_conversion_generated.hpp
  <output-dir>/message_conversion_generated.cpp

Usage (from CMake or command line):
  python3 gen_message_conversion.py \
    --proto-files ssl_vision_detection.proto ... \
    --proto-paths /path/to/protos \
    --sidecar ssl_ros_annotations.json \
    --output-dir /path/to/output \
    [--proto-include-prefix ssl_league_protobufs] \
    [--ros-package ssl_league_msgs] \
    [--cpp-namespace ssl_ros_bridge::message_conversion]

Sidecar annotation format is the same as for protoc_gen_ros2msg.py.
Consumed fields (from 'outputs' groups + 'fields' keys) are excluded from
passthrough. All other proto fields are emitted using proto2/proto3-appropriate
accessor patterns.

Intrinsic conversions (no 'conversion_func' needed in sidecar):
  builtin_interfaces/Time     float/double → from_seconds  (×1e9 ns cast)
                              int/uint     → from_microseconds (×1000 ns cast)
  builtin_interfaces/Duration same rules
"""

import argparse
import os
from pathlib import Path
import subprocess
import sys

from google.protobuf import descriptor_pb2
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore

_LEAGUE_MSGS_CMAKE = Path(__file__).resolve().parent.parent.parent / 'ssl_league_msgs' / 'cmake'
sys.path.insert(0, str(_LEAGUE_MSGS_CMAKE))
# Must follow the sys.path.insert() above, so this can't sort before the
# rosidl_pycommon import the way import-order linting wants.
from ateam_proto_shared import (  # noqa: E402, I100
    build_map_entry_type_names,
    field_shape,
    FieldAnnotation,
    HAS_FIELD_PREFIX,
    iter_messages,
    load_sidecar,
    MessageSidecarEntry,
    OutputEntry,
    Sidecar,
    sidecar_consumed,
)

FD = descriptor_pb2.FieldDescriptorProto

FLOAT_TYPES = frozenset({FD.TYPE_FLOAT, FD.TYPE_DOUBLE})
INT_TYPES = frozenset({
    FD.TYPE_INT32, FD.TYPE_INT64, FD.TYPE_UINT32, FD.TYPE_UINT64,
    FD.TYPE_SINT32, FD.TYPE_SINT64, FD.TYPE_FIXED32, FD.TYPE_FIXED64,
    FD.TYPE_SFIXED32, FD.TYPE_SFIXED64,
})
INTRINSIC_ROS_TYPES = frozenset({'builtin_interfaces/Time', 'builtin_interfaces/Duration'})

# ── Name helpers ──────────────────────────────────────────────────────────────


def ros_cpp_type(ros_type: str) -> str:
    """'geometry_msgs/Point32' → 'geometry_msgs::msg::Point32'."""
    if '/' in ros_type:
        pkg, typ = ros_type.split('/', 1)
        return f'{pkg}::msg::{typ}'
    return ros_type


def ros_msg_type(flat: str, pkg: str) -> str:
    return f'{pkg}::msg::{flat}'


def proto_oneof_const(field_name: str) -> str:
    """'aimless_kick' → 'kAimlessKick'."""
    return 'k' + ''.join(p.capitalize() for p in field_name.split('_'))


# ── C++ expression helpers ────────────────────────────────────────────────────

def scale_lit(val: float) -> str:
    if abs(val - 1e-3) < 1e-12:
        return '1e-3f'
    return f'{val}f'


def acc(field_name: str) -> str:
    return f'proto_msg.{field_name}()'


# ── Sidecar output codegen ────────────────────────────────────────────────────

def _emit_component_assignments(
    target: str, from_map: dict[str, str], scale: float | None, ind: str,
) -> list[str]:
    """
    Emit 'target.<comp> = proto_msg.<field>() [* scale];' for each from_map entry.

    from_map maps ros_component_name -> proto_field_name. Shared by every
    'outputs' shape below (Point32/Vector3 are one component group; Pose is
    two — position and orientation — each handled by one call).
    """
    lines = []
    for ros_comp, pf in from_map.items():
        expr = acc(pf)
        if scale:
            expr = f'{expr} * {scale_lit(scale)}'

        lines.append(f'{ind}{target}.{ros_comp} = {expr};')

    return lines


def emit_output(out: OutputEntry, ind: str) -> list[str]:
    """Emit C++ for one 'outputs' entry."""
    ros_field = out['ros_field']
    ros_type = out['ros_type']
    target = f'ros_msg.{ros_field}'

    if ros_type in ('geometry_msgs/Point32', 'geometry_msgs/Vector3'):
        return _emit_component_assignments(target, out['from'], out.get('scale'), ind)

    if ros_type == 'geometry_msgs/Quaternion':
        return _emit_component_assignments(target, out['from'], None, ind)

    if ros_type == 'geometry_msgs/Pose':
        pos = out.get('position', {})
        ori = out.get('orientation', {})
        return (
            _emit_component_assignments(
                f'{target}.position', pos.get('from', {}), pos.get('scale'), ind,
            )
            + _emit_component_assignments(f'{target}.orientation', ori.get('from', {}), None, ind)
        )

    return [f"{ind}// TODO: unsupported output ros_type '{ros_type}' → '{ros_field}'"]


# ── Sidecar field override codegen ────────────────────────────────────────────

# (ros_type, is_float_source_field) -> wrapper around a proto accessor
# expression producing the intrinsic conversion. int/uint sources are in
# proto microseconds (x1000 -> ns); float/double sources are in proto
# seconds (x1e9 -> ns).
_INTRINSIC_WRAPPERS = {
    ('builtin_interfaces/Time', True):
        lambda e: f'rclcpp::Time(static_cast<int64_t>({e} * 1e9))',
    ('builtin_interfaces/Time', False):
        lambda e: f'rclcpp::Time(static_cast<int64_t>({e}) * 1000LL)',
    ('builtin_interfaces/Duration', True):
        lambda e: f'rclcpp::Duration::from_nanoseconds(static_cast<int64_t>({e} * 1e9))',
    ('builtin_interfaces/Duration', False):
        lambda e: f'rclcpp::Duration::from_nanoseconds(static_cast<int64_t>({e}) * 1000LL)',
}


def emit_field_override(
    proto_name: str,
    ann: FieldAnnotation,
    field: descriptor_pb2.FieldDescriptorProto,
    proto2: bool,
    ind: str,
) -> list[str]:
    lines = []
    ros_name = ann.get('ros_field', proto_name)
    ros_type = ann.get('ros_type')
    scale = ann.get('scale')

    is_rep, _in_oneof, is_p2opt = field_shape(field, proto2)

    def wrap_optional(inner: str) -> list[str]:
        return [
            f'{ind}if (proto_msg.has_{proto_name}()) {{',
            f'{ind}  ros_msg.{ros_name} = {{{inner}}};',
            f'{ind}}}',
        ]

    if ros_type in INTRINSIC_ROS_TYPES:
        expr = _INTRINSIC_WRAPPERS[(ros_type, field.type in FLOAT_TYPES)](acc(proto_name))

        if is_p2opt:
            lines += wrap_optional(expr)
        elif is_rep:
            lines.append(f'{ind}// TODO: repeated Time/Duration not implemented')
        else:
            lines.append(f'{ind}ros_msg.{ros_name} = {expr};')

    elif scale is not None:
        expr = f'{acc(proto_name)} * {scale_lit(scale)}'
        if is_p2opt:
            lines += wrap_optional(expr)
        elif is_rep:
            lines.append(
                f'{ind}std::transform(proto_msg.{proto_name}().begin(), '
                f'proto_msg.{proto_name}().end(),'
            )
            lines.append(f'{ind}  std::back_inserter(ros_msg.{ros_name}),')
            lines.append(f'{ind}  [](const auto & v) {{ return v * {scale_lit(scale)}; }});')
        else:
            lines.append(f'{ind}ros_msg.{ros_name} = {expr};')

    else:
        # Rename only — same type, different ros field name
        if field.type == FD.TYPE_MESSAGE:
            inner = f'fromProto({acc(proto_name)})'
            if is_p2opt:
                lines += [
                    f'{ind}if (proto_msg.has_{proto_name}()) {{',
                    f'{ind}  ros_msg.{ros_name} = {{fromProto(proto_msg.{proto_name}())}};',
                    f'{ind}}}',
                ]
            elif is_rep:
                lines.append(
                    f'{ind}std::transform(proto_msg.{proto_name}().begin(), '
                    f'proto_msg.{proto_name}().end(),'
                )
                lines.append(f'{ind}  std::back_inserter(ros_msg.{ros_name}),')
                lines.append(f'{ind}  [](const auto & p) {{ return fromProto(p); }});')
            else:
                lines.append(f'{ind}ros_msg.{ros_name} = fromProto({acc(proto_name)});')
        elif field.type == FD.TYPE_ENUM:
            inner = f'static_cast<uint8_t>({acc(proto_name)})'
            if is_p2opt:
                lines += wrap_optional(inner)
            else:
                lines.append(f'{ind}ros_msg.{ros_name} = {inner};')
        else:
            if is_p2opt:
                lines += wrap_optional(acc(proto_name))
            elif is_rep:
                lines.append(
                    f'{ind}std::copy(proto_msg.{proto_name}().begin(), '
                    f'proto_msg.{proto_name}().end(),'
                )
                lines.append(f'{ind}  std::back_inserter(ros_msg.{ros_name}));')
            else:
                lines.append(f'{ind}ros_msg.{ros_name} = {acc(proto_name)};')

    return lines


# ── Passthrough codegen ───────────────────────────────────────────────────────

def emit_passthrough(
    field: descriptor_pb2.FieldDescriptorProto,
    proto2: bool,
    ind: str,
) -> list[str]:
    lines = []
    name = field.name
    is_rep, _in_oneof, is_p2opt = field_shape(field, proto2)

    if field.type == FD.TYPE_BYTES:
        if is_p2opt:
            lines += [
                f'{ind}if (proto_msg.has_{name}()) {{',
                f'{ind}  auto & _b = {acc(name)};',
                f'{ind}  ros_msg.{name} = {{std::vector<uint8_t>(_b.begin(), _b.end())}};',
                f'{ind}}}',
            ]
        elif is_rep:
            lines.append(f'{ind}// TODO: repeated bytes passthrough')
        else:
            lines += [
                f'{ind}{{',
                f'{ind}  auto & _b = {acc(name)};',
                f'{ind}  ros_msg.{name}.assign(_b.begin(), _b.end());',
                f'{ind}}}',
            ]

        return lines

    if is_rep:
        if field.type == FD.TYPE_MESSAGE:
            lines.append(
                f'{ind}std::transform(proto_msg.{name}().begin(), proto_msg.{name}().end(),'
            )
            lines.append(f'{ind}  std::back_inserter(ros_msg.{name}),')
            lines.append(f'{ind}  [](const auto & p) {{ return fromProto(p); }});')
        elif field.type == FD.TYPE_ENUM:
            lines.append(
                f'{ind}std::transform(proto_msg.{name}().begin(), proto_msg.{name}().end(),'
            )
            lines.append(f'{ind}  std::back_inserter(ros_msg.{name}),')
            lines.append(f'{ind}  [](const auto & v) {{ return static_cast<uint8_t>(v); }});')
        else:
            lines.append(f'{ind}std::copy(proto_msg.{name}().begin(), proto_msg.{name}().end(),')
            lines.append(f'{ind}  std::back_inserter(ros_msg.{name}));')

        return lines

    if is_p2opt:
        if field.type == FD.TYPE_MESSAGE:
            lines += [
                f'{ind}if (proto_msg.has_{name}()) {{',
                f'{ind}  ros_msg.{name} = {{fromProto(proto_msg.{name}())}};',
                f'{ind}}}',
            ]
        elif field.type == FD.TYPE_ENUM:
            lines += [
                f'{ind}if (proto_msg.has_{name}()) {{',
                f'{ind}  ros_msg.{name} = {{static_cast<uint8_t>(proto_msg.{name}())}};',
                f'{ind}}}',
            ]
        else:
            lines += [
                f'{ind}if (proto_msg.has_{name}()) {{',
                f'{ind}  ros_msg.{name} = {{proto_msg.{name}()}};',
                f'{ind}}}',
            ]

        return lines

    # Singular non-optional (proto2 required or proto3 default)
    if field.type == FD.TYPE_MESSAGE:
        if proto2:
            lines.append(f'{ind}ros_msg.{name} = fromProto(proto_msg.{name}());')
        else:
            # proto3 singular message — emit has_ sentinel
            lines += [
                f'{ind}if (proto_msg.has_{name}()) {{',
                f'{ind}  ros_msg.{HAS_FIELD_PREFIX}{name} = true;',
                f'{ind}  ros_msg.{name} = fromProto(proto_msg.{name}());',
                f'{ind}}}',
            ]
    elif field.type == FD.TYPE_ENUM:
        lines.append(f'{ind}ros_msg.{name} = static_cast<uint8_t>({acc(name)});')
    else:
        lines.append(f'{ind}ros_msg.{name} = {acc(name)};')

    return lines


def emit_oneof(
    oi: int,
    msg: descriptor_pb2.DescriptorProto,
    cpp_name: str,
    consumed: frozenset[str],
    ind: str,
) -> list[str]:
    lines = []
    oneof_name = msg.oneof_decl[oi].name
    arms = [f for f in msg.field
            if f.HasField('oneof_index') and f.oneof_index == oi
            and f.name not in consumed]

    lines.append(
        f'{ind}ros_msg.{oneof_name}_case = static_cast<uint8_t>(proto_msg.{oneof_name}_case());'
    )
    lines.append(f'{ind}switch (proto_msg.{oneof_name}_case()) {{')

    for f in arms:
        const = f'{cpp_name}::{proto_oneof_const(f.name)}'
        lines.append(f'{ind}  case {const}:')
        if f.type == FD.TYPE_MESSAGE:
            lines.append(f'{ind}    ros_msg.{f.name} = fromProto(proto_msg.{f.name}());')
        elif f.type == FD.TYPE_ENUM:
            lines.append(f'{ind}    ros_msg.{f.name} = static_cast<uint8_t>({acc(f.name)});')
        else:
            lines.append(f'{ind}    ros_msg.{f.name} = {acc(f.name)};')
        lines.append(f'{ind}    break;')

    lines.append(f'{ind}  default: break;')
    lines.append(f'{ind}}}')

    return lines


# ── Header + source generation ────────────────────────────────────────────────

LICENSE = """\
// Copyright 2025 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
"""


def generate_header(
    fds: list[descriptor_pb2.FileDescriptorProto],
    sidecar: Sidecar,
    ros_pkg: str,
    proto_prefix: str,
    namespace: str,
) -> str:
    skip_types = frozenset(sidecar.get('_skip_types', []))
    guard = 'CORE__MESSAGE_CONVERSION_GENERATED_HPP_'
    lines = [
        LICENSE,
        '// AUTO-GENERATED — do not edit. Re-run gen_message_conversion.py.',
        f'#ifndef {guard}',
        f'#define {guard}',
        '',
    ]

    # Proto pb.h includes
    for fd in fds:
        pb_h = fd.name.replace('.proto', '.pb.h')
        lines.append(f'#include <{proto_prefix}/{pb_h}>')

    lines.append('')

    # ROS msg includes
    seen_inc = set()
    for fd in fds:
        for flat, _cpp_name, msg in iter_messages(fd):
            if flat in skip_types:
                continue

            inc = f'{ros_pkg}/msg/{convert_camel_case_to_lower_case_underscore(flat)}.hpp'
            if inc not in seen_inc:
                seen_inc.add(inc)
                lines.append(f'#include <{inc}>')

    lines.append('')

    # Common includes
    lines += [
        '#include <rclcpp/time.hpp>',
        '#include <rclcpp/duration.hpp>',
        '#include <geometry_msgs/msg/point32.hpp>',
        '#include <geometry_msgs/msg/pose.hpp>',
        '#include <geometry_msgs/msg/quaternion.hpp>',
        '#include <builtin_interfaces/msg/time.hpp>',
        '#include <builtin_interfaces/msg/duration.hpp>',
        '',
    ]

    for ns in namespace.split('::'):
        lines.append(f'namespace {ns}')
        lines.append('{')

    lines.append('')

    for fd in fds:
        for flat, cpp_name, msg in iter_messages(fd):
            if flat in skip_types:
                continue
            ros_t = ros_msg_type(flat, ros_pkg)
            lines.append(f'{ros_t} fromProto(const {cpp_name} & proto_msg);')

    lines.append('')
    for ns in reversed(namespace.split('::')):
        lines.append(f'}}  // namespace {ns}')

    lines.append('')
    lines.append(f'#endif  // {guard}')

    return '\n'.join(lines) + '\n'


def generate_source(
    fds: list[descriptor_pb2.FileDescriptorProto],
    all_map_entries: frozenset[str],
    sidecar: Sidecar,
    ros_pkg: str,
    namespace: str,
) -> str:
    lines = [
        LICENSE,
        '// AUTO-GENERATED — do not edit. Re-run gen_message_conversion.py.',
        '',
        '#include "message_conversion_generated.hpp"',
        '#include <algorithm>',
        '#include <vector>',
        '',
    ]

    for ns in namespace.split('::'):
        lines.append(f'namespace {ns}')
        lines.append('{')
    lines.append('')

    skip_types = frozenset(sidecar.get('_skip_types', []))

    for fd in fds:
        proto2 = fd.syntax != 'proto3'

        for flat, cpp_name, msg in iter_messages(fd):
            if flat in skip_types:
                continue

            entry: MessageSidecarEntry = sidecar.get(flat, {})
            consumed = sidecar_consumed(entry)
            pf_map: dict[str, descriptor_pb2.FieldDescriptorProto] = {f.name: f for f in msg.field}

            ros_t = ros_msg_type(flat, ros_pkg)
            lines.append(f'{ros_t} fromProto(const {cpp_name} & proto_msg)')
            lines.append('{')
            lines.append(f'  {ros_t} ros_msg;')

            # Sidecar outputs (multi-field → ROS struct)
            for out in entry.get('outputs', []):
                lines += emit_output(out, '  ')

            # Sidecar field overrides
            for pname, ann in entry.get('fields', {}).items():
                if ann.get('skip'):
                    continue

                pf = pf_map.get(pname)
                if pf:
                    lines += emit_field_override(pname, ann, pf, proto2, '  ')

            # Passthrough — skip consumed, error on unskipped map entries
            emitted_oneofs: set[int] = set()
            for field in msg.field:
                if field.name in consumed:
                    continue

                if field.type == FD.TYPE_MESSAGE and field.type_name in all_map_entries:
                    print(
                        f'{flat}.{field.name}: proto map<K,V> fields have no ROS2 '
                        f"equivalent and are not supported. Add a 'skip' field "
                        f'annotation in the sidecar for this field to drop it explicitly.',
                        file=sys.stderr,
                    )
                    sys.exit(1)

                if field.HasField('oneof_index'):
                    oi = field.oneof_index
                    if oi not in emitted_oneofs:
                        emitted_oneofs.add(oi)
                        lines += emit_oneof(oi, msg, cpp_name, consumed, '  ')
                    continue

                lines += emit_passthrough(field, proto2, '  ')

            lines.append('  return ros_msg;')
            lines.append('}')
            lines.append('')

    for ns in reversed(namespace.split('::')):
        lines.append(f'}}  // namespace {ns}')

    return '\n'.join(lines) + '\n'


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--proto-files', nargs='+', required=True)
    ap.add_argument('--proto-paths', nargs='+', default=[])
    ap.add_argument('--sidecar', default=None)
    ap.add_argument('--output-dir', required=True)
    ap.add_argument('--proto-include-prefix', default='ssl_league_protobufs')
    ap.add_argument('--ros-package', default='ssl_league_msgs')
    ap.add_argument('--cpp-namespace', default='ssl_ros_bridge::message_conversion')
    ap.add_argument(
        '--protoc-path', default='protoc',
        help="Path to the protoc binary to invoke (default: 'protoc' on PATH). Pass "
             'the same protoc CMake located, so this generator and the .msg generator '
             'parse the proto set with the same protoc build/version.',
    )

    args = ap.parse_args()

    # Produce a FileDescriptorSet via protoc --descriptor_set_out, written straight to
    # stdout — avoids a temp-file write/read/unlink round trip for a throwaway artifact.
    try:
        proto_path_args = [f'--proto_path={p}' for p in args.proto_paths]
        r = subprocess.run(
            [args.protoc_path, '--descriptor_set_out=/dev/stdout', '--include_imports']
            + proto_path_args + args.proto_files,
            capture_output=True,
        )
    except FileNotFoundError as e:
        print(f'Could not run protoc ({args.protoc_path!r}): {e}', file=sys.stderr)
        sys.exit(1)

    if r.returncode != 0:
        print(f"protoc failed:\n{r.stderr.decode(errors='replace')}", file=sys.stderr)
        sys.exit(1)

    fds_pb = descriptor_pb2.FileDescriptorSet()
    fds_pb.ParseFromString(r.stdout)

    # Match requested files against descriptor
    requested = {Path(p).name for p in args.proto_files}
    target_fds = [fd for fd in fds_pb.file if Path(fd.name).name in requested]
    if not target_fds:
        print('No matching proto files found in descriptor.', file=sys.stderr)
        sys.exit(1)

    sidecar = load_sidecar(args.sidecar)
    all_map_entries = build_map_entry_type_names(fds_pb.file)

    os.makedirs(args.output_dir, exist_ok=True)

    hpp = generate_header(
        target_fds, sidecar, args.ros_package, args.proto_include_prefix, args.cpp_namespace,
    )
    cpp = generate_source(
        target_fds, all_map_entries, sidecar, args.ros_package, args.cpp_namespace,
    )

    hpp_path = os.path.join(args.output_dir, 'message_conversion_generated.hpp')
    cpp_path = os.path.join(args.output_dir, 'message_conversion_generated.cpp')

    with open(hpp_path, 'w') as f:
        f.write(hpp)
    with open(cpp_path, 'w') as f:
        f.write(cpp)

    print(f'Generated {hpp_path}')
    print(f'Generated {cpp_path}')


if __name__ == '__main__':
    main()
