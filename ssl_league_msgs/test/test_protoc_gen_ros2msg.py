"""
Unit tests for protoc_gen_ros2msg.py's recursion detection and type mapping.

find_type_cycles is the highest-value target here: a real recursive proto
(GameEvent -> MultipleFouls -> GameEvent) previously had to be caught by
hand-comparing generated .msg output against the old hand-written messages.
These tests build small fake descriptors directly (no protoc invocation
needed) to pin down that behavior.
"""

from google.protobuf import descriptor_pb2
import protoc_gen_ros2msg as gen
import pytest

FD = descriptor_pb2.FieldDescriptorProto
DP = descriptor_pb2.DescriptorProto
FDP = descriptor_pb2.FileDescriptorProto


def _message_field(name, type_name):
    return FD(
        name=name, number=1, label=FD.LABEL_OPTIONAL, type=FD.TYPE_MESSAGE,
        type_name=type_name,
    )


def _two_message_file(a_fields=(), b_fields=()):
    """Build a FileDescriptorProto with two top-level messages, A and B, in package testpkg."""
    a = DP(name='A', field=list(a_fields))
    b = DP(name='B', field=list(b_fields))
    return FDP(name='test.proto', package='testpkg', message_type=[a, b])


def test_find_type_cycles_no_cycle():
    fd = _two_message_file(a_fields=[_message_field('b', '.testpkg.B')])
    all_files = {'test.proto': fd}

    errors = gen.find_type_cycles(all_files, ['test.proto'], {}, frozenset())

    assert errors == []


def test_find_type_cycles_direct_self_reference():
    fd = _two_message_file(a_fields=[_message_field('self_ref', '.testpkg.A')])
    all_files = {'test.proto': fd}

    errors = gen.find_type_cycles(all_files, ['test.proto'], {}, frozenset())

    assert len(errors) == 1
    assert 'A --[self_ref]--> A' in errors[0]


def test_find_type_cycles_indirect_cycle():
    fd = _two_message_file(
        a_fields=[_message_field('b', '.testpkg.B')],
        b_fields=[_message_field('a', '.testpkg.A')],
    )
    all_files = {'test.proto': fd}

    errors = gen.find_type_cycles(all_files, ['test.proto'], {}, frozenset())

    assert len(errors) == 1
    assert 'A --[b]--> B --[a]--> A' in errors[0]


def test_find_type_cycles_skip_annotation_breaks_the_cycle():
    fd = _two_message_file(
        a_fields=[_message_field('b', '.testpkg.B')],
        b_fields=[_message_field('a', '.testpkg.A')],
    )
    all_files = {'test.proto': fd}
    annotations = {'A': {'fields': {'b': {'skip': True}}}}

    errors = gen.find_type_cycles(all_files, ['test.proto'], annotations, frozenset())

    assert errors == []


def test_map_field_to_ros2_type_scalar():
    field = FD(name='n', type=FD.TYPE_INT32)
    assert gen.map_field_to_ros2_type(field) == 'int32'


def test_map_field_to_ros2_type_enum_is_uint8():
    field = FD(name='e', type=FD.TYPE_ENUM)
    assert gen.map_field_to_ros2_type(field) == 'uint8'


def test_map_field_to_ros2_type_rejects_any():
    field = FD(name='a', type=FD.TYPE_MESSAGE, type_name='.google.protobuf.Any')
    with pytest.raises(ValueError, match='Any'):
        gen.map_field_to_ros2_type(field)


def test_map_field_to_ros2_type_rejects_skipped_type(monkeypatch):
    monkeypatch.setattr(gen, '_SKIP_TYPES', frozenset({'Dropped'}))
    field = FD(name='d', type=FD.TYPE_MESSAGE, type_name='.testpkg.Dropped')
    with pytest.raises(ValueError, match='_skip_types'):
        gen.map_field_to_ros2_type(field)
