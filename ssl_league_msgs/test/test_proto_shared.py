"""Unit tests for the pure helper functions in proto_shared.py."""

from google.protobuf import descriptor_pb2
from proto_shared import (
    build_map_entry_type_names,
    classify_field_shape,
    consumed_annotation_fields,
    flatten_type_name,
    iter_messages,
    output_proto_fields,
    parse_options,
)

FD = descriptor_pb2.FieldDescriptorProto
DP = descriptor_pb2.DescriptorProto


def test_parse_options_empty():
    assert parse_options('') == {}


def test_parse_options_multiple_keys():
    assert parse_options('a=1,b=2') == {'a': '1', 'b': '2'}


def test_parse_options_ignores_entries_without_equals():
    assert parse_options('a=1,noeq,b=2') == {'a': '1', 'b': '2'}


def test_flatten_type_name_examples():
    assert flatten_type_name('.ateam.BasicControl') == 'BasicControl'
    assert flatten_type_name('.GameEvent.BallLeftField') == 'GameEventBallLeftField'
    assert flatten_type_name('.ateam_test.OuterMessage.Inner') == 'OuterMessageInner'


def test_classify_field_shape_repeated():
    field = FD(name='items', label=FD.LABEL_REPEATED)
    shape = classify_field_shape(field, proto2=False)
    assert shape.is_repeated
    assert not shape.in_oneof
    assert not shape.is_proto2_optional


def test_classify_field_shape_oneof_member():
    field = FD(name='choice', label=FD.LABEL_OPTIONAL, oneof_index=0)
    shape = classify_field_shape(field, proto2=True)
    assert not shape.is_repeated
    assert shape.in_oneof
    # A oneof member is never treated as a proto2-optional array, even
    # though the label matches — presence is already modeled by the case enum.
    assert not shape.is_proto2_optional


def test_classify_field_shape_proto2_optional():
    field = FD(name='maybe', label=FD.LABEL_OPTIONAL)
    shape = classify_field_shape(field, proto2=True)
    assert not shape.is_repeated
    assert not shape.in_oneof
    assert shape.is_proto2_optional


def test_classify_field_shape_proto3_singular_is_not_proto2_optional():
    # Same descriptor shape as the proto2-optional case, but proto2=False —
    # the caller's syntax flag gates this, not the label alone.
    field = FD(name='maybe', label=FD.LABEL_OPTIONAL)
    shape = classify_field_shape(field, proto2=False)
    assert not shape.is_proto2_optional


def test_output_proto_fields_from_and_components():
    out = {
        'from': {'x': 'proto_x'},
        'position': {'from': {'x': 'tx', 'y': 'ty'}},
        'orientation': {'from': {'x': 'q0'}},
    }
    assert set(output_proto_fields(out)) == {'proto_x', 'tx', 'ty', 'q0'}


def test_consumed_annotation_fields_combines_fields_and_outputs():
    entry = {
        'fields': {'renamed_field': {'ros_field': 'renamed'}},
        'outputs': [{'ros_field': 'point', 'from': {'x': 'px', 'y': 'py'}}],
    }
    assert consumed_annotation_fields(entry) == {'renamed_field', 'px', 'py'}


def test_build_map_entry_type_names_finds_nested_map_entry():
    map_entry = DP(name='FooEntry')
    map_entry.options.map_entry = True
    msg = DP(name='Foo', nested_type=[map_entry])
    fd = descriptor_pb2.FileDescriptorProto(package='testpkg', message_type=[msg])

    result = build_map_entry_type_names([fd])

    assert result == frozenset({'.testpkg.Foo.FooEntry'})


def test_iter_messages_flattens_nested_names_and_strips_ssl_prefix():
    inner = DP(name='Inner')
    outer = DP(name='SSL_Outer', nested_type=[inner])
    fd = descriptor_pb2.FileDescriptorProto(package='testpkg', message_type=[outer])

    results = {flat: cpp for flat, cpp, _msg in iter_messages(fd)}

    assert results == {'Outer': 'SSL_Outer', 'OuterInner': 'SSL_Outer_Inner'}
