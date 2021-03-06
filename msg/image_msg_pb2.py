# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: image_msg.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='image_msg.proto',
  package='image_msg',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0fimage_msg.proto\x12\timage_msg\"k\n\x05image\x12\x12\n\ntime_stamp\x18\x01 \x01(\x01\x12\x0e\n\x06height\x18\x02 \x01(\x05\x12\r\n\x05width\x18\x03 \x01(\x05\x12\x0f\n\x07\x63hannel\x18\x04 \x01(\x05\x12\x0c\n\x04size\x18\x05 \x01(\x05\x12\x10\n\x08mat_data\x18\x06 \x01(\x0c\"^\n\x04\x62\x62ox\x12\x12\n\nx_left_top\x18\x01 \x01(\x05\x12\x12\n\ny_left_top\x18\x02 \x01(\x05\x12\x16\n\x0ex_right_bottom\x18\x03 \x01(\x05\x12\x16\n\x0ey_right_bottom\x18\x04 \x01(\x05\"Z\n\rone_detection\x12\x12\n\nconfindece\x18\x01 \x01(\x02\x12\x12\n\nthis_class\x18\x02 \x01(\x05\x12!\n\x08one_bbox\x18\x03 \x01(\x0b\x32\x0f.image_msg.bbox\"r\n\ndetections\x12\x15\n\rdetected_flag\x18\x01 \x01(\x08\x12 \n\x06image_\x18\x02 \x03(\x0b\x32\x10.image_msg.image\x12+\n\tdetection\x18\x03 \x03(\x0b\x32\x18.image_msg.one_detection\"-\n\timage_buf\x12 \n\x06image_\x18\x01 \x03(\x0b\x32\x10.image_msg.imageb\x06proto3'
)




_IMAGE = _descriptor.Descriptor(
  name='image',
  full_name='image_msg.image',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='time_stamp', full_name='image_msg.image.time_stamp', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='height', full_name='image_msg.image.height', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='width', full_name='image_msg.image.width', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='channel', full_name='image_msg.image.channel', index=3,
      number=4, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='size', full_name='image_msg.image.size', index=4,
      number=5, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='mat_data', full_name='image_msg.image.mat_data', index=5,
      number=6, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=b"",
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=30,
  serialized_end=137,
)


_BBOX = _descriptor.Descriptor(
  name='bbox',
  full_name='image_msg.bbox',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='x_left_top', full_name='image_msg.bbox.x_left_top', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y_left_top', full_name='image_msg.bbox.y_left_top', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='x_right_bottom', full_name='image_msg.bbox.x_right_bottom', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y_right_bottom', full_name='image_msg.bbox.y_right_bottom', index=3,
      number=4, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=139,
  serialized_end=233,
)


_ONE_DETECTION = _descriptor.Descriptor(
  name='one_detection',
  full_name='image_msg.one_detection',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='confindece', full_name='image_msg.one_detection.confindece', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='this_class', full_name='image_msg.one_detection.this_class', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='one_bbox', full_name='image_msg.one_detection.one_bbox', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=235,
  serialized_end=325,
)


_DETECTIONS = _descriptor.Descriptor(
  name='detections',
  full_name='image_msg.detections',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='detected_flag', full_name='image_msg.detections.detected_flag', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='image_', full_name='image_msg.detections.image_', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='detection', full_name='image_msg.detections.detection', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=327,
  serialized_end=441,
)


_IMAGE_BUF = _descriptor.Descriptor(
  name='image_buf',
  full_name='image_msg.image_buf',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='image_', full_name='image_msg.image_buf.image_', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=443,
  serialized_end=488,
)

_ONE_DETECTION.fields_by_name['one_bbox'].message_type = _BBOX
_DETECTIONS.fields_by_name['image_'].message_type = _IMAGE
_DETECTIONS.fields_by_name['detection'].message_type = _ONE_DETECTION
_IMAGE_BUF.fields_by_name['image_'].message_type = _IMAGE
DESCRIPTOR.message_types_by_name['image'] = _IMAGE
DESCRIPTOR.message_types_by_name['bbox'] = _BBOX
DESCRIPTOR.message_types_by_name['one_detection'] = _ONE_DETECTION
DESCRIPTOR.message_types_by_name['detections'] = _DETECTIONS
DESCRIPTOR.message_types_by_name['image_buf'] = _IMAGE_BUF
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

image = _reflection.GeneratedProtocolMessageType('image', (_message.Message,), {
  'DESCRIPTOR' : _IMAGE,
  '__module__' : 'image_msg_pb2'
  # @@protoc_insertion_point(class_scope:image_msg.image)
  })
_sym_db.RegisterMessage(image)

bbox = _reflection.GeneratedProtocolMessageType('bbox', (_message.Message,), {
  'DESCRIPTOR' : _BBOX,
  '__module__' : 'image_msg_pb2'
  # @@protoc_insertion_point(class_scope:image_msg.bbox)
  })
_sym_db.RegisterMessage(bbox)

one_detection = _reflection.GeneratedProtocolMessageType('one_detection', (_message.Message,), {
  'DESCRIPTOR' : _ONE_DETECTION,
  '__module__' : 'image_msg_pb2'
  # @@protoc_insertion_point(class_scope:image_msg.one_detection)
  })
_sym_db.RegisterMessage(one_detection)

detections = _reflection.GeneratedProtocolMessageType('detections', (_message.Message,), {
  'DESCRIPTOR' : _DETECTIONS,
  '__module__' : 'image_msg_pb2'
  # @@protoc_insertion_point(class_scope:image_msg.detections)
  })
_sym_db.RegisterMessage(detections)

image_buf = _reflection.GeneratedProtocolMessageType('image_buf', (_message.Message,), {
  'DESCRIPTOR' : _IMAGE_BUF,
  '__module__' : 'image_msg_pb2'
  # @@protoc_insertion_point(class_scope:image_msg.image_buf)
  })
_sym_db.RegisterMessage(image_buf)


# @@protoc_insertion_point(module_scope)
