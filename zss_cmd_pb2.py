# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: zss_cmd.proto
# Protobuf Python Version: 4.25-main
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\rzss_cmd.proto\x12\x0cZSS.Protocol\"B\n\rRobots_Status\x12\x31\n\rrobots_status\x18\x01 \x03(\x0b\x32\x1a.ZSS.Protocol.Robot_Status\"X\n\x0cRobot_Status\x12\x10\n\x08robot_id\x18\x01 \x02(\x05\x12\x10\n\x08infrared\x18\x02 \x02(\x08\x12\x11\n\tflat_kick\x18\x03 \x02(\x08\x12\x11\n\tchip_kick\x18\x04 \x02(\x08\"M\n\x0eRobots_Command\x12,\n\x07\x63ommand\x18\x01 \x03(\x0b\x32\x1b.ZSS.Protocol.Robot_Command\x12\r\n\x05\x64\x65lay\x18\x02 \x01(\x05\"\xbe\x01\n\rRobot_Command\x12\x10\n\x08robot_id\x18\x01 \x02(\x05\x12\x12\n\nvelocity_x\x18\x02 \x02(\x02\x12\x12\n\nvelocity_y\x18\x03 \x02(\x02\x12\x12\n\nvelocity_r\x18\x04 \x02(\x02\x12\x0c\n\x04kick\x18\x05 \x02(\x08\x12\r\n\x05power\x18\x06 \x02(\x02\x12\x15\n\rdribbler_spin\x18\x07 \x02(\x02\x12\x15\n\rcurrent_angle\x18\x08 \x01(\x02\x12\x14\n\x0ctarget_angle\x18\t \x01(\x02')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'zss_cmd_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_ROBOTS_STATUS']._serialized_start=31
  _globals['_ROBOTS_STATUS']._serialized_end=97
  _globals['_ROBOT_STATUS']._serialized_start=99
  _globals['_ROBOT_STATUS']._serialized_end=187
  _globals['_ROBOTS_COMMAND']._serialized_start=189
  _globals['_ROBOTS_COMMAND']._serialized_end=266
  _globals['_ROBOT_COMMAND']._serialized_start=269
  _globals['_ROBOT_COMMAND']._serialized_end=459
# @@protoc_insertion_point(module_scope)
