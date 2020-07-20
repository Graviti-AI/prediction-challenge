# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simulator.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='simulator.proto',
  package='service',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0fsimulator.proto\x12\x07service\"\xb3\x01\n\x05State\x12\x10\n\x08track_id\x18\x01 \x01(\x04\x12\x10\n\x08\x66rame_id\x18\x02 \x01(\x04\x12\x14\n\x0ctimestamp_ms\x18\x03 \x01(\x04\x12\x12\n\nagent_type\x18\x04 \x01(\t\x12\t\n\x01x\x18\x05 \x01(\x01\x12\t\n\x01y\x18\x06 \x01(\x01\x12\n\n\x02vx\x18\x07 \x01(\x01\x12\n\n\x02vy\x18\x08 \x01(\x01\x12\x0f\n\x07psi_rad\x18\t \x01(\x01\x12\x0e\n\x06length\x18\n \x01(\x01\x12\r\n\x05width\x18\x0b \x01(\x01\"+\n\nTrajectory\x12\x1d\n\x05state\x18\x01 \x03(\x0b\x32\x0e.service.State\"\x11\n\x0f\x46\x65tchEnvRequest\"[\n\x10\x46\x65tchEnvResponse\x12\x11\n\tresp_code\x18\x01 \x01(\x05\x12\x0b\n\x03msg\x18\x02 \x01(\t\x12\'\n\ntrajectory\x18\x03 \x01(\x0b\x32\x13.service.Trajectory\"B\n\x17PushMyTrajectoryRequest\x12\'\n\ntrajectory\x18\x03 \x01(\x0b\x32\x13.service.Trajectory\":\n\x18PushMyTrajectoryResponse\x12\x11\n\tresp_code\x18\x01 \x01(\x05\x12\x0b\n\x03msg\x18\x02 \x01(\t2\xaf\x01\n\x0fSimulatorServer\x12\x41\n\x08\x46\x65tchEnv\x12\x18.service.FetchEnvRequest\x1a\x19.service.FetchEnvResponse\"\x00\x12Y\n\x10PushMyTrajectory\x12 .service.PushMyTrajectoryRequest\x1a!.service.PushMyTrajectoryResponse\"\x00\x62\x06proto3'
)




_STATE = _descriptor.Descriptor(
  name='State',
  full_name='service.State',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='track_id', full_name='service.State.track_id', index=0,
      number=1, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='frame_id', full_name='service.State.frame_id', index=1,
      number=2, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='timestamp_ms', full_name='service.State.timestamp_ms', index=2,
      number=3, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='agent_type', full_name='service.State.agent_type', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='x', full_name='service.State.x', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y', full_name='service.State.y', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='vx', full_name='service.State.vx', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='vy', full_name='service.State.vy', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='psi_rad', full_name='service.State.psi_rad', index=8,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='length', full_name='service.State.length', index=9,
      number=10, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='width', full_name='service.State.width', index=10,
      number=11, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_start=29,
  serialized_end=208,
)


_TRAJECTORY = _descriptor.Descriptor(
  name='Trajectory',
  full_name='service.Trajectory',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='state', full_name='service.Trajectory.state', index=0,
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
  serialized_start=210,
  serialized_end=253,
)


_FETCHENVREQUEST = _descriptor.Descriptor(
  name='FetchEnvRequest',
  full_name='service.FetchEnvRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
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
  serialized_start=255,
  serialized_end=272,
)


_FETCHENVRESPONSE = _descriptor.Descriptor(
  name='FetchEnvResponse',
  full_name='service.FetchEnvResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='resp_code', full_name='service.FetchEnvResponse.resp_code', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='msg', full_name='service.FetchEnvResponse.msg', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='trajectory', full_name='service.FetchEnvResponse.trajectory', index=2,
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
  serialized_start=274,
  serialized_end=365,
)


_PUSHMYTRAJECTORYREQUEST = _descriptor.Descriptor(
  name='PushMyTrajectoryRequest',
  full_name='service.PushMyTrajectoryRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='trajectory', full_name='service.PushMyTrajectoryRequest.trajectory', index=0,
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
  serialized_start=367,
  serialized_end=433,
)


_PUSHMYTRAJECTORYRESPONSE = _descriptor.Descriptor(
  name='PushMyTrajectoryResponse',
  full_name='service.PushMyTrajectoryResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='resp_code', full_name='service.PushMyTrajectoryResponse.resp_code', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='msg', full_name='service.PushMyTrajectoryResponse.msg', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
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
  serialized_start=435,
  serialized_end=493,
)

_TRAJECTORY.fields_by_name['state'].message_type = _STATE
_FETCHENVRESPONSE.fields_by_name['trajectory'].message_type = _TRAJECTORY
_PUSHMYTRAJECTORYREQUEST.fields_by_name['trajectory'].message_type = _TRAJECTORY
DESCRIPTOR.message_types_by_name['State'] = _STATE
DESCRIPTOR.message_types_by_name['Trajectory'] = _TRAJECTORY
DESCRIPTOR.message_types_by_name['FetchEnvRequest'] = _FETCHENVREQUEST
DESCRIPTOR.message_types_by_name['FetchEnvResponse'] = _FETCHENVRESPONSE
DESCRIPTOR.message_types_by_name['PushMyTrajectoryRequest'] = _PUSHMYTRAJECTORYREQUEST
DESCRIPTOR.message_types_by_name['PushMyTrajectoryResponse'] = _PUSHMYTRAJECTORYRESPONSE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

State = _reflection.GeneratedProtocolMessageType('State', (_message.Message,), {
  'DESCRIPTOR' : _STATE,
  '__module__' : 'simulator_pb2'
  # @@protoc_insertion_point(class_scope:service.State)
  })
_sym_db.RegisterMessage(State)

Trajectory = _reflection.GeneratedProtocolMessageType('Trajectory', (_message.Message,), {
  'DESCRIPTOR' : _TRAJECTORY,
  '__module__' : 'simulator_pb2'
  # @@protoc_insertion_point(class_scope:service.Trajectory)
  })
_sym_db.RegisterMessage(Trajectory)

FetchEnvRequest = _reflection.GeneratedProtocolMessageType('FetchEnvRequest', (_message.Message,), {
  'DESCRIPTOR' : _FETCHENVREQUEST,
  '__module__' : 'simulator_pb2'
  # @@protoc_insertion_point(class_scope:service.FetchEnvRequest)
  })
_sym_db.RegisterMessage(FetchEnvRequest)

FetchEnvResponse = _reflection.GeneratedProtocolMessageType('FetchEnvResponse', (_message.Message,), {
  'DESCRIPTOR' : _FETCHENVRESPONSE,
  '__module__' : 'simulator_pb2'
  # @@protoc_insertion_point(class_scope:service.FetchEnvResponse)
  })
_sym_db.RegisterMessage(FetchEnvResponse)

PushMyTrajectoryRequest = _reflection.GeneratedProtocolMessageType('PushMyTrajectoryRequest', (_message.Message,), {
  'DESCRIPTOR' : _PUSHMYTRAJECTORYREQUEST,
  '__module__' : 'simulator_pb2'
  # @@protoc_insertion_point(class_scope:service.PushMyTrajectoryRequest)
  })
_sym_db.RegisterMessage(PushMyTrajectoryRequest)

PushMyTrajectoryResponse = _reflection.GeneratedProtocolMessageType('PushMyTrajectoryResponse', (_message.Message,), {
  'DESCRIPTOR' : _PUSHMYTRAJECTORYRESPONSE,
  '__module__' : 'simulator_pb2'
  # @@protoc_insertion_point(class_scope:service.PushMyTrajectoryResponse)
  })
_sym_db.RegisterMessage(PushMyTrajectoryResponse)



_SIMULATORSERVER = _descriptor.ServiceDescriptor(
  name='SimulatorServer',
  full_name='service.SimulatorServer',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=496,
  serialized_end=671,
  methods=[
  _descriptor.MethodDescriptor(
    name='FetchEnv',
    full_name='service.SimulatorServer.FetchEnv',
    index=0,
    containing_service=None,
    input_type=_FETCHENVREQUEST,
    output_type=_FETCHENVRESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='PushMyTrajectory',
    full_name='service.SimulatorServer.PushMyTrajectory',
    index=1,
    containing_service=None,
    input_type=_PUSHMYTRAJECTORYREQUEST,
    output_type=_PUSHMYTRAJECTORYRESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_SIMULATORSERVER)

DESCRIPTOR.services_by_name['SimulatorServer'] = _SIMULATORSERVER

# @@protoc_insertion_point(module_scope)
