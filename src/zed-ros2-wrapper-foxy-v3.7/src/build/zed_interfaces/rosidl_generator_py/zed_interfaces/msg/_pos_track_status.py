# generated from rosidl_generator_py/resource/_idl.py.em
# with input from zed_interfaces:msg/PosTrackStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PosTrackStatus(type):
    """Metaclass of message 'PosTrackStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'OK': 0,
        'NOT_OK': 1,
        'LOOP_CLOSED': 1,
        'SEARCHING': 2,
        'OFF': 1,
        'CALIBRATION_IN_PROGRESS': 2,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('zed_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'zed_interfaces.msg.PosTrackStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__pos_track_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__pos_track_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__pos_track_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__pos_track_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__pos_track_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'OK': cls.__constants['OK'],
            'NOT_OK': cls.__constants['NOT_OK'],
            'LOOP_CLOSED': cls.__constants['LOOP_CLOSED'],
            'SEARCHING': cls.__constants['SEARCHING'],
            'OFF': cls.__constants['OFF'],
            'CALIBRATION_IN_PROGRESS': cls.__constants['CALIBRATION_IN_PROGRESS'],
        }

    @property
    def OK(self):
        """Message constant 'OK'."""
        return Metaclass_PosTrackStatus.__constants['OK']

    @property
    def NOT_OK(self):
        """Message constant 'NOT_OK'."""
        return Metaclass_PosTrackStatus.__constants['NOT_OK']

    @property
    def LOOP_CLOSED(self):
        """Message constant 'LOOP_CLOSED'."""
        return Metaclass_PosTrackStatus.__constants['LOOP_CLOSED']

    @property
    def SEARCHING(self):
        """Message constant 'SEARCHING'."""
        return Metaclass_PosTrackStatus.__constants['SEARCHING']

    @property
    def OFF(self):
        """Message constant 'OFF'."""
        return Metaclass_PosTrackStatus.__constants['OFF']

    @property
    def CALIBRATION_IN_PROGRESS(self):
        """Message constant 'CALIBRATION_IN_PROGRESS'."""
        return Metaclass_PosTrackStatus.__constants['CALIBRATION_IN_PROGRESS']


class PosTrackStatus(metaclass=Metaclass_PosTrackStatus):
    """
    Message class 'PosTrackStatus'.

    Constants:
      OK
      NOT_OK
      LOOP_CLOSED
      SEARCHING
      OFF
      CALIBRATION_IN_PROGRESS
    """

    __slots__ = [
        '_visual_odometry_tracking_status',
        '_map_tracking_status',
        '_imu_fusion_status',
        '_status',
    ]

    _fields_and_field_types = {
        'visual_odometry_tracking_status': 'uint8',
        'map_tracking_status': 'uint8',
        'imu_fusion_status': 'uint8',
        'status': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.visual_odometry_tracking_status = kwargs.get('visual_odometry_tracking_status', int())
        self.map_tracking_status = kwargs.get('map_tracking_status', int())
        self.imu_fusion_status = kwargs.get('imu_fusion_status', int())
        self.status = kwargs.get('status', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.visual_odometry_tracking_status != other.visual_odometry_tracking_status:
            return False
        if self.map_tracking_status != other.map_tracking_status:
            return False
        if self.imu_fusion_status != other.imu_fusion_status:
            return False
        if self.status != other.status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def visual_odometry_tracking_status(self):
        """Message field 'visual_odometry_tracking_status'."""
        return self._visual_odometry_tracking_status

    @visual_odometry_tracking_status.setter
    def visual_odometry_tracking_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'visual_odometry_tracking_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'visual_odometry_tracking_status' field must be an unsigned integer in [0, 255]"
        self._visual_odometry_tracking_status = value

    @property
    def map_tracking_status(self):
        """Message field 'map_tracking_status'."""
        return self._map_tracking_status

    @map_tracking_status.setter
    def map_tracking_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'map_tracking_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'map_tracking_status' field must be an unsigned integer in [0, 255]"
        self._map_tracking_status = value

    @property
    def imu_fusion_status(self):
        """Message field 'imu_fusion_status'."""
        return self._imu_fusion_status

    @imu_fusion_status.setter
    def imu_fusion_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'imu_fusion_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'imu_fusion_status' field must be an unsigned integer in [0, 255]"
        self._imu_fusion_status = value

    @property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'status' field must be an unsigned integer in [0, 255]"
        self._status = value
