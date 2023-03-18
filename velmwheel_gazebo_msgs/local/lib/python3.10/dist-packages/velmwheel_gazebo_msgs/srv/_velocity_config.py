# generated from rosidl_generator_py/resource/_idl.py.em
# with input from velmwheel_gazebo_msgs:srv/VelocityConfig.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VelocityConfig_Request(type):
    """Metaclass of message 'VelocityConfig_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('velmwheel_gazebo_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'velmwheel_gazebo_msgs.srv.VelocityConfig_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__velocity_config__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__velocity_config__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__velocity_config__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__velocity_config__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__velocity_config__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class VelocityConfig_Request(metaclass=Metaclass_VelocityConfig_Request):
    """Message class 'VelocityConfig_Request'."""

    __slots__ = [
        '_fr',
        '_fl',
        '_rl',
        '_rr',
    ]

    _fields_and_field_types = {
        'fr': 'double',
        'fl': 'double',
        'rl': 'double',
        'rr': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.fr = kwargs.get('fr', float())
        self.fl = kwargs.get('fl', float())
        self.rl = kwargs.get('rl', float())
        self.rr = kwargs.get('rr', float())

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
        if self.fr != other.fr:
            return False
        if self.fl != other.fl:
            return False
        if self.rl != other.rl:
            return False
        if self.rr != other.rr:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def fr(self):
        """Message field 'fr'."""
        return self._fr

    @fr.setter
    def fr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fr' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fr' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fr = value

    @builtins.property
    def fl(self):
        """Message field 'fl'."""
        return self._fl

    @fl.setter
    def fl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fl' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fl' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fl = value

    @builtins.property
    def rl(self):
        """Message field 'rl'."""
        return self._rl

    @rl.setter
    def rl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rl' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rl' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rl = value

    @builtins.property
    def rr(self):
        """Message field 'rr'."""
        return self._rr

    @rr.setter
    def rr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rr' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rr' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rr = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_VelocityConfig_Response(type):
    """Metaclass of message 'VelocityConfig_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('velmwheel_gazebo_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'velmwheel_gazebo_msgs.srv.VelocityConfig_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__velocity_config__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__velocity_config__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__velocity_config__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__velocity_config__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__velocity_config__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class VelocityConfig_Response(metaclass=Metaclass_VelocityConfig_Response):
    """Message class 'VelocityConfig_Response'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


class Metaclass_VelocityConfig(type):
    """Metaclass of service 'VelocityConfig'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('velmwheel_gazebo_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'velmwheel_gazebo_msgs.srv.VelocityConfig')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__velocity_config

            from velmwheel_gazebo_msgs.srv import _velocity_config
            if _velocity_config.Metaclass_VelocityConfig_Request._TYPE_SUPPORT is None:
                _velocity_config.Metaclass_VelocityConfig_Request.__import_type_support__()
            if _velocity_config.Metaclass_VelocityConfig_Response._TYPE_SUPPORT is None:
                _velocity_config.Metaclass_VelocityConfig_Response.__import_type_support__()


class VelocityConfig(metaclass=Metaclass_VelocityConfig):
    from velmwheel_gazebo_msgs.srv._velocity_config import VelocityConfig_Request as Request
    from velmwheel_gazebo_msgs.srv._velocity_config import VelocityConfig_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
