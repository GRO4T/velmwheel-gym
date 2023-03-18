# generated from rosidl_generator_py/resource/_idl.py.em
# with input from velmwheel_gazebo_msgs:srv/FrictionConfig.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_FrictionConfig_Request(type):
    """Metaclass of message 'FrictionConfig_Request'."""

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
                'velmwheel_gazebo_msgs.srv.FrictionConfig_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__friction_config__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__friction_config__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__friction_config__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__friction_config__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__friction_config__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class FrictionConfig_Request(metaclass=Metaclass_FrictionConfig_Request):
    """Message class 'FrictionConfig_Request'."""

    __slots__ = [
        '_mu1',
        '_mu2',
    ]

    _fields_and_field_types = {
        'mu1': 'double',
        'mu2': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mu1 = kwargs.get('mu1', float())
        self.mu2 = kwargs.get('mu2', float())

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
        if self.mu1 != other.mu1:
            return False
        if self.mu2 != other.mu2:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def mu1(self):
        """Message field 'mu1'."""
        return self._mu1

    @mu1.setter
    def mu1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'mu1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'mu1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._mu1 = value

    @builtins.property
    def mu2(self):
        """Message field 'mu2'."""
        return self._mu2

    @mu2.setter
    def mu2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'mu2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'mu2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._mu2 = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_FrictionConfig_Response(type):
    """Metaclass of message 'FrictionConfig_Response'."""

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
                'velmwheel_gazebo_msgs.srv.FrictionConfig_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__friction_config__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__friction_config__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__friction_config__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__friction_config__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__friction_config__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class FrictionConfig_Response(metaclass=Metaclass_FrictionConfig_Response):
    """Message class 'FrictionConfig_Response'."""

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


class Metaclass_FrictionConfig(type):
    """Metaclass of service 'FrictionConfig'."""

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
                'velmwheel_gazebo_msgs.srv.FrictionConfig')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__friction_config

            from velmwheel_gazebo_msgs.srv import _friction_config
            if _friction_config.Metaclass_FrictionConfig_Request._TYPE_SUPPORT is None:
                _friction_config.Metaclass_FrictionConfig_Request.__import_type_support__()
            if _friction_config.Metaclass_FrictionConfig_Response._TYPE_SUPPORT is None:
                _friction_config.Metaclass_FrictionConfig_Response.__import_type_support__()


class FrictionConfig(metaclass=Metaclass_FrictionConfig):
    from velmwheel_gazebo_msgs.srv._friction_config import FrictionConfig_Request as Request
    from velmwheel_gazebo_msgs.srv._friction_config import FrictionConfig_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
