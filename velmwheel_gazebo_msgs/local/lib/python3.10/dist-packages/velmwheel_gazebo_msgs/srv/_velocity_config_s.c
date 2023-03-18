// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from velmwheel_gazebo_msgs:srv/VelocityConfig.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "velmwheel_gazebo_msgs/srv/detail/velocity_config__struct.h"
#include "velmwheel_gazebo_msgs/srv/detail/velocity_config__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool velmwheel_gazebo_msgs__srv__velocity_config__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[66];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("velmwheel_gazebo_msgs.srv._velocity_config.VelocityConfig_Request", full_classname_dest, 65) == 0);
  }
  velmwheel_gazebo_msgs__srv__VelocityConfig_Request * ros_message = _ros_message;
  {  // fr
    PyObject * field = PyObject_GetAttrString(_pymsg, "fr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fr = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fl
    PyObject * field = PyObject_GetAttrString(_pymsg, "fl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fl = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rl
    PyObject * field = PyObject_GetAttrString(_pymsg, "rl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rl = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rr
    PyObject * field = PyObject_GetAttrString(_pymsg, "rr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rr = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * velmwheel_gazebo_msgs__srv__velocity_config__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of VelocityConfig_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("velmwheel_gazebo_msgs.srv._velocity_config");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "VelocityConfig_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  velmwheel_gazebo_msgs__srv__VelocityConfig_Request * ros_message = (velmwheel_gazebo_msgs__srv__VelocityConfig_Request *)raw_ros_message;
  {  // fr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "velmwheel_gazebo_msgs/srv/detail/velocity_config__struct.h"
// already included above
// #include "velmwheel_gazebo_msgs/srv/detail/velocity_config__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool velmwheel_gazebo_msgs__srv__velocity_config__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[67];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("velmwheel_gazebo_msgs.srv._velocity_config.VelocityConfig_Response", full_classname_dest, 66) == 0);
  }
  velmwheel_gazebo_msgs__srv__VelocityConfig_Response * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * velmwheel_gazebo_msgs__srv__velocity_config__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of VelocityConfig_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("velmwheel_gazebo_msgs.srv._velocity_config");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "VelocityConfig_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  (void)raw_ros_message;

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
