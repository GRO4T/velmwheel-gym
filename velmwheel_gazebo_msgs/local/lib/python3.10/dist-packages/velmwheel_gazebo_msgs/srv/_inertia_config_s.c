// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from velmwheel_gazebo_msgs:srv/InertiaConfig.idl
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
#include "velmwheel_gazebo_msgs/srv/detail/inertia_config__struct.h"
#include "velmwheel_gazebo_msgs/srv/detail/inertia_config__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__inertia__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__inertia__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__inertia__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__inertia__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__inertia__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__inertia__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool velmwheel_gazebo_msgs__srv__inertia_config__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[64];
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
    assert(strncmp("velmwheel_gazebo_msgs.srv._inertia_config.InertiaConfig_Request", full_classname_dest, 63) == 0);
  }
  velmwheel_gazebo_msgs__srv__InertiaConfig_Request * ros_message = _ros_message;
  {  // base
    PyObject * field = PyObject_GetAttrString(_pymsg, "base");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__inertia__convert_from_py(field, &ros_message->base)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // front
    PyObject * field = PyObject_GetAttrString(_pymsg, "front");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__inertia__convert_from_py(field, &ros_message->front)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // wheel
    PyObject * field = PyObject_GetAttrString(_pymsg, "wheel");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__inertia__convert_from_py(field, &ros_message->wheel)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * velmwheel_gazebo_msgs__srv__inertia_config__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of InertiaConfig_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("velmwheel_gazebo_msgs.srv._inertia_config");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "InertiaConfig_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  velmwheel_gazebo_msgs__srv__InertiaConfig_Request * ros_message = (velmwheel_gazebo_msgs__srv__InertiaConfig_Request *)raw_ros_message;
  {  // base
    PyObject * field = NULL;
    field = geometry_msgs__msg__inertia__convert_to_py(&ros_message->base);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "base", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // front
    PyObject * field = NULL;
    field = geometry_msgs__msg__inertia__convert_to_py(&ros_message->front);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "front", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wheel
    PyObject * field = NULL;
    field = geometry_msgs__msg__inertia__convert_to_py(&ros_message->wheel);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "wheel", field);
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
// #include "velmwheel_gazebo_msgs/srv/detail/inertia_config__struct.h"
// already included above
// #include "velmwheel_gazebo_msgs/srv/detail/inertia_config__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool velmwheel_gazebo_msgs__srv__inertia_config__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[65];
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
    assert(strncmp("velmwheel_gazebo_msgs.srv._inertia_config.InertiaConfig_Response", full_classname_dest, 64) == 0);
  }
  velmwheel_gazebo_msgs__srv__InertiaConfig_Response * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * velmwheel_gazebo_msgs__srv__inertia_config__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of InertiaConfig_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("velmwheel_gazebo_msgs.srv._inertia_config");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "InertiaConfig_Response");
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
