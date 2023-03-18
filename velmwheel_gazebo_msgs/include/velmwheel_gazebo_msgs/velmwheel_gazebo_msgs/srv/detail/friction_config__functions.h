// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from velmwheel_gazebo_msgs:srv/FrictionConfig.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__FRICTION_CONFIG__FUNCTIONS_H_
#define VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__FRICTION_CONFIG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "velmwheel_gazebo_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "velmwheel_gazebo_msgs/srv/detail/friction_config__struct.h"

/// Initialize srv/FrictionConfig message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Request
 * )) before or use
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__srv__FrictionConfig_Request__init(velmwheel_gazebo_msgs__srv__FrictionConfig_Request * msg);

/// Finalize srv/FrictionConfig message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
void
velmwheel_gazebo_msgs__srv__FrictionConfig_Request__fini(velmwheel_gazebo_msgs__srv__FrictionConfig_Request * msg);

/// Create srv/FrictionConfig message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
velmwheel_gazebo_msgs__srv__FrictionConfig_Request *
velmwheel_gazebo_msgs__srv__FrictionConfig_Request__create();

/// Destroy srv/FrictionConfig message.
/**
 * It calls
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
void
velmwheel_gazebo_msgs__srv__FrictionConfig_Request__destroy(velmwheel_gazebo_msgs__srv__FrictionConfig_Request * msg);

/// Check for srv/FrictionConfig message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__srv__FrictionConfig_Request__are_equal(const velmwheel_gazebo_msgs__srv__FrictionConfig_Request * lhs, const velmwheel_gazebo_msgs__srv__FrictionConfig_Request * rhs);

/// Copy a srv/FrictionConfig message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__srv__FrictionConfig_Request__copy(
  const velmwheel_gazebo_msgs__srv__FrictionConfig_Request * input,
  velmwheel_gazebo_msgs__srv__FrictionConfig_Request * output);

/// Initialize array of srv/FrictionConfig messages.
/**
 * It allocates the memory for the number of elements and calls
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence__init(velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence * array, size_t size);

/// Finalize array of srv/FrictionConfig messages.
/**
 * It calls
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
void
velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence__fini(velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence * array);

/// Create array of srv/FrictionConfig messages.
/**
 * It allocates the memory for the array and calls
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence *
velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence__create(size_t size);

/// Destroy array of srv/FrictionConfig messages.
/**
 * It calls
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
void
velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence__destroy(velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence * array);

/// Check for srv/FrictionConfig message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence__are_equal(const velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence * lhs, const velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence * rhs);

/// Copy an array of srv/FrictionConfig messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence__copy(
  const velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence * input,
  velmwheel_gazebo_msgs__srv__FrictionConfig_Request__Sequence * output);

/// Initialize srv/FrictionConfig message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Response
 * )) before or use
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__srv__FrictionConfig_Response__init(velmwheel_gazebo_msgs__srv__FrictionConfig_Response * msg);

/// Finalize srv/FrictionConfig message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
void
velmwheel_gazebo_msgs__srv__FrictionConfig_Response__fini(velmwheel_gazebo_msgs__srv__FrictionConfig_Response * msg);

/// Create srv/FrictionConfig message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
velmwheel_gazebo_msgs__srv__FrictionConfig_Response *
velmwheel_gazebo_msgs__srv__FrictionConfig_Response__create();

/// Destroy srv/FrictionConfig message.
/**
 * It calls
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
void
velmwheel_gazebo_msgs__srv__FrictionConfig_Response__destroy(velmwheel_gazebo_msgs__srv__FrictionConfig_Response * msg);

/// Check for srv/FrictionConfig message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__srv__FrictionConfig_Response__are_equal(const velmwheel_gazebo_msgs__srv__FrictionConfig_Response * lhs, const velmwheel_gazebo_msgs__srv__FrictionConfig_Response * rhs);

/// Copy a srv/FrictionConfig message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__srv__FrictionConfig_Response__copy(
  const velmwheel_gazebo_msgs__srv__FrictionConfig_Response * input,
  velmwheel_gazebo_msgs__srv__FrictionConfig_Response * output);

/// Initialize array of srv/FrictionConfig messages.
/**
 * It allocates the memory for the number of elements and calls
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence__init(velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence * array, size_t size);

/// Finalize array of srv/FrictionConfig messages.
/**
 * It calls
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
void
velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence__fini(velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence * array);

/// Create array of srv/FrictionConfig messages.
/**
 * It allocates the memory for the array and calls
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence *
velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence__create(size_t size);

/// Destroy array of srv/FrictionConfig messages.
/**
 * It calls
 * velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
void
velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence__destroy(velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence * array);

/// Check for srv/FrictionConfig message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence__are_equal(const velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence * lhs, const velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence * rhs);

/// Copy an array of srv/FrictionConfig messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence__copy(
  const velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence * input,
  velmwheel_gazebo_msgs__srv__FrictionConfig_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // VELMWHEEL_GAZEBO_MSGS__SRV__DETAIL__FRICTION_CONFIG__FUNCTIONS_H_
