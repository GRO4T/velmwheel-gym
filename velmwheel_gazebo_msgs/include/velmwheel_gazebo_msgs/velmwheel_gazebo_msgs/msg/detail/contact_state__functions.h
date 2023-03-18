// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from velmwheel_gazebo_msgs:msg/ContactState.idl
// generated code does not contain a copyright notice

#ifndef VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__FUNCTIONS_H_
#define VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "velmwheel_gazebo_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "velmwheel_gazebo_msgs/msg/detail/contact_state__struct.h"

/// Initialize msg/ContactState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * velmwheel_gazebo_msgs__msg__ContactState
 * )) before or use
 * velmwheel_gazebo_msgs__msg__ContactState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__msg__ContactState__init(velmwheel_gazebo_msgs__msg__ContactState * msg);

/// Finalize msg/ContactState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
void
velmwheel_gazebo_msgs__msg__ContactState__fini(velmwheel_gazebo_msgs__msg__ContactState * msg);

/// Create msg/ContactState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * velmwheel_gazebo_msgs__msg__ContactState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
velmwheel_gazebo_msgs__msg__ContactState *
velmwheel_gazebo_msgs__msg__ContactState__create();

/// Destroy msg/ContactState message.
/**
 * It calls
 * velmwheel_gazebo_msgs__msg__ContactState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
void
velmwheel_gazebo_msgs__msg__ContactState__destroy(velmwheel_gazebo_msgs__msg__ContactState * msg);

/// Check for msg/ContactState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__msg__ContactState__are_equal(const velmwheel_gazebo_msgs__msg__ContactState * lhs, const velmwheel_gazebo_msgs__msg__ContactState * rhs);

/// Copy a msg/ContactState message.
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
velmwheel_gazebo_msgs__msg__ContactState__copy(
  const velmwheel_gazebo_msgs__msg__ContactState * input,
  velmwheel_gazebo_msgs__msg__ContactState * output);

/// Initialize array of msg/ContactState messages.
/**
 * It allocates the memory for the number of elements and calls
 * velmwheel_gazebo_msgs__msg__ContactState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__msg__ContactState__Sequence__init(velmwheel_gazebo_msgs__msg__ContactState__Sequence * array, size_t size);

/// Finalize array of msg/ContactState messages.
/**
 * It calls
 * velmwheel_gazebo_msgs__msg__ContactState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
void
velmwheel_gazebo_msgs__msg__ContactState__Sequence__fini(velmwheel_gazebo_msgs__msg__ContactState__Sequence * array);

/// Create array of msg/ContactState messages.
/**
 * It allocates the memory for the array and calls
 * velmwheel_gazebo_msgs__msg__ContactState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
velmwheel_gazebo_msgs__msg__ContactState__Sequence *
velmwheel_gazebo_msgs__msg__ContactState__Sequence__create(size_t size);

/// Destroy array of msg/ContactState messages.
/**
 * It calls
 * velmwheel_gazebo_msgs__msg__ContactState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
void
velmwheel_gazebo_msgs__msg__ContactState__Sequence__destroy(velmwheel_gazebo_msgs__msg__ContactState__Sequence * array);

/// Check for msg/ContactState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_velmwheel_gazebo_msgs
bool
velmwheel_gazebo_msgs__msg__ContactState__Sequence__are_equal(const velmwheel_gazebo_msgs__msg__ContactState__Sequence * lhs, const velmwheel_gazebo_msgs__msg__ContactState__Sequence * rhs);

/// Copy an array of msg/ContactState messages.
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
velmwheel_gazebo_msgs__msg__ContactState__Sequence__copy(
  const velmwheel_gazebo_msgs__msg__ContactState__Sequence * input,
  velmwheel_gazebo_msgs__msg__ContactState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // VELMWHEEL_GAZEBO_MSGS__MSG__DETAIL__CONTACT_STATE__FUNCTIONS_H_
