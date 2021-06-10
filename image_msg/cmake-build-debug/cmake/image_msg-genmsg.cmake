# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "image_msg: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iimage_msg:/home/haorui/catkin_ws/src/image_msg/msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(image_msg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/haorui/catkin_ws/src/image_msg/msg/ImageLocation.msg" NAME_WE)
add_custom_target(_image_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "image_msg" "/home/haorui/catkin_ws/src/image_msg/msg/ImageLocation.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(image_msg
  "/home/haorui/catkin_ws/src/image_msg/msg/ImageLocation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/image_msg
)

### Generating Services

### Generating Module File
_generate_module_cpp(image_msg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/image_msg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(image_msg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(image_msg_generate_messages image_msg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/haorui/catkin_ws/src/image_msg/msg/ImageLocation.msg" NAME_WE)
add_dependencies(image_msg_generate_messages_cpp _image_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(image_msg_gencpp)
add_dependencies(image_msg_gencpp image_msg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS image_msg_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(image_msg
  "/home/haorui/catkin_ws/src/image_msg/msg/ImageLocation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/image_msg
)

### Generating Services

### Generating Module File
_generate_module_lisp(image_msg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/image_msg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(image_msg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(image_msg_generate_messages image_msg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/haorui/catkin_ws/src/image_msg/msg/ImageLocation.msg" NAME_WE)
add_dependencies(image_msg_generate_messages_lisp _image_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(image_msg_genlisp)
add_dependencies(image_msg_genlisp image_msg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS image_msg_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(image_msg
  "/home/haorui/catkin_ws/src/image_msg/msg/ImageLocation.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/image_msg
)

### Generating Services

### Generating Module File
_generate_module_py(image_msg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/image_msg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(image_msg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(image_msg_generate_messages image_msg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/haorui/catkin_ws/src/image_msg/msg/ImageLocation.msg" NAME_WE)
add_dependencies(image_msg_generate_messages_py _image_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(image_msg_genpy)
add_dependencies(image_msg_genpy image_msg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS image_msg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/image_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/image_msg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(image_msg_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(image_msg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/image_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/image_msg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(image_msg_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(image_msg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/image_msg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/image_msg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/image_msg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(image_msg_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(image_msg_generate_messages_py std_msgs_generate_messages_py)
endif()
