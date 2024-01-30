# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robotic_deposition: 2 messages, 0 services")

set(MSG_I_FLAGS "-Irobotic_deposition:/home/docker/bind_mount/src/robotic_deposition/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robotic_deposition_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_data.msg" NAME_WE)
add_custom_target(_robotic_deposition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotic_deposition" "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_data.msg" "std_msgs/Bool:std_msgs/Float64:std_msgs/Float64MultiArray:std_msgs/MultiArrayDimension:std_msgs/MultiArrayLayout"
)

get_filename_component(_filename "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_control.msg" NAME_WE)
add_custom_target(_robotic_deposition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotic_deposition" "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_control.msg" "std_msgs/MultiArrayLayout:std_msgs/String:std_msgs/Float64MultiArray:std_msgs/MultiArrayDimension"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robotic_deposition
  "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_data.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotic_deposition
)
_generate_msg_cpp(robotic_deposition
  "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotic_deposition
)

### Generating Services

### Generating Module File
_generate_module_cpp(robotic_deposition
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotic_deposition
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robotic_deposition_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robotic_deposition_generate_messages robotic_deposition_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_data.msg" NAME_WE)
add_dependencies(robotic_deposition_generate_messages_cpp _robotic_deposition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_control.msg" NAME_WE)
add_dependencies(robotic_deposition_generate_messages_cpp _robotic_deposition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotic_deposition_gencpp)
add_dependencies(robotic_deposition_gencpp robotic_deposition_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotic_deposition_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robotic_deposition
  "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_data.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotic_deposition
)
_generate_msg_eus(robotic_deposition
  "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotic_deposition
)

### Generating Services

### Generating Module File
_generate_module_eus(robotic_deposition
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotic_deposition
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robotic_deposition_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robotic_deposition_generate_messages robotic_deposition_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_data.msg" NAME_WE)
add_dependencies(robotic_deposition_generate_messages_eus _robotic_deposition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_control.msg" NAME_WE)
add_dependencies(robotic_deposition_generate_messages_eus _robotic_deposition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotic_deposition_geneus)
add_dependencies(robotic_deposition_geneus robotic_deposition_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotic_deposition_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robotic_deposition
  "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_data.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotic_deposition
)
_generate_msg_lisp(robotic_deposition
  "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotic_deposition
)

### Generating Services

### Generating Module File
_generate_module_lisp(robotic_deposition
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotic_deposition
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robotic_deposition_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robotic_deposition_generate_messages robotic_deposition_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_data.msg" NAME_WE)
add_dependencies(robotic_deposition_generate_messages_lisp _robotic_deposition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_control.msg" NAME_WE)
add_dependencies(robotic_deposition_generate_messages_lisp _robotic_deposition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotic_deposition_genlisp)
add_dependencies(robotic_deposition_genlisp robotic_deposition_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotic_deposition_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robotic_deposition
  "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_data.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotic_deposition
)
_generate_msg_nodejs(robotic_deposition
  "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotic_deposition
)

### Generating Services

### Generating Module File
_generate_module_nodejs(robotic_deposition
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotic_deposition
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robotic_deposition_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robotic_deposition_generate_messages robotic_deposition_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_data.msg" NAME_WE)
add_dependencies(robotic_deposition_generate_messages_nodejs _robotic_deposition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_control.msg" NAME_WE)
add_dependencies(robotic_deposition_generate_messages_nodejs _robotic_deposition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotic_deposition_gennodejs)
add_dependencies(robotic_deposition_gennodejs robotic_deposition_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotic_deposition_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robotic_deposition
  "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_data.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotic_deposition
)
_generate_msg_py(robotic_deposition
  "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotic_deposition
)

### Generating Services

### Generating Module File
_generate_module_py(robotic_deposition
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotic_deposition
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robotic_deposition_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robotic_deposition_generate_messages robotic_deposition_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_data.msg" NAME_WE)
add_dependencies(robotic_deposition_generate_messages_py _robotic_deposition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/docker/bind_mount/src/robotic_deposition/msg/ur5e_control.msg" NAME_WE)
add_dependencies(robotic_deposition_generate_messages_py _robotic_deposition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotic_deposition_genpy)
add_dependencies(robotic_deposition_genpy robotic_deposition_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotic_deposition_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotic_deposition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotic_deposition
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robotic_deposition_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotic_deposition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotic_deposition
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robotic_deposition_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotic_deposition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotic_deposition
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robotic_deposition_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotic_deposition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotic_deposition
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robotic_deposition_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotic_deposition)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3.7\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotic_deposition\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotic_deposition
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robotic_deposition_generate_messages_py std_msgs_generate_messages_py)
endif()
