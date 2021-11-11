# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "bug0: 2 messages, 4 services")

set(MSG_I_FLAGS "-Ibug0:/home/phineas/ros_ws/src/bug0/msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(bug0_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/msg/NavError.msg" NAME_WE)
add_custom_target(_bug0_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bug0" "/home/phineas/ros_ws/src/bug0/msg/NavError.msg" ""
)

get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg" NAME_WE)
add_custom_target(_bug0_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bug0" "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg" ""
)

get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/UpdateGoal.srv" NAME_WE)
add_custom_target(_bug0_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bug0" "/home/phineas/ros_ws/src/bug0/srv/UpdateGoal.srv" "geometry_msgs/Point32"
)

get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetVel.srv" NAME_WE)
add_custom_target(_bug0_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bug0" "/home/phineas/ros_ws/src/bug0/srv/SetVel.srv" ""
)

get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetMaxVel.srv" NAME_WE)
add_custom_target(_bug0_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bug0" "/home/phineas/ros_ws/src/bug0/srv/SetMaxVel.srv" ""
)

get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetGains.srv" NAME_WE)
add_custom_target(_bug0_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bug0" "/home/phineas/ros_ws/src/bug0/srv/SetGains.srv" "bug0/PIDGains"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(bug0
  "/home/phineas/ros_ws/src/bug0/msg/NavError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bug0
)
_generate_msg_cpp(bug0
  "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bug0
)

### Generating Services
_generate_srv_cpp(bug0
  "/home/phineas/ros_ws/src/bug0/srv/UpdateGoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bug0
)
_generate_srv_cpp(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bug0
)
_generate_srv_cpp(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetMaxVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bug0
)
_generate_srv_cpp(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetGains.srv"
  "${MSG_I_FLAGS}"
  "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bug0
)

### Generating Module File
_generate_module_cpp(bug0
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bug0
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(bug0_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(bug0_generate_messages bug0_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/msg/NavError.msg" NAME_WE)
add_dependencies(bug0_generate_messages_cpp _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg" NAME_WE)
add_dependencies(bug0_generate_messages_cpp _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/UpdateGoal.srv" NAME_WE)
add_dependencies(bug0_generate_messages_cpp _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetVel.srv" NAME_WE)
add_dependencies(bug0_generate_messages_cpp _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetMaxVel.srv" NAME_WE)
add_dependencies(bug0_generate_messages_cpp _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetGains.srv" NAME_WE)
add_dependencies(bug0_generate_messages_cpp _bug0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bug0_gencpp)
add_dependencies(bug0_gencpp bug0_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bug0_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(bug0
  "/home/phineas/ros_ws/src/bug0/msg/NavError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bug0
)
_generate_msg_eus(bug0
  "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bug0
)

### Generating Services
_generate_srv_eus(bug0
  "/home/phineas/ros_ws/src/bug0/srv/UpdateGoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bug0
)
_generate_srv_eus(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bug0
)
_generate_srv_eus(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetMaxVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bug0
)
_generate_srv_eus(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetGains.srv"
  "${MSG_I_FLAGS}"
  "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bug0
)

### Generating Module File
_generate_module_eus(bug0
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bug0
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(bug0_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(bug0_generate_messages bug0_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/msg/NavError.msg" NAME_WE)
add_dependencies(bug0_generate_messages_eus _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg" NAME_WE)
add_dependencies(bug0_generate_messages_eus _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/UpdateGoal.srv" NAME_WE)
add_dependencies(bug0_generate_messages_eus _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetVel.srv" NAME_WE)
add_dependencies(bug0_generate_messages_eus _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetMaxVel.srv" NAME_WE)
add_dependencies(bug0_generate_messages_eus _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetGains.srv" NAME_WE)
add_dependencies(bug0_generate_messages_eus _bug0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bug0_geneus)
add_dependencies(bug0_geneus bug0_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bug0_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(bug0
  "/home/phineas/ros_ws/src/bug0/msg/NavError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bug0
)
_generate_msg_lisp(bug0
  "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bug0
)

### Generating Services
_generate_srv_lisp(bug0
  "/home/phineas/ros_ws/src/bug0/srv/UpdateGoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bug0
)
_generate_srv_lisp(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bug0
)
_generate_srv_lisp(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetMaxVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bug0
)
_generate_srv_lisp(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetGains.srv"
  "${MSG_I_FLAGS}"
  "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bug0
)

### Generating Module File
_generate_module_lisp(bug0
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bug0
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(bug0_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(bug0_generate_messages bug0_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/msg/NavError.msg" NAME_WE)
add_dependencies(bug0_generate_messages_lisp _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg" NAME_WE)
add_dependencies(bug0_generate_messages_lisp _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/UpdateGoal.srv" NAME_WE)
add_dependencies(bug0_generate_messages_lisp _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetVel.srv" NAME_WE)
add_dependencies(bug0_generate_messages_lisp _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetMaxVel.srv" NAME_WE)
add_dependencies(bug0_generate_messages_lisp _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetGains.srv" NAME_WE)
add_dependencies(bug0_generate_messages_lisp _bug0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bug0_genlisp)
add_dependencies(bug0_genlisp bug0_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bug0_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(bug0
  "/home/phineas/ros_ws/src/bug0/msg/NavError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bug0
)
_generate_msg_nodejs(bug0
  "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bug0
)

### Generating Services
_generate_srv_nodejs(bug0
  "/home/phineas/ros_ws/src/bug0/srv/UpdateGoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bug0
)
_generate_srv_nodejs(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bug0
)
_generate_srv_nodejs(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetMaxVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bug0
)
_generate_srv_nodejs(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetGains.srv"
  "${MSG_I_FLAGS}"
  "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bug0
)

### Generating Module File
_generate_module_nodejs(bug0
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bug0
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(bug0_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(bug0_generate_messages bug0_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/msg/NavError.msg" NAME_WE)
add_dependencies(bug0_generate_messages_nodejs _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg" NAME_WE)
add_dependencies(bug0_generate_messages_nodejs _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/UpdateGoal.srv" NAME_WE)
add_dependencies(bug0_generate_messages_nodejs _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetVel.srv" NAME_WE)
add_dependencies(bug0_generate_messages_nodejs _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetMaxVel.srv" NAME_WE)
add_dependencies(bug0_generate_messages_nodejs _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetGains.srv" NAME_WE)
add_dependencies(bug0_generate_messages_nodejs _bug0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bug0_gennodejs)
add_dependencies(bug0_gennodejs bug0_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bug0_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(bug0
  "/home/phineas/ros_ws/src/bug0/msg/NavError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bug0
)
_generate_msg_py(bug0
  "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bug0
)

### Generating Services
_generate_srv_py(bug0
  "/home/phineas/ros_ws/src/bug0/srv/UpdateGoal.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bug0
)
_generate_srv_py(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bug0
)
_generate_srv_py(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetMaxVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bug0
)
_generate_srv_py(bug0
  "/home/phineas/ros_ws/src/bug0/srv/SetGains.srv"
  "${MSG_I_FLAGS}"
  "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bug0
)

### Generating Module File
_generate_module_py(bug0
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bug0
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(bug0_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(bug0_generate_messages bug0_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/msg/NavError.msg" NAME_WE)
add_dependencies(bug0_generate_messages_py _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/msg/PIDGains.msg" NAME_WE)
add_dependencies(bug0_generate_messages_py _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/UpdateGoal.srv" NAME_WE)
add_dependencies(bug0_generate_messages_py _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetVel.srv" NAME_WE)
add_dependencies(bug0_generate_messages_py _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetMaxVel.srv" NAME_WE)
add_dependencies(bug0_generate_messages_py _bug0_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phineas/ros_ws/src/bug0/srv/SetGains.srv" NAME_WE)
add_dependencies(bug0_generate_messages_py _bug0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bug0_genpy)
add_dependencies(bug0_genpy bug0_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bug0_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bug0)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bug0
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(bug0_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(bug0_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(bug0_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(bug0_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bug0)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bug0
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(bug0_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(bug0_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(bug0_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(bug0_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bug0)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bug0
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(bug0_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(bug0_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(bug0_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(bug0_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bug0)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bug0
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(bug0_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(bug0_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(bug0_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(bug0_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bug0)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bug0\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bug0
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(bug0_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(bug0_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(bug0_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(bug0_generate_messages_py geometry_msgs_generate_messages_py)
endif()
