# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(WARNING "Invoking generate_messages() without having added any message or service file before.
You should either add add_message_files() and/or add_service_files() calls or remove the invocation of generate_messages().")
message(STATUS "barcode_functions: 0 messages, 0 services")

set(MSG_I_FLAGS "-Icustom_msgs:/home/greenbot/catkin_ws/src/custom_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(barcode_functions_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_cpp(barcode_functions
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barcode_functions
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(barcode_functions_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(barcode_functions_generate_messages barcode_functions_generate_messages_cpp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(barcode_functions_gencpp)
add_dependencies(barcode_functions_gencpp barcode_functions_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barcode_functions_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_eus(barcode_functions
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barcode_functions
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(barcode_functions_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(barcode_functions_generate_messages barcode_functions_generate_messages_eus)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(barcode_functions_geneus)
add_dependencies(barcode_functions_geneus barcode_functions_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barcode_functions_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_lisp(barcode_functions
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barcode_functions
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(barcode_functions_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(barcode_functions_generate_messages barcode_functions_generate_messages_lisp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(barcode_functions_genlisp)
add_dependencies(barcode_functions_genlisp barcode_functions_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barcode_functions_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_nodejs(barcode_functions
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barcode_functions
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(barcode_functions_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(barcode_functions_generate_messages barcode_functions_generate_messages_nodejs)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(barcode_functions_gennodejs)
add_dependencies(barcode_functions_gennodejs barcode_functions_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barcode_functions_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_py(barcode_functions
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barcode_functions
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(barcode_functions_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(barcode_functions_generate_messages barcode_functions_generate_messages_py)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(barcode_functions_genpy)
add_dependencies(barcode_functions_genpy barcode_functions_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barcode_functions_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barcode_functions)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barcode_functions
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET custom_msgs_generate_messages_cpp)
  add_dependencies(barcode_functions_generate_messages_cpp custom_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barcode_functions)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barcode_functions
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET custom_msgs_generate_messages_eus)
  add_dependencies(barcode_functions_generate_messages_eus custom_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barcode_functions)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barcode_functions
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET custom_msgs_generate_messages_lisp)
  add_dependencies(barcode_functions_generate_messages_lisp custom_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barcode_functions)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barcode_functions
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET custom_msgs_generate_messages_nodejs)
  add_dependencies(barcode_functions_generate_messages_nodejs custom_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barcode_functions)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barcode_functions\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barcode_functions
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET custom_msgs_generate_messages_py)
  add_dependencies(barcode_functions_generate_messages_py custom_msgs_generate_messages_py)
endif()
