idf_component_register(
  SRCS
    "main.cpp"
    "encoder.cpp"
    "motor.cpp"
    "serial_link.cpp"
    "joint_controller.cpp"
    "control_task.cpp"
    "comms_task.cpp"
  INCLUDE_DIRS
    "."
    "${ARM_COMMON_INCLUDE_DIR}"
  REQUIRES
    driver
    esp_timer
    log)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
