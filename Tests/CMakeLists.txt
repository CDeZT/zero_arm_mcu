cmake_minimum_required(VERSION 3.22)

project(zero_arm_mcu_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(test_robot
    test_robot.c
    Stubs/cmsis_os2.h
    ../Robot/Inc/robot.h
    ../Robot/Inc/robot_state.h
    ../Robot/Inc/robot_types.h
    ../Robot/Src/robot.c
    ../Robot/Src/robot_state.c
)

target_include_directories(test_robot PRIVATE
    Stubs
    ../Config
    ../Robot/Inc
)

target_compile_options(test_robot PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(NAME test_robot COMMAND test_robot)

add_executable(test_X_V2
    test_X_V2.c
    ../Motor/Inc/X_V2.h
    ../Motor/Src/X_V2.c
)

target_include_directories(test_X_V2 PRIVATE
    Stubs
    ../Motor/Inc
    ../Platform/Inc
)

target_compile_options(test_X_V2 PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(NAME test_X_V2 COMMAND test_X_V2)

add_executable(test_platform_fdcan
    test_platform_fdcan.c
    Stubs/cmsis_os2.h
    Stubs/fdcan.h
    ../Platform/Inc/platform_fdcan.h
    ../Platform/Src/platform_fdcan.c
)

target_include_directories(test_platform_fdcan PRIVATE
    Stubs
    ../Platform/Inc
    ../Config
    ../Motor/Inc
)

target_compile_options(test_platform_fdcan PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_platform_fdcan
    COMMAND test_platform_fdcan
)

add_executable(test_platform_uart
    test_platform_uart.c
    Stubs/cmsis_os2.h
    Stubs/usart.h
    ../Config/build_config.h
    ../Platform/Inc/platform_uart.h
    ../Platform/Src/platform_uart.c
)

target_include_directories(test_platform_uart PRIVATE
    Stubs
    ../App/Inc
    ../Config
    ../Platform/Inc
)

target_compile_options(test_platform_uart PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_platform_uart
    COMMAND test_platform_uart
)

add_executable(test_app_rtos
    test_app_rtos.c
    Stubs/cmsis_os2.h
    ../App/Inc/app_rtos.h
    ../Config/app_events.h
)

target_include_directories(test_app_rtos PRIVATE
    Stubs
    ../App/Inc
    ../Config
)

target_compile_options(test_app_rtos PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_app_rtos
    COMMAND test_app_rtos
)

add_executable(test_app_host_tx
    test_app_host_tx.c
    Stubs/cmsis_os2.h
    ../App/Inc/app_host_tx.h
    ../App/Src/app_host_tx.c
)

target_include_directories(test_app_host_tx PRIVATE
    Stubs
    ../App/Inc
    ../Config
    ../Platform/Inc
    ../Protocol/Inc
    ../Robot/Inc
)

target_compile_options(test_app_host_tx PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_app_host_tx
    COMMAND test_app_host_tx
)

add_executable(test_motor_manager
    test_motor_manager.c
    Stubs/cmsis_os2.h
    ../Config/joint_config.h
    ../Config/joint_config.c
    ../Motor/Inc/motor_manager.h
    ../Motor/Inc/motor_types.h
    ../Motor/Inc/X_V2.h
    ../Motor/Src/motor_manager.c
    ../Motion/Inc/joint_transform.h
    ../Motion/Src/joint_transform.c
)

target_include_directories(test_motor_manager PRIVATE
    Stubs
    ../App/Inc
    ../Config
    ../Motor/Inc
    ../Motion/Inc
    ../Platform/Inc
    ../Robot/Inc
)

target_compile_options(test_motor_manager PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_motor_manager
    COMMAND test_motor_manager
)

add_executable(test_messages
    test_messages.c
    Stubs/cmsis_os2.h
    ../Protocol/Inc/messages.h
    ../Protocol/Inc/protocol.h
    ../Protocol/Src/messages.c
    ../Protocol/Src/protocol.c
    ../Gripper/Inc/feetech_sts.h
    ../Gripper/Src/feetech_sts.c
)

target_include_directories(test_messages PRIVATE
    Stubs
    ../App/Inc
    ../Config
    ../Gripper/Inc
    ../Motor/Inc
    ../Motion/Inc
    ../Protocol/Inc
    ../Robot/Inc
)

target_compile_options(test_messages PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_messages
    COMMAND test_messages
)

add_executable(test_joint_transform
    test_joint_transform.c
    ../Config/joint_config.h
    ../Config/joint_config.c
    ../Motion/Inc/joint_transform.h
    ../Motion/Src/joint_transform.c
)

target_include_directories(test_joint_transform PRIVATE
    ../Config
    ../Motion/Inc
)

target_compile_options(test_joint_transform PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_joint_transform
    COMMAND test_joint_transform
)

add_executable(test_trajectory
    test_trajectory.c
    ../Config/joint_config.h
    ../Config/joint_config.c
    ../Motion/Inc/trajectory.h
    ../Motion/Src/trajectory.c
)

target_include_directories(test_trajectory PRIVATE
    ../Config
    ../Motion/Inc
    ../Robot/Inc
)

target_compile_options(test_trajectory PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_trajectory
    COMMAND test_trajectory
)

add_executable(test_motion
    test_motion.c
    ../Config/joint_config.h
    ../Config/joint_config.c
    ../Motion/Inc/motion.h
    ../Motion/Inc/joint_transform.h
    ../Motion/Inc/trajectory.h
    ../Motion/Src/motion.c
    ../Motion/Src/joint_transform.c
)

target_include_directories(test_motion PRIVATE
    ../Config
    ../Motion/Inc
    ../Motor/Inc
    ../Robot/Inc
)

target_compile_options(test_motion PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_motion
    COMMAND test_motion
)

add_executable(test_platform_gpio
    test_platform_gpio.c
    ../Config/build_config.h
    ../Config/joint_config.h
    ../Config/joint_config.c
    ../Platform/Inc/platform_gpio.h
    ../Platform/Src/platform_gpio.c
)

target_include_directories(test_platform_gpio PRIVATE
    ../Config
    ../Platform/Inc
)

target_compile_options(test_platform_gpio PRIVATE
    -Wall
    -Wextra
    -Werror
)

target_compile_definitions(test_platform_gpio PRIVATE
    PLATFORM_GPIO_TEST=1
)

add_test(
    NAME test_platform_gpio
    COMMAND test_platform_gpio
)

add_executable(test_homing
    test_homing.c
    ../Config/build_config.h
    ../Config/joint_config.h
    ../Config/joint_config.c
    ../Motion/Inc/homing.h
    ../Motion/Inc/joint_transform.h
    ../Motion/Src/homing.c
    ../Motion/Src/joint_transform.c
)

target_include_directories(test_homing PRIVATE
    Stubs
    ../Config
    ../Motion/Inc
    ../Motor/Inc
    ../Platform/Inc
    ../Robot/Inc
)

target_compile_options(test_homing PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_homing
    COMMAND test_homing
)

add_executable(test_robot_state_full
    test_robot_state_full.c
    Stubs/cmsis_os2.h
    ../Config/build_config.h
    ../Robot/Inc/robot_state.h
    ../Robot/Inc/robot_types.h
    ../Robot/Src/robot_state.c
)

target_include_directories(test_robot_state_full PRIVATE
    Stubs
    ../Config
    ../Robot/Inc
)

target_compile_options(test_robot_state_full PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_robot_state_full
    COMMAND test_robot_state_full
)

add_executable(test_robustness
    test_robustness.c
    ../Config/build_config.h
    ../Config/joint_config.h
    ../Config/joint_config.c
    ../Motion/Inc/joint_transform.h
    ../Motion/Src/joint_transform.c
    ../Motion/Inc/trajectory.h
    ../Motion/Src/trajectory.c
    ../Motion/Inc/motion.h
    ../Motion/Src/motion.c
)

target_include_directories(test_robustness PRIVATE
    ../Config
    ../Motion/Inc
    ../Robot/Inc
    ../Motor/Inc
)

target_compile_options(test_robustness PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_robustness
    COMMAND test_robustness
)

add_executable(test_integration
    test_integration.c
    Stubs/cmsis_os2.h
    ../Config/build_config.h
    ../Protocol/Inc/protocol.h
    ../Protocol/Inc/messages.h
    ../Protocol/Src/protocol.c
    ../Protocol/Src/messages.c
    ../Gripper/Inc/feetech_sts.h
    ../Gripper/Src/feetech_sts.c
)

target_include_directories(test_integration PRIVATE
    Stubs
    ../Config
    ../Gripper/Inc
    ../Motor/Inc
    ../Motion/Inc
    ../Protocol/Inc
    ../Robot/Inc
)

target_compile_options(test_integration PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_integration
    COMMAND test_integration
)

add_executable(test_bug_regression
    test_bug_regression.c
    Stubs/cmsis_os2.h
    ../Config/build_config.h
    ../Config/joint_config.h
    ../Config/joint_config.c
    ../Protocol/Inc/protocol.h
    ../Protocol/Inc/messages.h
    ../Protocol/Src/protocol.c
    ../Protocol/Src/messages.c
    ../Gripper/Inc/feetech_sts.h
    ../Gripper/Src/feetech_sts.c
    ../Robot/Inc/robot_state.h
    ../Robot/Inc/robot_types.h
    ../Robot/Src/robot_state.c
)

target_include_directories(test_bug_regression PRIVATE
    Stubs
    ../Config
    ../Gripper/Inc
    ../Motor/Inc
    ../Motion/Inc
    ../Protocol/Inc
    ../Robot/Inc
)

target_compile_options(test_bug_regression PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_bug_regression
    COMMAND test_bug_regression
)

add_executable(test_protocol_robustness
    test_protocol_robustness.c
    ../Config/build_config.h
    ../Protocol/Inc/protocol.h
    ../Protocol/Src/protocol.c
)

target_include_directories(test_protocol_robustness PRIVATE
    ../Config
    ../Protocol/Inc
)

target_compile_options(test_protocol_robustness PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_protocol_robustness
    COMMAND test_protocol_robustness
)

add_executable(test_feetech_sts
    test_feetech_sts.c
    ../Gripper/Inc/feetech_sts.h
    ../Gripper/Src/feetech_sts.c
)

target_include_directories(test_feetech_sts PRIVATE
    ../Gripper/Inc
)

target_compile_options(test_feetech_sts PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(
    NAME test_feetech_sts
    COMMAND test_feetech_sts
)
