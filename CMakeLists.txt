cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME zero_arm_mcu)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/Inc/app.h
    App/Inc/app_tasks.h
    App/Inc/app_rtos.h
    App/Src/app.c
    App/Src/app_internal.h
    App/Src/app_tasks.c
    Config/joint_config.c
    Robot/Src/robot_state.c
    Protocol/Src/protocol.c
    Protocol/Src/messages.c
    Platform/Src/platform_time.c
    Platform/Src/platform_uart.c
    Robot/Src/robot.c
    Motor/Inc/X_V2.h
    Motor/Src/X_V2.c
    Motor/Inc/motor_manager.h
    Motor/Src/motor_manager.c
    Motion/Inc/motion.h
    Motion/Src/motion.c
    Motion/Inc/joint_transform.h
    Motion/Src/joint_transform.c
    Motion/Inc/trajectory.h
    Motion/Src/trajectory.c
    Motion/Inc/homing.h
    Motion/Src/homing.c
    Platform/Inc/platform_fdcan.h
    Platform/Src/platform_fdcan.c
    Platform/Inc/platform_gpio.h
    Platform/Src/platform_gpio.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App/Inc
    Config
    Robot/Inc
    Protocol/Inc
    Platform/Inc
    Motor/Inc
    Motion/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

add_custom_target(test_robot
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_robot
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_robot$
        --output-on-failure
    USES_TERMINAL
)

# Build the platform-independent X_V2 byte-packing test with the host compiler.
add_custom_target(test_X_V2
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_X_V2
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_X_V2$
        --output-on-failure
    USES_TERMINAL
)

add_custom_target(test_platform_fdcan
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_platform_fdcan
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_platform_fdcan$
        --output-on-failure
    USES_TERMINAL
)

add_custom_target(test_motor_manager
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_motor_manager
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_motor_manager$
        --output-on-failure
    USES_TERMINAL
)

add_custom_target(test_messages
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_messages
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_messages$
        --output-on-failure
    USES_TERMINAL
)

add_custom_target(test_joint_transform
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_joint_transform
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_joint_transform$
        --output-on-failure
    USES_TERMINAL
)

add_custom_target(test_trajectory
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_trajectory
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_trajectory$
        --output-on-failure
    USES_TERMINAL
)

add_custom_target(test_motion
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_motion
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_motion$
        --output-on-failure
    USES_TERMINAL
)

add_custom_target(test_platform_gpio
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_platform_gpio
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_platform_gpio$
        --output-on-failure
    USES_TERMINAL
)

add_custom_target(test_homing
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_homing
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_homing$
        --output-on-failure
    USES_TERMINAL
)

add_custom_target(test_robot_state_full
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_robot_state_full
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_robot_state_full$
        --output-on-failure
    USES_TERMINAL
)

add_custom_target(test_robustness
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_robustness
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_robustness$
        --output-on-failure
    USES_TERMINAL
)

add_custom_target(test_integration
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_integration
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_integration$
        --output-on-failure
    USES_TERMINAL
)

add_custom_target(test_bug_regression
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_bug_regression
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_bug_regression$
        --output-on-failure
    USES_TERMINAL
)

add_custom_target(test_protocol_robustness
    COMMAND ${CMAKE_COMMAND}
        -S ${CMAKE_CURRENT_SOURCE_DIR}/Tests
        -B ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -DCMAKE_BUILD_TYPE=Debug
    COMMAND ${CMAKE_COMMAND}
        --build ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        --target test_protocol_robustness
        --parallel
    COMMAND ${CMAKE_CTEST_COMMAND}
        --test-dir ${CMAKE_CURRENT_BINARY_DIR}/host-tests
        -R ^test_protocol_robustness$
        --output-on-failure
    USES_TERMINAL
)
