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
set(CMAKE_PROJECT_NAME PumpkinCar)

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

# Produce an Intel HEX image for FlyMcu after each successful link.
get_filename_component(ARM_TOOLCHAIN_BIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
find_program(PUMPKIN_OBJCOPY NAMES ${CMAKE_OBJCOPY} arm-none-eabi-objcopy
    HINTS "${ARM_TOOLCHAIN_BIN_DIR}" REQUIRED)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND "${PUMPKIN_OBJCOPY}" -O ihex
        $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex for FlyMcu"
    VERBATIM
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    HARDWARE/USART/serial_dma.c
    APP/COMM/comm_app.c
    APP/COMM/ros_link.c
    PROTOCOL/wire_codec.c
    HARDWARE/MOTOR/motor_driver.c
    HARDWARE/ENCODER/encoder.c
    HARDWARE/KEY/user_key.c
    APP/MOTOR/motor_app.c
    CONTROL/speed_pi.c
    CONTROL/omni_kinematics.c
    CONTROL/chassis_yaw.c
    HARDWARE/OLED/oled.c
    SYSTEM/cpu_load.c
    APP/DISPLAY/display_app.c
    HARDWARE/IMU/icm20948.c
    APP/IMU/imu_app.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    HARDWARE/USART
    APP/COMM
    HARDWARE/MOTOR
    HARDWARE/ENCODER
    HARDWARE/KEY
    APP/MOTOR
    CONTROL
    HARDWARE/OLED
    SYSTEM
    APP/DISPLAY
    HARDWARE/IMU
    APP/IMU
    PROTOCOL
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
