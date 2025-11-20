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
set(CMAKE_PROJECT_NAME FinalTask)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        # sources
        UserCode/Src/Control/pid.cpp
        UserCode/Src/Device/imu.cpp
        UserCode/Src/HAL/bmi088.cpp
        UserCode/Src/Device/motor.cpp
        UserCode/Src/Device/rc.cpp
        UserCode/Src/Object/Gimbal.cpp
        UserCode/Src/RTOS/rtos.cpp
        UserCode/Src/RTOS/callback.cpp
        UserCode/Src/Control/mahony.cpp

        # headers are optional, but harmless
        UserCode/Inc/Device/motor.h
        UserCode/Inc/Device/imu.h
        UserCode/Inc/Device/rc.h
        UserCode/Inc/Control/pid.h
        UserCode/Inc/HAL/bmi088.h
        UserCode/Inc/RTOS/rtos.h
        UserCode/Inc/Object/Gimbal.h
        UserCode/Inc/Control/mahony.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/UserCode/Inc
        ${CMAKE_SOURCE_DIR}/UserCode/Inc/Device
        ${CMAKE_SOURCE_DIR}/UserCode/Inc/Control
        ${CMAKE_SOURCE_DIR}/UserCode/Inc/HAL
        ${CMAKE_SOURCE_DIR}/UserCode/Inc/RTOS
        ${CMAKE_SOURCE_DIR}/UserCode/Inc/Object
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
