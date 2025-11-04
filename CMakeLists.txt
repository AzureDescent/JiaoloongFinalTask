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
        UserCode/Inc/Device/motor.h
        UserCode/Inc/Device/imu.h
        UserCode/Inc/Device/rc.h
        UserCode/Inc/Control/pid.h
        UserCode/Inc/HAL/bmi088.h
        UserCode/Inc/RTOS/rtos.h
        UserCode/Inc/Object/Chassis.h
        UserCode/Inc/Object/Gimbal.h
        UserCode/Src/Control/pid.cpp
        UserCode/Src/Device/imu.cpp
        UserCode/Src/HAL/bmi088.cpp
        UserCode/Src/Device/motor.cpp
        UserCode/Src/Device/rc.cpp
        UserCode/Src/Object/Chassis.cpp
        UserCode/Src/Object/Gimbal.cpp
        UserCode/Src/RTOS/rtos.cpp
        UserCode/Src/RTOS/callback.cpp
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        UserCode/Src
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        UserCode/Inc/RTOS
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
