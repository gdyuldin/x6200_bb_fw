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
set(CMAKE_PROJECT_NAME x6200_bb_fw)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Log level compiled into the firmware (see Platform/bsp_log.h).
# 4 = BSP_LOG_TRACE (Debug), 2 = BSP_LOG_INFO (Release).
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(BSP_LOG_MAX_LEVEL=4)
    add_compile_definitions(BSP_PROF_ENABLE=1)
else()
    add_compile_definitions(BSP_LOG_MAX_LEVEL=2)
    add_compile_definitions(BSP_PROF_ENABLE=0)
endif()

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/ARM_CMSIS/Include
    # ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/ARM_CMSIS/PrivateInclude
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/ARM_CMSIS/dsppp/Include
)

# Add float support for printf
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float")

# Add project layers, bottom-up
add_subdirectory(Platform)
add_subdirectory(Devices)

# Add application (business logic) library
add_subdirectory(App)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
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
    App

    # Add user defined libraries
)


# Convert to bin file -> add conditional check?
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
 COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)
