#
# CMakeLists for a test ST0204 program
#
# Copyright (c) 2019 BAE Systems
#
# Author: Roman Kostin <rqk@stlport.com>
#

# 2.8 may work too, never checked
cmake_minimum_required (VERSION 3.0)

# This will detect differences between Windows + VS + VGDB
# and Linux-based systems
set(CMAKE_TOOLCHAIN_FILE toolchain_vgdb.cmake)

# Never build is a source tree
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "DO NOT BUILD in-tree.")
endif()

# This is our program and it's all written is C
project(ST0204 C)

# A dedicated function for ELF to BIN conversion
function(make_binary INPUT)
  add_custom_command(TARGET ${INPUT}
    COMMAND ${TOOLCHAIN}-objcopy -O binary ${INPUT} ${INPUT}.bin
    COMMENT "Converting to binary")
  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${INPUT}.bin)
endfunction(make_binary)

# All HAL and LL calls are in libbsp.a library that's built
# in a separate catalog
add_subdirectory(VGDB_BSP)

# This is where our sources are
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/Src source)

# Includes
file(GLOB headers ${CMAKE_CURRENT_SOURCE_DIR}/Inc/*.h)

# Include directories
set (PLATFORM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/VGDB_BSP/STM32L4xxxx)
include_directories(${PLATFORM_DIR}/STM32L4xx_HAL_Driver/Inc)
include_directories(${PLATFORM_DIR}/STM32L4xx_HAL_Driver/Inc/Legacy)
include_directories(${PLATFORM_DIR}/CMSIS_HAL/Device/ST/STM32L4xx/Include)
include_directories(${PLATFORM_DIR}/CMSIS_HAL/Include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Inc)

# Add linker script
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-T${CMAKE_CURRENT_SOURCE_DIR}/VGDB_BSP/STM32L4xxxx/LinkerScripts/STM32L476RG_flash.lds" CACHE INTERNAL "Linker options")

# Build it using the libbsp.a library
add_executable(ST0204 ${source} ${headers})
target_link_libraries(ST0204 bsp)
SET_TARGET_PROPERTIES(ST0204 PROPERTIES COMPILE_FLAGS "-DSLAVE_BOARD=1")

# Build with low level support
SET_TARGET_PROPERTIES(bsp PROPERTIES COMPILE_FLAGS "-DUSE_FULL_LL_DRIVER=1")

# Convert to BIN for programming into flash
make_binary(ST0204)

