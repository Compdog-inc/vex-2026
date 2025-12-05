cmake_minimum_required(VERSION 3.20)

set(VEX_SDK_PATH "" CACHE STRING "Path to vex SDK")

# VEX V5 toolchain configuration for CMake (modern style)
# Usage: cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-vexv5.cmake

# Allow overriding via -D on command line
set(PLATFORM "vexv5" CACHE STRING "VEX platform")
set(HEADERS "8.0.0" CACHE STRING "Clang headers version")
# If not provided, default to $ENV{HOME}/sdk similar to original makefile
if(NOT VEX_SDK_PATH)
  if(DEFINED ENV{T})
    set(VEX_SDK_PATH "$ENV{T}")
  else()
    if(DEFINED ENV{VEX_SDK_PATH})
      set(VEX_SDK_PATH "$ENV{VEX_SDK_PATH}")
    else()
      set(VEX_SDK_PATH "$ENV{HOME}/sdk")
    endif()
  endif()
endif()

# Compilers
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Force clang for C and C++
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang)

# Target architecture and common flags (mirror mkenv.mk)
set(CFLAGS_CL "-target thumbv7-none-eabi -fshort-enums -Wno-unknown-attributes -U__INT32_TYPE__ -U__UINT32_TYPE__ -D__INT32_TYPE__=long -D__UINT32_TYPE__='unsigned long'")
set(CFLAGS_V7 "-march=armv7-a -mfpu=neon -mfloat-abi=softfp")

# C flags
set(CMAKE_C_FLAGS_INIT "${CFLAGS_CL} ${CFLAGS_V7} -Os -Wall -Werror=return-type -ansi -std=gnu99 -DVexV5")
# C++ flags
set(CMAKE_CXX_FLAGS_INIT "${CFLAGS_CL} ${CFLAGS_V7} -Os -Wall -Werror=return-type -fno-rtti -fno-threadsafe-statics -fno-exceptions -std=gnu++17 -ffunction-sections -fdata-sections -DVexV5")

# Linker and related tools
find_program(ARM_OBJCOPY arm-none-eabi-objcopy REQUIRED)
find_program(ARM_SIZE arm-none-eabi-size REQUIRED)
# Use GNU ld from arm-none-eabi for linking
find_program(ARM_LD arm-none-eabi-ld REQUIRED)
set(CMAKE_LINKER "${ARM_LD}")

# Ensure CMake uses arm-none-eabi-ld for linking executables instead of clang
# Place pre-linker flags (<FLAGS>) first, then object(s) and output, then the
# grouped libraries that are added as link options (so objects precede --start-group).
#
# Force the linker to execute inside the binary (build) directory, not the source root.
# We keep the custom CMAKE_*_LINK_EXECUTABLE rule but wrap it with `cmake -E chdir`.
set(CMAKE_C_LINK_EXECUTABLE   "${CMAKE_COMMAND} -E chdir \"${CMAKE_BINARY_DIR}\" \"${ARM_LD}\"<LINK_FLAGS> -o <TARGET> <OBJECTS><LINK_LIBRARIES>")
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_COMMAND} -E chdir \"${CMAKE_BINARY_DIR}\" \"${ARM_LD}\"<LINK_FLAGS> -o <TARGET> <OBJECTS><LINK_LIBRARIES>")

# Library search path and include path setup from SDK
set(TOOL_INC 
  "-I${VEX_SDK_PATH}/${PLATFORM}/clang/${HEADERS}/include"
  "-I${VEX_SDK_PATH}/${PLATFORM}/gcc/include/c++/4.9.3"
  "-I${VEX_SDK_PATH}/${PLATFORM}/gcc/include/c++/4.9.3/arm-none-eabi/armv7-ar/thumb"
  "-I${VEX_SDK_PATH}/${PLATFORM}/gcc/include"
)

set(TOOL_LIB "-L${VEX_SDK_PATH}/${PLATFORM}/gcc/libs")

# Linker script and map
set(LINKER_SCRIPT "${VEX_SDK_PATH}/${PLATFORM}/lscript.ld")
set(STDLIB_REF    "${VEX_SDK_PATH}/${PLATFORM}/stdlib_0.lib")

# Linker flags mirroring makefile
set(LINKER_FLAGS
  -nostdlib
  -T "${LINKER_SCRIPT}"
  -R "${STDLIB_REF}"
  --gc-section
  -L "${VEX_SDK_PATH}/${PLATFORM}"
  -L "${VEX_SDK_PATH}/${PLATFORM}/gcc/libs"
)

# Compose a single raw linker flags string to preserve exact token shapes (e.g., -L"/path")
set(VEX_LINKER_FLAGS_STRING "-nostdlib -T \"${LINKER_SCRIPT}\" -R \"${STDLIB_REF}\" --gc-section -L\"${VEX_SDK_PATH}/${PLATFORM}\" -L\"${VEX_SDK_PATH}/${PLATFORM}/gcc/libs\"")

# Export variables so project CMakeLists can consume
set(VEX_TOOL_INC "${TOOL_INC}" CACHE STRING "SDK tool includes")
set(VEX_TOOL_LIB "${TOOL_LIB}" CACHE STRING "SDK tool libs")
set(VEX_LINKER_FLAGS "${LINKER_FLAGS}" CACHE STRING "Linker flags for VEX")
set(VEX_LINKER_FLAGS_STRING "${VEX_LINKER_FLAGS_STRING}" CACHE STRING "Linker flags string for VEX (exact formatting)")
set(VEX_OBJCOPY "${ARM_OBJCOPY}" CACHE FILEPATH "arm-none-eabi-objcopy path")
set(VEX_SIZE "${ARM_SIZE}" CACHE FILEPATH "arm-none-eabi-size path")

# Libraries to link (order preserved)
set(VEX_LIBS "--start-group" "v5rt" "stdc++" "c" "m" "gcc" "--end-group" CACHE STRING "VEX libraries")

# Provide default include dir to consumers
set(VEX_PLATFORM_INCLUDE "${VEX_SDK_PATH}/${PLATFORM}/include" CACHE PATH "Platform include path")