cmake_minimum_required(VERSION 3.20)

get_filename_component(PROJECT_NAME_FROM_DIR ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT_NAME_FROM_DIR} LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(INC_F "include" CACHE PATH "Project header include directory")

# Ensure project include directory is globally prioritized (for FetchContent deps too)
include_directories(BEFORE ${INC_F})

file(GLOB SRC_C CONFIGURE_DEPENDS
  "src/*.cpp" "src/*.c"
  "src/*/*.cpp" "src/*/*.c"
)

if(NOT SRC_C)
  message(FATAL_ERROR "No sources found in src/.")
endif()

add_executable(${PROJECT_NAME} ${SRC_C})

# Match Makefile: produce an .elf filename in the build directory
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${PROJECT_NAME}.elf")

if(DEFINED VEX_PLATFORM_INCLUDE)
  target_include_directories(${PROJECT_NAME} BEFORE PRIVATE ${INC_F} ${VEX_PLATFORM_INCLUDE})
else()
  target_include_directories(${PROJECT_NAME} BEFORE PRIVATE ${INC_F})
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE VEX_ADD_TO_STRING_SHIM=1)

# Ensure SDK headers are on the include path for standard C/C++ headers
target_compile_options(${PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fexceptions>)

target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
  "${VEX_SDK_PATH}/${PLATFORM}/clang/${HEADERS}/include"
  "${VEX_SDK_PATH}/${PLATFORM}/gcc/include"
  "${VEX_SDK_PATH}/${PLATFORM}/gcc/include/c++/4.9.3"
  "${VEX_SDK_PATH}/${PLATFORM}/gcc/include/c++/4.9.3/arm-none-eabi/armv7-ar/thumb"
)

# Apply toolchain-provided include flags
if(DEFINED VEX_TOOL_INC)
  foreach(inc_flag IN LISTS VEX_TOOL_INC)
    target_compile_options(${PROJECT_NAME} PRIVATE ${inc_flag})
  endforeach()
endif()

include(FetchContent)

FetchContent_Declare(
    ruckig
    GIT_REPOSITORY https://github.com/pantor/ruckig.git
    GIT_TAG        v0.15.3
)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(ruckig)
target_compile_definitions(ruckig PRIVATE _USE_MATH_DEFINES)

if(DEFINED VEX_PLATFORM_INCLUDE)
  target_include_directories(ruckig BEFORE PRIVATE ${INC_F} ${VEX_PLATFORM_INCLUDE})
else()
  target_include_directories(ruckig BEFORE PRIVATE ${INC_F})
endif()
target_compile_options(ruckig PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fexceptions>)
target_compile_definitions(ruckig PRIVATE VEX_ADD_TO_STRING_SHIM=1)

# Ensure SDK headers are on the include path for standard C/C++ headers
target_include_directories(ruckig SYSTEM PRIVATE
  "${VEX_SDK_PATH}/${PLATFORM}/clang/${HEADERS}/include"
  "${VEX_SDK_PATH}/${PLATFORM}/gcc/include"
  "${VEX_SDK_PATH}/${PLATFORM}/gcc/include/c++/4.9.3"
  "${VEX_SDK_PATH}/${PLATFORM}/gcc/include/c++/4.9.3/arm-none-eabi/armv7-ar/thumb"
)

# Apply toolchain-provided include flags
if(DEFINED VEX_TOOL_INC)
  foreach(inc_flag IN LISTS VEX_TOOL_INC)
    target_compile_options(ruckig PRIVATE ${inc_flag})
  endforeach()
endif()

# Make ruckig available to the main target (headers + symbols)
target_include_directories(${PROJECT_NAME} PRIVATE ${ruckig_SOURCE_DIR}/include)
# Place ruckig inside a linker group to ensure all needed objects are retained
# Force inclusion of all ruckig objects to avoid GC dropping needed code
target_link_libraries(${PROJECT_NAME} PRIVATE --whole-archive ruckig --no-whole-archive)

# Linker flags: ensure pre-flags (nostdlib, script, -Map, -L, etc.) appear
# BEFORE objects by placing them into CMake's <FLAGS> bucket via
# CMAKE_EXE_LINKER_FLAGS. The grouped libs remain in LINK_FLAGS.
set(LINK_MAP "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map")
if(DEFINED VEX_LINKER_FLAGS_STRING)
  string(REPLACE " --gc-section" " -Map=\"${LINK_MAP}\" --gc-section" VEX_LINKER_FLAGS_WITH_MAP "${VEX_LINKER_FLAGS_STRING}")
  if(VEX_LINKER_FLAGS_STRING STREQUAL VEX_LINKER_FLAGS_WITH_MAP)
    set(VEX_LINKER_FLAGS_WITH_MAP "${VEX_LINKER_FLAGS_STRING} -Map=\"${LINK_MAP}\"")
  endif()
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${VEX_LINKER_FLAGS_WITH_MAP}")
elseif(DEFINED VEX_LINKER_FLAGS)
  set(_new_flags)
  set(_inserted FALSE)
  foreach(link_flag IN LISTS VEX_LINKER_FLAGS)
    if(NOT _inserted AND link_flag STREQUAL "--gc-section")
      list(APPEND _new_flags "-Map=\"${LINK_MAP}\"")
      set(_inserted TRUE)
    endif()
    list(APPEND _new_flags ${link_flag})
  endforeach()
  if(NOT _inserted)
    list(APPEND _new_flags "-Map=\"${LINK_MAP}\"")
  endif()
  # Put pre-flags into CMAKE_EXE_LINKER_FLAGS so they come before <OBJECTS>
  string(JOIN " " _new_flags_str ${_new_flags})
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${_new_flags_str}")
endif()

# Link libraries: ensure grouped libs appear after objects/target
if(DEFINED VEX_LIBS)
  set(_vex_link_items)
  foreach(lib IN LISTS VEX_LIBS)
    if(lib MATCHES "^-")
      # pass grouping flags like --start-group/--end-group as link "items"
      list(APPEND _vex_link_items ${lib})
    else()
      list(APPEND _vex_link_items -l${lib})
    endif()
  endforeach()
  # Put all in <LINK_LIBRARIES> bucket so they come after objects
  target_link_libraries(${PROJECT_NAME} PRIVATE ${_vex_link_items})
endif()

# Size report after link
if(DEFINED VEX_SIZE)
  add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${VEX_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Size of ELF"
  )
endif()

# Produce .bin via objcopy like Makefile
set(BIN_OUTPUT ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E echo "Generating ${PROJECT_NAME}.bin"
)
if(DEFINED VEX_OBJCOPY)
  add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${VEX_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${BIN_OUTPUT}
    BYPRODUCTS ${BIN_OUTPUT}
    COMMENT "Objcopy to binary"
  )
endif()
