cmake_minimum_required(VERSION 3.25)
# Component/module name (set once to template this project)
set(COMPONENT_NAME "electrophorus")
project(${COMPONENT_NAME} LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find LinuxCNC headers (installed by linuxcnc-uspace-dev)
find_path(LINUXCNC_INCLUDE_DIR hal.h
        PATH_SUFFIXES linuxcnc
        PATHS /usr/include /usr/local/include
)

if (NOT LINUXCNC_INCLUDE_DIR)
    message(FATAL_ERROR "Could not find linuxcnc headers (hal.h). Install linuxcnc-uspace-dev.")
endif ()

# Build an OBJECT library so we can mirror halcompile's version-script flow
add_library(${COMPONENT_NAME}_objs OBJECT
        electrophorus.cc
)

# Align compile flags/defs with halcompile Makefile.modinc (uspace)
target_include_directories(${COMPONENT_NAME}_objs SYSTEM PRIVATE ${LINUXCNC_INCLUDE_DIR})

target_compile_definitions(${COMPONENT_NAME}_objs PRIVATE RTAPI realtime _GNU_SOURCE __MODULE__ SIM)
target_compile_options(${COMPONENT_NAME}_objs PRIVATE
        -fPIC
        -Os
        -g
        -fno-fast-math
        -fno-unsafe-math-optimizations
        -fno-builtin-sin
        -fno-builtin-cos
        -fno-builtin-sincos
        -Wframe-larger-than=2560
)
# Stricter options for the C++ object library
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${COMPONENT_NAME}_objs PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti -Wall -Wextra -Wpedantic>)
endif ()


# Generate a version script from the .rtapi_export section (if tools are available)

find_program(OBJCOPY_EXECUTABLE objcopy)


set(SYM_FILE ${CMAKE_CURRENT_BINARY_DIR}/${COMPONENT_NAME}.sym)
set(VER_FILE ${CMAKE_CURRENT_BINARY_DIR}/${COMPONENT_NAME}.ver)

# Collect object files from the object library into a list (to avoid semicolon issues)
set(OBJ_FILES $<TARGET_OBJECTS:${COMPONENT_NAME}_objs>)

if (OBJCOPY_EXECUTABLE)
    add_custom_command(
            OUTPUT ${VER_FILE}
            COMMAND ${OBJCOPY_EXECUTABLE} -j .rtapi_export -O binary ${OBJ_FILES} ${SYM_FILE} || ${CMAKE_COMMAND} -E touch ${SYM_FILE}
            COMMAND /bin/sh -c "printf '{ global : \\n' > '${VER_FILE}'; if [ -s '${SYM_FILE}' ]; then tr -s '\\0' '\\n' < '${SYM_FILE}' | awk 'NF{ printf \"%s;\\n\", \$0 }' >> '${VER_FILE}'; fi; printf 'local : * ; };\\n' >> '${VER_FILE}'"
            DEPENDS ${COMPONENT_NAME}_objs
            VERBATIM

    )
    add_custom_target(${COMPONENT_NAME}_ver DEPENDS ${VER_FILE})
endif ()

# Realtime module target
add_library(${COMPONENT_NAME} MODULE
        $<TARGET_OBJECTS:${COMPONENT_NAME}_objs>
        stepgen/stepgen.cc
        spi/spi_driver.cc
        spi/rpi4_spi_driver.cc
        spi/rpi5_spi_driver.cc
)


set_target_properties(${COMPONENT_NAME} PROPERTIES
        PREFIX ""
        SUFFIX ".so"
        OUTPUT_NAME "${COMPONENT_NAME}"
)
# Prefer hidden visibility for C++ (exports controlled by version-script)
set_target_properties(${COMPONENT_NAME} PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON)


# Optional but recommended, matches halcompile linking behavior
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_options(${COMPONENT_NAME} PRIVATE -Wl,-Bsymbolic)
endif ()

# Linker visibility control via version-script (if generated)
if (OBJCOPY_EXECUTABLE)
    add_dependencies(${COMPONENT_NAME} ${COMPONENT_NAME}_ver)
    target_link_options(${COMPONENT_NAME} PRIVATE -Wl,--version-script,${VER_FILE})
endif ()
# Align compile flags/defs for MODULE target too (for C sources we added)
target_compile_definitions(${COMPONENT_NAME} PRIVATE RTAPI realtime _GNU_SOURCE __MODULE__ SIM)
target_compile_options(${COMPONENT_NAME} PRIVATE
        -fPIC
        -Os
        -g
        -fno-fast-math
        -fno-unsafe-math-optimizations
        -fno-builtin-sin
        -fno-builtin-cos
        -fno-builtin-sincos
        -Wframe-larger-than=2560
)


# Include dirs for the final target as well (for IDEs)
# Stricter options for C++ bits we add
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${COMPONENT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti -Wall -Wextra -Wpedantic>)
endif ()

target_include_directories(${COMPONENT_NAME} SYSTEM PRIVATE ${LINUXCNC_INCLUDE_DIR})

target_include_directories(${COMPONENT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# halcompile links -lm by default
target_link_libraries(${COMPONENT_NAME} PRIVATE m)

# Install to the standard LinuxCNC rtlib directory
install(TARGETS ${COMPONENT_NAME} LIBRARY DESTINATION lib/linuxcnc/modules)
