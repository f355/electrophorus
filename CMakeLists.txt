cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

# cysecuretools fails to install on macOS and we don't need it anyway,
# so pretend we already have it
set(HAVE_PYTHON_CYSECURETOOLS TRUE)

# Use debug config for Debug builds, release config for everything else
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(MBED_APP_JSON_PATH mbed_app_debug.json5)
else ()
    set(MBED_APP_JSON_PATH mbed_app_release.json5)
endif ()

include(mbed-os/tools/cmake/mbed_toolchain_setup.cmake)

project(electrophorus
        # VERSION 1.0.0
        LANGUAGES C CXX ASM)

include(mbed_project_setup)

if ("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Linux")
    set(UPLOAD_METHOD OPENOCD)
    set(OPENOCD_CHIP_CONFIG_COMMANDS -f ${CMAKE_CURRENT_LIST_DIR}/openocd.cfg -f ${CMAKE_CURRENT_LIST_DIR}/mbed-os/targets/upload_method_cfg/openocd_cfgs/lpc1768.cfg)
else ()
    set(UPLOAD_METHOD NONE)
endif ()

# Fix implicit function declaration error in mbed-os
add_compile_options(-Wno-error=implicit-function-declaration)

add_subdirectory(mbed-os)

add_subdirectory(lib/MODDMA)

add_executable(${PROJECT_NAME}
        src/main.cpp
        src/irq_ticker.cpp
        src/machine_config.cpp
        src/module.cpp
        src/modules/adc/adc.cpp
        src/modules/e_stop/e_stop.cpp
        src/modules/gpio/digital_ins.cpp
        src/modules/gpio/digital_outs.cpp
        src/modules/gpio/pulse_counter.cpp
        src/modules/pwm/pwm.cpp
        src/modules/stepper/stepper.cpp
        src/pin.cpp
        src/spi_comms.cpp
)

# Add crash handler only for debug builds
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_sources(${PROJECT_NAME} PRIVATE src/crash_handler.cpp)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC . src include)

target_link_libraries(${PROJECT_NAME} mbed-baremetal moddma)

# Add debug flags for better crash reporting
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${PROJECT_NAME} PRIVATE
            -g3                          # Maximum debug info
            -fno-omit-frame-pointer      # Keep frame pointers for better stack traces
            -funwind-tables              # Generate unwind tables
            -fasynchronous-unwind-tables # Better exception handling
    )
endif ()

mbed_set_post_build(${PROJECT_NAME})
