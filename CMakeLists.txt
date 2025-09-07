cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

# cysecuretools fails to install on macOS and we don't need it anyway,
# so pretend we already have it
set(HAVE_PYTHON_CYSECURETOOLS TRUE)

set(MBED_APP_JSON_PATH mbed_app.json5)

include(mbed-os/tools/cmake/mbed_toolchain_setup.cmake)

project(electrophorus
        # VERSION 1.0.0
        LANGUAGES C CXX ASM)

# Debug option: disable hardware output manipulation in firmware builds
option(EPHO_NO_HW_IO "Disable hardware output manipulation (DigitalOuts, PWM, pin writes)" OFF)

# Select which LPC1768 UART the firmware uses for comms with LinuxCNC: 0 (/dev/ttyAMA0) or 2 (/dev/ttyUSB0)
set(EPHO_COMMS_UART 0 CACHE STRING "LPC1768 UART to use for comms: 0 (PL011 on Pi) or 2 (FT232R)")
set_property(CACHE EPHO_COMMS_UART PROPERTY STRINGS 0 2)

include(mbed_project_setup)

if ("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Linux")
    set(UPLOAD_METHOD OPENOCD)
    set(OPENOCD_CHIP_CONFIG_COMMANDS -f ${CMAKE_CURRENT_LIST_DIR}/openocd.cfg -f ${CMAKE_CURRENT_LIST_DIR}/mbed-os/targets/upload_method_cfg/openocd_cfgs/lpc1768.cfg)
else ()
    set(UPLOAD_METHOD NONE)
endif ()

add_subdirectory(mbed-os)

add_subdirectory(lib/MODDMA)

add_executable(${PROJECT_NAME}
        src/machine_config.cpp
        src/main.cpp
        src/module.cpp
        src/modules/adc/adc.cpp
        src/modules/e_stop/e_stop.cpp
        src/modules/gpio/digital_ins.cpp
        src/modules/gpio/digital_outs.cpp
        src/modules/gpio/pulse_counter.cpp
        src/modules/pwm/pwm.cpp
        src/modules/stepgen/stepgen.cpp
        src/pin.cpp
        src/rx_listener.cpp
        src/serial_comms.cpp
        src/console_override.cpp
        src/stepgen_ticker.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC . src include)
if (EPHO_NO_HW_IO)
    target_compile_definitions(${PROJECT_NAME} PRIVATE EPHO_NO_HW_IO)
endif ()

# Propagate UART selection to firmware
if (NOT EPHO_COMMS_UART MATCHES "^(0|2)$")
    message(FATAL_ERROR "EPHO_COMMS_UART must be 0 or 2, got '${EPHO_COMMS_UART}'")
endif ()

target_compile_definitions(${PROJECT_NAME} PRIVATE EPHO_COMMS_UART=${EPHO_COMMS_UART})


target_link_libraries(${PROJECT_NAME} mbed-baremetal moddma)

mbed_set_post_build(${PROJECT_NAME})
