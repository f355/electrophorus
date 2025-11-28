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

include(mbed_project_setup)

if ("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Linux")
    set(UPLOAD_METHOD OPENOCD)
    set(OPENOCD_CHIP_CONFIG_COMMANDS -f ${CMAKE_CURRENT_LIST_DIR}/openocd.cfg -f ${CMAKE_CURRENT_LIST_DIR}/mbed-os/targets/upload_method_cfg/openocd_cfgs/lpc1768.cfg)
else ()
    set(UPLOAD_METHOD NONE)
endif ()

add_subdirectory(mbed-os)

add_executable(${PROJECT_NAME}
        src/machine_config.cc
        src/main.cc
        src/modules/adc/adc.cc
        src/modules/e_stop/e_stop.cc
        src/modules/gpio/digital_ins.cc
        src/modules/gpio/digital_outs.cc
        src/modules/gpio/pulse_counter.cc
        src/modules/pwm/pwm.cc
        src/modules/stepgen/stepgen.cc
        src/pin.cc
        src/rx_listener.cc
        src/spi_comms.cc
        src/stepgen_ticker.cc
)

target_include_directories(${PROJECT_NAME} PUBLIC . src linuxcnc/hal_comps)

target_link_libraries(${PROJECT_NAME} mbed-baremetal)

mbed_set_post_build(${PROJECT_NAME})
