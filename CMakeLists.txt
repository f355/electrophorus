cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

# cysecuretools fails to install on macOS and we don't need it anyway,
# so pretend we already have it
set(HAVE_PYTHON_CYSECURETOOLS TRUE)
set(MBED_APP_JSON_PATH mbed_app.json5)

include(mbed-os/tools/cmake/mbed_toolchain_setup.cmake)

project(firmware
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

add_subdirectory(lib/MODDMA)

add_executable(${PROJECT_NAME}
        src/main.cpp
        src/lpc_timer.cpp
        src/machine_config.cpp
        src/module.cpp
        src/modules/adc/adc.cpp
        src/modules/e_stop/e_stop.cpp
        src/modules/gpio/digitalIns.cpp
        src/modules/gpio/digitalOuts.cpp
        src/modules/pulse_counter/pulse_counter.cpp
        src/modules/pwm/pwm.cpp
        src/modules/stepgen/stepgen.cpp
        src/pin.cpp
        src/pru_thread.cpp
        src/spi_comms.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC . src include)

target_link_libraries(${PROJECT_NAME} mbed-baremetal moddma)

mbed_set_post_build(${PROJECT_NAME})
