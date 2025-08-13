add_library(moddma STATIC DATALUTS.cpp INIT.cpp MODDMA.cpp SETUP.cpp)

target_include_directories(moddma PUBLIC .)

target_link_libraries(moddma PUBLIC mbed-baremetal)