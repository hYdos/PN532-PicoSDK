cmake_minimum_required(VERSION 3.13)

include(pico-sdk/pico_sdk_init.cmake)

project(PN532)

pico_sdk_init()

add_library(PN532 INTERFACE)
target_sources(PN532 INTERFACE
PN532/emulatetag.cpp
PN532/llcp.cpp
PN532/mac_link.cpp
PN532/PN532.cpp
PN532/snep.cpp

PN532_SPI/PN532_SPI.cpp
)

target_include_directories(PN532 INTERFACE
"${CMAKE_SOURCE_DIR}/PN532"
"${CMAKE_SOURCE_DIR}/PN532_SPI"
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(PN532 INTERFACE
        hardware_flash
        pico_stdlib
        hardware_spi
)