cmake_minimum_required(VERSION 3.13.4)
project(RetroWave)

set(CMAKE_CXX_STANDARD 17)

if(${CMAKE_C_COMPILER} MATCHES "(C|c)lang|Xcode")
    message("-- Omg: you are using clang!")
    add_compile_options(-Wno-unknown-warning-option)
endif()

file(GLOB_RECURSE RETROWAVE_BOARD_SOURCES "RetroWaveLib/Board/*.c")
file(GLOB_RECURSE RETROWAVE_BOARD_HEADERS "RetroWaveLib/Board/*.h")
file(GLOB_RECURSE RETROWAVE_PLATFORM_SOURCES "RetroWaveLib/Platform/*.c")
file(GLOB_RECURSE RETROWAVE_PLATFORM_HEADERS "RetroWaveLib/Platform/*.h")
file(GLOB_RECURSE RETROWAVE_PROTOCOL_SOURCES "RetroWaveLib/Protocol/*.c")
file(GLOB_RECURSE RETROWAVE_PROTOCOL_HEADERS "RetroWaveLib/Protocol/*.h")

add_library(
        RetroWave

        RetroWaveLib/RetroWave.c RetroWaveLib/RetroWave.h

        ${RETROWAVE_BOARD_SOURCES}
        ${RETROWAVE_BOARD_HEADERS}
        ${RETROWAVE_PLATFORM_SOURCES}
        ${RETROWAVE_PLATFORM_HEADERS}
        ${RETROWAVE_PROTOCOL_SOURCES}
        ${RETROWAVE_PROTOCOL_HEADERS}
)

target_include_directories(RetroWave INTERFACE .)

install(FILES RetroWaveLib/RetroWave.h DESTINATION include/RetroWaveLib)
install(FILES ${RETROWAVE_BOARD_HEADERS} DESTINATION include/RetroWaveLib/Board)
install(FILES ${RETROWAVE_PLATFORM_HEADERS} DESTINATION include/RetroWaveLib/Platform)
install(FILES ${RETROWAVE_PROTOCOL_HEADERS} DESTINATION include/RetroWaveLib/Protocol)

install(TARGETS RetroWave DESTINATION lib)
