cmake_minimum_required(VERSION 3.5)

project(dsp-fw VERSION 1.0.0)

set(CMAKE_PROJECT_NAME "adsp-fw")

set(ROOT_PATH "${CMAKE_SOURCE_DIR}/../")
#set(BUILD_SDK_PATH "$ENV{QNX_HOST}/usr/bin")
#set(INSTALL_PATH "${ROOT_PATH}")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang")
# set(CMAKE_AR "gcc-ar")
set(CMAKE_CXX_FLAGS " -O2 -Wall -Wextra")
# -std=gnu++14 -stdlib=libstdc++

include_directories(
    ${ROOT_PATH}/src
    #${CMAKE_CURRENT_SOURCE_DIR}/gen
    #${CMAKE_CURRENT_SOURCE_DIR}/qnx
    #${CMAKE_SOURCE_DIR}/../common
)

link_directories(
    #${ROOT_PATH}/qnx_ap/install/aarch64le/lib/
)

set(SRC_CODE
    ${ROOT_PATH}/src/audio_effect_buffers.c
    ${ROOT_PATH}/src/audio_effect_core.c
    ${ROOT_PATH}/src/audio_basic_effects.c
    ${ROOT_PATH}/src/main.c
)

set(DEPEND_LIBS
    m
# pthread
#    slog2
#    tinyxml2
#    prop_interface
)

add_executable(${CMAKE_PROJECT_NAME} ${SRC_CODE})
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC ${DEPEND_LIBS})

message(STATUS "")
message(STATUS "======================================================================")
#message(STATUS "ROOT_PATH = ${ROOT_PATH}")
#message(STATUS "QNX_HOST = $ENV{QNX_HOST}")
message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
#message(STATUS "INSTALL_PATH = ${INSTALL_PATH}")
#message(STATUS "BUILD_SDK_PATH = ${BUILD_SDK_PATH}")
message(STATUS "======================================================================")
message(STATUS "")

#install(TARGETS ${MODULE_NAME} RUNTIME DESTINATION ${INSTALL_PATH})

