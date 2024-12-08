cmake_minimum_required(VERSION 3.5)

project(foc-svpwm VERSION 1.0.0)

set(CMAKE_PROJECT_NAME "adsp-fw")

set(ROOT_PATH "${CMAKE_SOURCE_DIR}/../")
#set(BUILD_SDK_PATH "$ENV{QNX_HOST}/usr/bin")
#set(PREBUILD_PATH "${ROOT_PATH}/micar/prebuild/bin")
#set(INSTALL_PATH "${ROOT_PATH}")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang")
# set(CMAKE_AR "gcc-ar")
set(CMAKE_CXX_FLAGS " -O2 -Wall -Wextra")
# -std=gnu++14 -stdlib=libstdc++

#add_definitions("-D__qnx__")

include_directories(
    ${ROOT_PATH}/src
    #${CMAKE_CURRENT_SOURCE_DIR}/gen
    #${CMAKE_CURRENT_SOURCE_DIR}/qnx
    #${CMAKE_SOURCE_DIR}/../common
    #${ROOT_PATH}/qnx_ap/install/usr/include/
    #${ROOT_PATH}/micar/install/include/
    #${ROOT_PATH}/qnx_ap/install/usr/include/amss/multimedia/audio_driver_ar/
)

link_directories(
    #${ROOT_PATH}/qnx_ap/install/aarch64le/lib/
    #${CMAKE_SOURCE_DIR}/../third_lib
    #${ROOT_PATH}/qnx_ap/prebuilt/emmc_bu/svp/lib
)

set(SRC_CODE
    ${ROOT_PATH}/src/audio_effect_buffers.c
    ${ROOT_PATH}/src/audio_effect_core.c
    ${ROOT_PATH}/src/effects.c
    ${ROOT_PATH}/src/main.c
)

set(DEPEND_LIBS
    pthread
    m
#    slog2
#    tinyxml2
#    variant-help
#    prop_interface
#    diagnose
#    ${GRPC_LIBRARIES_DEPRECATED}
#    ${MICAR_MW_V2_LIBRARIES_DEPRECATED}
#    micar_log
#    mipm_ipc
#    pps
#    xbus_ppsS
#    csd2IpcClient
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

