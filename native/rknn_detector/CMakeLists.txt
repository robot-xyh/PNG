cmake_minimum_required(VERSION 3.16)
project(circle_rknn_detector_bridge VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_path(RKNN_INCLUDE_DIR rknn_api.h
  PATHS /usr/local/include /usr/include REQUIRED)
find_library(RKNN_RUNTIME_LIBRARY rknnrt
  PATHS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu REQUIRED)

set(VENDOR_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/vendor/circle_pilot)

add_library(rknn_detector_bridge SHARED
  src/rknn_detector_bridge.cpp
  ${VENDOR_ROOT}/adapters/common/perception/src/rknn_engine.cpp
  ${VENDOR_ROOT}/core/src/vision/yolo_postprocess.cpp)

target_include_directories(rknn_detector_bridge
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
    ${RKNN_INCLUDE_DIR}
    ${VENDOR_ROOT}/adapters/common/perception/include
    ${VENDOR_ROOT}/core/include)

target_compile_definitions(rknn_detector_bridge PRIVATE USE_RKNN=1)
target_compile_options(rknn_detector_bridge PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(rknn_detector_bridge PRIVATE ${RKNN_RUNTIME_LIBRARY} dl)

install(TARGETS rknn_detector_bridge LIBRARY DESTINATION lib)
