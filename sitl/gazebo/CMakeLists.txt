cmake_minimum_required(VERSION 3.16)
project(png_betaflight_sil_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(PngBetaflightSilBridge SHARED PngBetaflightSilBridge.cc)
target_link_libraries(
  PngBetaflightSilBridge
  PRIVATE
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    gz-math7::gz-math7
)

target_compile_options(PngBetaflightSilBridge PRIVATE -Wall -Wextra -Wpedantic)
