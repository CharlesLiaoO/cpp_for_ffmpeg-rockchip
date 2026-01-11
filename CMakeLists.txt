cmake_minimum_required(VERSION 3.10)
project(rockchip_codec LANGUAGES CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(rockchip_codec main.cpp mjpeg_to_h264.cpp)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
  libavcodec libavformat libavutil libswscale
)

target_link_libraries(rockchip_codec
  PkgConfig::LIBAV
)
