cmake_minimum_required(VERSION 3.10)
project(RockchipCodec LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(rockchip_codec main.cpp)

target_link_libraries(rockchip_codec
    avdevice        # V4L2
    avfilter
    avformat
    avcodec         # FFmpeg
    swscale         # FFmpeg software scale
    swresample      # FFmpeg audio re-sample
    avutil
    rockchip_mpp
    rga
    drm
    pthread
    lzma            # compression algorithm
    z               # zlib
    m               # math
    dl              # dynamic library
)