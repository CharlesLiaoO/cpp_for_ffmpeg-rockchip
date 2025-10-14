#include "mjpeg_to_h264.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <thread>
#include <chrono>

namespace fs = std::filesystem;

std::vector<uint8_t> read_file(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    std::vector<uint8_t> data(fs::file_size(path));
    file.read((char*)data.data(), data.size());
    return data;
}

void write_file(const std::string& path, const std::vector<uint8_t>& data) {
    std::ofstream file(path, std::ios::binary);
    file.write((char*)data.data(), data.size());
}

int main() {
    std::cout<< "---- pwd: "<< fs::current_path() << std::endl;

    std::vector<fs::path> files;
    for (auto& entry : fs::directory_iterator("jpegs")) {
        if (entry.path().extension() == ".jpeg")
            files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());

    MJPEGDecoder decoder(1600, 600);
    H264Encoder encoder(1600, 600, 30, 4000000, 3000000, 5000000);

    fs::create_directories("yuvs");
    std::vector<uint8_t> h264_stream;

    std::cout<< "---- start ----"<< std::endl;
    for (auto& file : files) {
        auto jpeg_data = read_file(file.string());

        // Method 1. copy yuv frame to cpu then encode. [not support encode cpu frame in this branch]
        // auto yuv_data = decoder.decode_2cpu(jpeg_data.data(), jpeg_data.size());
        // auto h264_frame = encoder.encode_cpu2bytes(yuv_data);

        // Method 2. use drm frame to encode; faster.
        auto yuv_data = decoder.decode_2drm(jpeg_data.data(), jpeg_data.size());
        auto h264_frame = encoder.encode_drm2bytes(yuv_data);

        std::cout << "h264_frame.size() "<< h264_frame.size() << std::endl;

        h264_stream.insert(h264_stream.end(), h264_frame.begin(), h264_frame.end());
    }

    write_file("output.h264", h264_stream);
    std::cout<< "---- end ----"<< std::endl;
    return 0;
}