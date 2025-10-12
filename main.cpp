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

    std::cout<< "---- start ----";
    for (auto& file : files) {
        auto jpeg_data = read_file(file.string());

        // auto yuv_data = decoder.decode(jpeg_data.data(), jpeg_data.size());  // copy yuv frame to cpu then encode. ok
        auto yuv_data = decoder.decode_drm(jpeg_data.data(), jpeg_data.size());  // use drm frame to encode. Not ok, get green-screen h264

        auto h264_frame = encoder.encode_drm2bytes(yuv_data);
        std::cout << "h264_frame.size() "<< h264_frame.size() << std::endl;  // an other method to check if encoding is ok

        h264_stream.insert(h264_stream.end(), h264_frame.begin(), h264_frame.end());
    }

    write_file("output.h264", h264_stream);
    std::cout<< "---- end ----";
    return 0;
}