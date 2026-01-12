#include "mjpeg_to_h264.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <algorithm>
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

    MJPEGDecoder decoder;
    H264Encoder encoder;

    decoder.setDisableHwCodec();
    encoder.setDisableHwCodec();

    decoder.init(1600, 600);
    encoder.init(1600, 600, 30, 4000000, 3000000, 5000000);

    fs::create_directories("yuvs");
    std::vector<uint8_t> h264_stream;

    std::cout<< "---- start ----"<< std::endl;
    int frame_count = 0;
    double total_time_ms = 0.0;

    for (auto& file : files) {
        frame_count++;
        auto start_time = std::chrono::high_resolution_clock::now();

        auto jpeg_data = read_file(file.string());

        // Method 0. Only cpu: mjpeg --mjpeg--> yuv --libx264--> h264, with setDisableHwCodec()
        // 50ms, 20fps
        auto yuv_data = decoder.decode_2cpu(jpeg_data.data(), jpeg_data.size());
        auto h264_frame = encoder.encode_cpu2bytes(yuv_data);

        // Method 1. copy yuv frame to cpu then encode.
        // 16ms, 64fps
        // auto yuv_data = decoder.decode_2cpu(jpeg_data.data(), jpeg_data.size());
        // auto h264_frame = encoder.encode_cpu2bytes(yuv_data);

        // Method 2. use drm frame to encode; faster.
        // 12ms, 80fps
        // auto yuv_data = decoder.decode_2drm(jpeg_data.data(), jpeg_data.size());
        // auto h264_frame = encoder.encode_drm2bytes(yuv_data);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double time_ms = duration.count() / 1000.0;
        double fps = 1000.0 / time_ms;
        total_time_ms += time_ms;

        std::cout << "Frame " << frame_count << ": "
                  << "size=" << h264_frame.size() << " bytes, "
                  << "time=" << time_ms << " ms, "
                  << "fps=" << fps << std::endl;

        h264_stream.insert(h264_stream.end(), h264_frame.begin(), h264_frame.end());
    }

    write_file("output.h264", h264_stream);

    std::cout<< "---- end ----"<< std::endl;
    std::cout<< "Total frames: " << frame_count << std::endl;
    std::cout<< "Total time: " << total_time_ms << " ms" << std::endl;
    if (frame_count > 0) {
        std::cout<< "Average time per frame: " << (total_time_ms / frame_count) << " ms" << std::endl;
        std::cout<< "Average FPS: " << (1000.0 * frame_count / total_time_ms) << std::endl;
    }
    return 0;
}