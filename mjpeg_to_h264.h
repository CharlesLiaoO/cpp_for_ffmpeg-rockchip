#ifndef MJPEG_TO_H264_H
#define MJPEG_TO_H264_H

#include <vector>
#include <cstdint>

// Forward declarations to avoid including heavy FFmpeg headers in .h
struct AVFrame;
struct AVPacket;
struct AVCodec;
struct AVCodecContext;

// MJPEG Decoder: decode jpeg to yuv
// Decode function name rule:
// decode_2xxx: decode (cpu jpeg) to xxx yuv(frame)
//   xxx: drm    (VPU/GPU memory)
//   xxx: cpu    (CPU memory)
//   xxx: bytes  (CPU memory)
// usually use: 2drm and 2bytes
class MJPEGDecoder {
public:
    MJPEGDecoder(int width, int height);
    ~MJPEGDecoder();

    // decode to drm yuv
    AVFrame* decode_2drm(const uint8_t* jpeg_data, size_t data_size);

    // decode to cpu yuv
    AVFrame* decode_2cpu(const uint8_t* jpeg_data, size_t data_size);

    // decode to bytes yuv
    const std::vector<uint8_t> & decode_2bytes(const uint8_t* jpeg_data, size_t data_size);

private:
    const AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVPacket* packet_ = nullptr;
    AVFrame* frame_drm_ = nullptr;
    AVFrame* frame_cpu_ = nullptr;
    std::vector<uint8_t> frame_bytes_;
};


// H264 Encoder: encode yuv to h264
// Encoder function name rule:
// encode_aaa2bbb: encode aaa yuv(frame) to bbb h264(frame/packet)
//   aaa/bbb: drm    (VPU/GPU memory)
//   aaa/bbb: cpu    (CPU memory)
//   aaa/bbb: bytes  (CPU memory)
// usually use: drm2bytes and bytes2bytes
class H264Encoder {
public:
    H264Encoder(int width, int height, int framerate = 30,
            int bitrate = 4000000, int min_bitrate = 3000000,
            int max_bitrate = 5000000);

    ~H264Encoder() {
        cleanup();
    }

    // Encode drm yuv to bytes h264
    std::vector<uint8_t> encode_drm2bytes(AVFrame* yuv) {
        auto fcpu = encode_drm2cpu(yuv);
        return cpu2bytes(fcpu);
    }

    // Encode bytes yuv to bytes h264
    std::vector<uint8_t> encode_bytes2bytes(const std::vector<uint8_t>& yuv);

    // Encode cpu yuv to bytes h264
    std::vector<uint8_t> encode_cpu2bytes(AVFrame* frame) {
        auto pkt = encode_cpu2cpu(frame);
        return cpu2bytes(pkt);
    }

    // Encode drm yuv to cpu h264
    AVPacket* encode_drm2cpu(AVFrame* frame) {
        return encode_xxx2cpu(frame, true);
    }

    // Encode cpu yuv to cpu h264
    AVPacket* encode_cpu2cpu(AVFrame* frame) {
        return encode_xxx2cpu(frame, false);
    }

private:
    void cleanup();

    // Encode drm/cpu yuv to cpu h264
    AVPacket* encode_xxx2cpu(AVFrame* frame, bool is_drm_frame);

    // transfer cpu frame to bytes
    std::vector<uint8_t> cpu2bytes(AVPacket* packet);

    const AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_drm_ = nullptr;  // encode from drm
    AVCodecContext* codec_ctx_cpu_ = nullptr;  // encode from cpu
    AVPacket* packet_ = nullptr;
    AVFrame* frame_drm_ = nullptr;
    AVFrame* frame_cpu_ = nullptr;
};

#endif // MJPEG_TO_H264_H
