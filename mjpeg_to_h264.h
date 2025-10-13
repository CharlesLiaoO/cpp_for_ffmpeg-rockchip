#ifndef MJPEG_TO_H264_H
#define MJPEG_TO_H264_H

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_drm.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <memory>
#include <vector>
#include <iostream>
#include <chrono>
#include <cstring>

// MJPEG Decoder: decode jpeg to yuv
class MJPEGDecoder {
public:
    MJPEGDecoder(int width, int height)
        : width_(width), height_(height)
    {
        codec_ = avcodec_find_decoder_by_name("mjpeg_rkmpp");
        codec_ctx_ = avcodec_alloc_context3(codec_);
        codec_ctx_->width = width_;
        codec_ctx_->height = height_;
        codec_ctx_->pix_fmt = AV_PIX_FMT_DRM_PRIME;
        // codec_ctx_->pix_fmt = AV_PIX_FMT_YUVJ422P ;  // ok
        // codec_ctx_->pix_fmt = AV_PIX_FMT_NV16 ;  // runtime error (AV_PIX_FMT_NV12 also): not supported

        if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0)
            throw std::runtime_error("Failed to open decoder");

        packet = av_packet_alloc();
        frame_drm = av_frame_alloc();

        frame = av_frame_alloc();
        frame->format = AV_PIX_FMT_NV16;  // source is yuvj422p: nv16 = yuv422sp ≈ yuvj422p
        frame->width = width;
        frame->height = height;

        int buf_size = av_image_get_buffer_size((AVPixelFormat)frame->format, width, height, 1);
        if (buf_size < 0)
            throw std::runtime_error("av_image_get_buffer_size");
        frame_bytes.resize(buf_size);
    }

    // decode to drm frame (VPU/GPU memory)
    AVFrame* decode_drm(const uint8_t* jpeg_data, size_t data_size) {
        packet->data = const_cast<uint8_t*>(jpeg_data);
        packet->size = data_size;
        int ret = avcodec_send_packet(codec_ctx_, packet);
        if (ret != 0)
            throw std::runtime_error("avcodec_send_packet");

        ret = avcodec_receive_frame(codec_ctx_, frame_drm);
        if (ret != 0)
            throw std::runtime_error("avcodec_receive_frame");

        return frame_drm;
    }

    // decode (CPU memory)
    AVFrame* decode(const uint8_t* jpeg_data, size_t data_size) {
        // auto t1 = std::chrono::steady_clock::now();
        AVFrame* drm_frame = decode_drm(jpeg_data, data_size);  // average < 5ms
        // auto t2 = std::chrono::steady_clock::now();
        // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;

        int ret = av_hwframe_transfer_data(frame, drm_frame, 0);
        if (ret != 0)
            throw std::runtime_error("av_hwframe_transfer_data");

        return frame;
    }

    // decode to bytes (CPU memory)
    const std::vector<uint8_t> & decode_bytes(const uint8_t* jpeg_data, size_t data_size) {
        AVFrame* frame = decode(jpeg_data, data_size);

        int ret = av_image_copy_to_buffer(frame_bytes.data(), frame_bytes.size(), frame->data, frame->linesize, AV_PIX_FMT_NV16, frame->width, frame->height, 1);
        if (ret < 0)
            throw std::runtime_error("av_image_copy_to_buffer");

        return frame_bytes;
    }

private:
    int width_;
    int height_;
    const AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVPacket* packet = nullptr;
    AVFrame* frame_drm = nullptr;
    AVFrame* frame = nullptr;
    std::vector<uint8_t> frame_bytes;
};


// H264 Encoder: encode yuv to h264
class H264Encoder {
public:
    H264Encoder(int width, int height, int framerate = 30,
            int bitrate = 4000000, int min_bitrate = 3000000,
            int max_bitrate = 5000000)
        : width_(width), height_(height)
    {
        codec_ = avcodec_find_encoder_by_name("h264_rkmpp");
        if (!codec_)
            throw std::runtime_error("h264_rkmpp encoder not found");

        codec_ctx_ = avcodec_alloc_context3(codec_);
        codec_ctx_->width = width_;
        codec_ctx_->height = height_;
        codec_ctx_->time_base = {1, framerate};
        codec_ctx_->framerate = {framerate, 1};

        codec_ctx_->pix_fmt = AV_PIX_FMT_DRM_PRIME;  // need to set codec_ctx_->sw_pix_fmt, otherwise get runtime error: [h264_rkmpp @ 0x55576db7b0] Unsupported input pixel format '(null)'
        codec_ctx_->sw_pix_fmt = AV_PIX_FMT_NV16;

        // bit_rate
        codec_ctx_->bit_rate = bitrate;
        codec_ctx_->rc_min_rate = min_bitrate;
        codec_ctx_->rc_max_rate = max_bitrate;
        codec_ctx_->rc_buffer_size = bitrate;

        // GOP
        codec_ctx_->gop_size = framerate;
        codec_ctx_->max_b_frames = 0;

        // Profile and Level
        codec_ctx_->profile = FF_PROFILE_H264_HIGH;
        codec_ctx_->level = 41;

        // flags
        codec_ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;

        // encoder extra options
        AVDictionary* opts = nullptr;
        av_dict_set(&opts, "rc_mode", "0", 0);  // VBR
        // av_dict_set(&opts, "afbc", "rga", 0);

        if (avcodec_open2(codec_ctx_, codec_, &opts) < 0)
            throw std::runtime_error("Failed to open h264_rkmpp encoder");

        av_dict_free(&opts);

        packet_ = av_packet_alloc();

        drm_frame_ = av_frame_alloc();
        drm_frame_->format = AV_PIX_FMT_DRM_PRIME;
        drm_frame_->width = width_;
        drm_frame_->height = height_;

        cpu_frame = av_frame_alloc();
        cpu_frame->format = AV_PIX_FMT_NV16;
        cpu_frame->width = width_;
        cpu_frame->height = height_;
    }

    ~H264Encoder() {
        av_packet_free(&packet_);
        av_frame_free(&drm_frame_);
        av_frame_free(&cpu_frame);
        avcodec_free_context(&codec_ctx_);
    }

    // Encode drm frame to cpu packet(frame)
    AVPacket* encode_drm2packet(AVFrame* frame) {
        if (!sei_data_.empty()) {
            attach_sei_to_frame(frame);
        }

        frame->pts = frame_count_++;
        int ret = avcodec_send_frame(codec_ctx_, frame);
        if (ret < 0)
            throw std::runtime_error("avcodec_send_frame failed");

        // no B frame, no loop to receive packet
        ret = avcodec_receive_packet(codec_ctx_, packet_);
        if (ret < 0)
            throw std::runtime_error("avcodec_receive_packet failed");

        return packet_;
    }

    // transfer (CPU)packet to bytes
    std::vector<uint8_t> packet2bytes(AVPacket* packet) {
        return std::vector<uint8_t>(packet->data, packet->data + packet->size);
    }

    // Encode drm frame to bytes
    std::vector<uint8_t> encode_drm2bytes(AVFrame* frame) {
        auto pkt = encode_drm2packet(frame);
        return packet2bytes(pkt);
    }

    // Encode bytes to packet (CPU)
    AVPacket* encode_bytes2packet(const std::vector<uint8_t>& frame_bytes) {
        av_image_fill_arrays(cpu_frame->data, cpu_frame->linesize,
                           frame_bytes.data(), AV_PIX_FMT_NV16,
                           width_, height_, 1);


        // int ret = av_hwframe_transfer_data(drm_frame_, cpu_frame, 0);  // trans to DRM PRIME, no need
        // if (ret < 0)
        //     throw std::runtime_error("av_hwframe_transfer_data failed");
        drm_frame_ = cpu_frame;  // use CPU frame directly

        return encode_drm2packet(drm_frame_);
    }

    // Encode bytes frame to bytes frame
    std::vector<uint8_t> encode_bytes2bytes(const std::vector<uint8_t>& frame_bytes) {
        auto pkt = encode_bytes2packet(frame_bytes);
        return packet2bytes(pkt);
    }

    // custom SEI data
    void set_sei(const std::vector<char>& data) {
        sei_data_ = data;
    }

private:
    void attach_sei_to_frame(AVFrame* frame) {
        // SEI type 5 (User Data Unregistered)
        const int SEI_TYPE_USER_DATA_UNREGISTERED = 5;

        // UUID for identifying the SEI message
        const uint8_t uuid[16] = {
            0x4d, 0x59, 0x55, 0x55, 0x49, 0x44, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
        };

        size_t total_size = 16 + sei_data_.size();

        AVFrameSideData* side_data = av_frame_new_side_data(frame, AV_FRAME_DATA_SEI_UNREGISTERED, total_size);

        if (side_data) {
            std::memcpy(side_data->data, uuid, 16);
            std::memcpy(side_data->data + 16, sei_data_.data(), sei_data_.size());
        }
    }

    int width_;
    int height_;
    int64_t frame_count_ = 0;  // for pts
    const AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVPacket* packet_ = nullptr;
    AVFrame* drm_frame_ = nullptr;
    AVFrame* cpu_frame = nullptr;

    std::vector<char> sei_data_;
};

#endif // MJPEG_TO_H264_H