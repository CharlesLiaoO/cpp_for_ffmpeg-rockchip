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
// Decode function name rule:
// decode_2xxx: decode (cpu jpeg) to xxx yuv(frame)
//   xxx: drm    (VPU/GPU memory)
//   xxx: cpu    (CPU memory)
//   xxx: bytes  (CPU memory)
// usually use: 2drm and 2bytes
class MJPEGDecoder {
public:
    MJPEGDecoder(int width, int height)
    {
        codec_ = avcodec_find_decoder_by_name("mjpeg_rkmpp");
        codec_ctx_ = avcodec_alloc_context3(codec_);
        codec_ctx_->width = width;
        codec_ctx_->height = height;
        codec_ctx_->pix_fmt = AV_PIX_FMT_DRM_PRIME;
        // codec_ctx_->sw_pix_fmt = AV_PIX_FMT_NV12;  // sw_pix_fmt may not be used
        // codec_ctx_->pix_fmt = AV_PIX_FMT_YUVJ422P ;  // ok, match source
        // codec_ctx_->pix_fmt = AV_PIX_FMT_NV16 ;  // runtime error (AV_PIX_FMT_NV12 also): not supported

        if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0)
            throw std::runtime_error("Failed to open decoder");

        packet_ = av_packet_alloc();  // packet is always in cpu memory
        frame_drm_ = av_frame_alloc();

        frame_cpu_ = av_frame_alloc();
        // frame_cpu_->format = AV_PIX_FMT_YUVJ422P;  // ok; not mentioned in doc('ffmpeg -h decoder=mjpeg_rkmpp')?
        frame_cpu_->format = AV_PIX_FMT_NV16;  // nv16 = yuv422sp <--hardware decode-- yuvj422p(source); mentioned in doc
        frame_cpu_->width = width;
        frame_cpu_->height = height;

        int buf_size = av_image_get_buffer_size((AVPixelFormat)frame_cpu_->format, width, height, 1);
        if (buf_size < 0)
            throw std::runtime_error("av_image_get_buffer_size");
        frame_bytes_.resize(buf_size);
    }

    // decode to drm yuv
    AVFrame* decode_2drm(const uint8_t* jpeg_data, size_t data_size) {
        // auto t1 = std::chrono::steady_clock::now();
        packet_->data = const_cast<uint8_t*>(jpeg_data);
        packet_->size = data_size;
        int ret = avcodec_send_packet(codec_ctx_, packet_);
        if (ret != 0)
            throw std::runtime_error("avcodec_send_packet");

        ret = avcodec_receive_frame(codec_ctx_, frame_drm_);
        if (ret != 0)
            throw std::runtime_error("avcodec_receive_frame");

        // auto t2 = std::chrono::steady_clock::now();
        // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;
        return frame_drm_;
    }

    // decode to cpu yuv
    AVFrame* decode_2cpu(const uint8_t* jpeg_data, size_t data_size) {
        AVFrame* fdrm = decode_2drm(jpeg_data, data_size);

        int ret = av_hwframe_transfer_data(frame_cpu_, fdrm, 0);
        if (ret != 0)
            throw std::runtime_error("av_hwframe_transfer_data");

        return frame_cpu_;
    }

    // decode to bytes yuv
    const std::vector<uint8_t> & decode_2bytes(const uint8_t* jpeg_data, size_t data_size) {
        AVFrame* fcpu = decode_2cpu(jpeg_data, data_size);

        int ret = av_image_copy_to_buffer(frame_bytes_.data(), frame_bytes_.size()
            , fcpu->data, fcpu->linesize, (AVPixelFormat)fcpu->format, fcpu->width, fcpu->height, 1);
        if (ret < 0)
            throw std::runtime_error("av_image_copy_to_buffer");

        return frame_bytes_;
    }

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
            int max_bitrate = 5000000)
    {
        codec_ = avcodec_find_encoder_by_name("h264_rkmpp");
        if (!codec_)
            throw std::runtime_error("h264_rkmpp encoder not found");

        codec_ctx_ = avcodec_alloc_context3(codec_);
        codec_ctx_->width = width;
        codec_ctx_->height = height;
        codec_ctx_->time_base = {1, framerate};
        codec_ctx_->framerate = {framerate, 1};

        codec_ctx_->pix_fmt = AV_PIX_FMT_DRM_PRIME;  // need to set with codec_ctx_->sw_pix_fmt
        // , otherwise get runtime error: [h264_rkmpp @ 0x55576db7b0] Unsupported input pixel format '(null)'
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
        // av_dict_set(&opts, "afbc", "rga", 0);  // dont use when dont use rga

        if (avcodec_open2(codec_ctx_, codec_, &opts) < 0)
            throw std::runtime_error("Failed to open h264_rkmpp encoder");

        av_dict_free(&opts);

        packet_ = av_packet_alloc();

        frame_drm_ = av_frame_alloc();
        frame_drm_->format = AV_PIX_FMT_DRM_PRIME;
        frame_drm_->width = width;
        frame_drm_->height = height;
        frame_drm_->pts = 0;

        frame_cpu_ = av_frame_alloc();
        frame_cpu_->format = AV_PIX_FMT_NV16;
        frame_cpu_->width = width;
        frame_cpu_->height = height;
        frame_cpu_->pts = 0;
    }

    ~H264Encoder() {
        av_packet_free(&packet_);
        av_frame_free(&frame_drm_);
        av_frame_free(&frame_cpu_);
    }

    // Encode drm yuv to bytes h264
    std::vector<uint8_t> encode_drm2bytes(AVFrame* yuv) {
        auto fcpu = encode_drm2cpu(yuv);
        return cpu2bytes(fcpu);
    }

    // Encode bytes yuv to bytes h264
    std::vector<uint8_t> encode_bytes2bytes(const std::vector<uint8_t>& yuv) {
        // yuv bytes to cpu [no api to trans bytes to drm directly]
        av_image_fill_arrays(frame_cpu_->data, frame_cpu_->linesize,
                           yuv.data(), (AVPixelFormat)frame_cpu_->format,
                           frame_cpu_->width, frame_cpu_->height, 1);
        return encode_cpu2bytes(frame_cpu_);
    }

    // Encode cpu yuv to bytes h264
    std::vector<uint8_t> encode_cpu2bytes(AVFrame* frame) {
        auto fcpu = encode_cpu2cpu(frame);
        return cpu2bytes(fcpu);
    }

    // Encode drm yuv to cpu h264
    AVPacket* encode_drm2cpu(AVFrame* frame) {
        auto t1 = std::chrono::steady_clock::now();

        if (!sei_data_.empty()) {
            attach_sei_to_frame(frame);
        }

        frame->pts++;
        int ret = avcodec_send_frame(codec_ctx_, frame);
        if (ret < 0)
            throw std::runtime_error("avcodec_send_frame failed");

        // no B frame, no loop to receive packet
        ret = avcodec_receive_packet(codec_ctx_, packet_);
        if (ret < 0)
            throw std::runtime_error("avcodec_receive_packet failed");

        auto t2 = std::chrono::steady_clock::now();
        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;

        return packet_;
    }

    // Encode cpu yuv to cpu h264
    AVPacket* encode_cpu2cpu(AVFrame* frame) {
        // Before invoking encode_drm2cpu(), the codec_ctx_drm_->hw_device_ctx(&hw_frames_ctx) is null, so get err: Invalid argument.
        // Create codec_ctx_drm_->hw_device_ctx(&hw_frames_ctx) in H264Encoder constructor is a bit complicated for this example,
        // so use codec_ctx_cpu_.
        // int ret = av_hwframe_transfer_data(drm_frame_, cpu_frame_, 0);  // err: Invalid argument
        // if (ret < 0) {
        //     char errbuf[AV_ERROR_MAX_STRING_SIZE];
        //     av_strerror(ret, errbuf, sizeof(errbuf));
        //     throw std::runtime_error(errbuf);
        // }
        // return encode_drm2cpu(drm_frame_);

        // use cpu frame directly get err:
        // [h264_rkmpp @ 0x55576cbe00] Only linear and AFBC modifiers are supported
        // [h264_rkmpp @ 0x55576cbe00] Failed to submit frame on input
        // return encode_drm2cpu(frame);

        throw std::runtime_error("not support encode cpu frame in this branch");
    }

    // Set custom SEI data
    void set_sei(const std::vector<char>& data) {
        sei_data_ = data;
    }

private:
    // transfer cpu frame to bytes
    std::vector<uint8_t> cpu2bytes(AVPacket* packet) {
        return std::vector<uint8_t>(packet->data, packet->data + packet->size);
    }

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

    const AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVPacket* packet_ = nullptr;
    AVFrame* frame_drm_ = nullptr;
    AVFrame* frame_cpu_ = nullptr;

    std::vector<char> sei_data_;
};

#endif // MJPEG_TO_H264_H