// Put FFmpeg headers in .cpp to avoid recompiling them every time
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_drm.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <iostream>
#include <chrono>
#include <cstring>
#include <stdexcept>

#include "mjpeg_to_h264.h"

MJPEGDecoder::MJPEGDecoder(int width, int height)
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

MJPEGDecoder::~MJPEGDecoder()
{
    av_packet_free(&packet_);
    av_frame_free(&frame_drm_);
    av_frame_free(&frame_cpu_);
    if (codec_ctx_) {
        avcodec_free_context(&codec_ctx_);
    }
}

AVFrame *MJPEGDecoder::decode_2drm(const uint8_t *jpeg_data, size_t data_size)
{
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

AVFrame *MJPEGDecoder::decode_2cpu(const uint8_t *jpeg_data, size_t data_size)
{
    AVFrame* fdrm = decode_2drm(jpeg_data, data_size);

    int ret = av_hwframe_transfer_data(frame_cpu_, fdrm, 0);
    if (ret != 0)
        throw std::runtime_error("av_hwframe_transfer_data");

    return frame_cpu_;
}

const std::vector<uint8_t> &MJPEGDecoder::decode_2bytes(const uint8_t *jpeg_data, size_t data_size)
{
    AVFrame* fcpu = decode_2cpu(jpeg_data, data_size);

    int ret = av_image_copy_to_buffer(frame_bytes_.data(), frame_bytes_.size()
        , fcpu->data, fcpu->linesize, (AVPixelFormat)fcpu->format, fcpu->width, fcpu->height, 1);
    if (ret < 0)
        throw std::runtime_error("av_image_copy_to_buffer");

    return frame_bytes_;
}

H264Encoder::H264Encoder(int width, int height, int framerate, int bitrate, int min_bitrate, int max_bitrate)
{
    codec_ = avcodec_find_encoder_by_name("h264_rkmpp");
    if (!codec_)
        throw std::runtime_error("h264_rkmpp encoder not found");

    auto open_context = [&](AVCodecContext* &codec_ctx_xxx, bool is_drm) {
        codec_ctx_xxx = avcodec_alloc_context3(codec_);
        codec_ctx_xxx->width = width;
        codec_ctx_xxx->height = height;
        codec_ctx_xxx->time_base = {1, framerate};
        codec_ctx_xxx->framerate = {framerate, 1};

        if (is_drm) {
            codec_ctx_xxx->pix_fmt = AV_PIX_FMT_DRM_PRIME;  // need to set with codec_ctx_xxx->sw_pix_fmt
            // , otherwise get runtime error: [h264_rkmpp @ 0x55576db7b0] Unsupported input pixel format '(null)'
            codec_ctx_xxx->sw_pix_fmt = AV_PIX_FMT_NV16;
        } else
            codec_ctx_xxx->pix_fmt = AV_PIX_FMT_NV16;

        // bit_rate
        codec_ctx_xxx->bit_rate = bitrate;
        codec_ctx_xxx->rc_min_rate = min_bitrate;
        codec_ctx_xxx->rc_max_rate = max_bitrate;
        codec_ctx_xxx->rc_buffer_size = bitrate;

        // GOP
        codec_ctx_xxx->gop_size = framerate;
        codec_ctx_xxx->max_b_frames = 0;

        // Profile and Level
        codec_ctx_xxx->profile = FF_PROFILE_H264_HIGH;
        codec_ctx_xxx->level = 41;

        // flags
        codec_ctx_xxx->flags |= AV_CODEC_FLAG_LOW_DELAY;

        // encoder extra options
        AVDictionary* opts = nullptr;
        av_dict_set(&opts, "rc_mode", "0", 0);  // VBR
        // av_dict_set(&opts, "afbc", "rga", 0);  // dont use when dont use rga

        if (avcodec_open2(codec_ctx_xxx, codec_, &opts) < 0)
            throw std::runtime_error("Failed to open h264_rkmpp encoder");

        av_dict_free(&opts);
    };

    open_context(codec_ctx_drm_, true);  // can open two contexts with same encoder
    open_context(codec_ctx_cpu_, false);

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

void H264Encoder::cleanup()
{
    av_packet_free(&packet_);
    av_frame_free(&frame_drm_);
    av_frame_free(&frame_cpu_);
    if (codec_ctx_drm_) {
        avcodec_free_context(&codec_ctx_drm_);
    }
    if (codec_ctx_cpu_) {
        avcodec_free_context(&codec_ctx_cpu_);
    }
}

std::vector<uint8_t> H264Encoder::encode_bytes2bytes(const std::vector<uint8_t>& yuv)
{
    // yuv bytes to cpu
    av_image_fill_arrays(frame_cpu_->data, frame_cpu_->linesize,
                       yuv.data(), (AVPixelFormat)frame_cpu_->format,
                       frame_cpu_->width, frame_cpu_->height, 1);

    return cpu2bytes(encode_cpu2cpu(frame_cpu_));
}

AVPacket * H264Encoder::encode_xxx2cpu(AVFrame * frame, bool is_drm_frame)
{
    // auto t1 = std::chrono::steady_clock::now();

    auto codec_ctx = is_drm_frame ? codec_ctx_drm_ : codec_ctx_cpu_;

    frame->pts++;
    int ret = avcodec_send_frame(codec_ctx, frame);
    if (ret < 0)
        throw std::runtime_error("avcodec_send_frame failed");

    // no B frame, no loop to receive packet
    ret = avcodec_receive_packet(codec_ctx, packet_);
    if (ret < 0)
        throw std::runtime_error("avcodec_receive_packet failed");

    // auto t2 = std::chrono::steady_clock::now();
    // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;

    return packet_;
}

std::vector<uint8_t> H264Encoder::cpu2bytes(AVPacket *packet)  // write here for cpp includes (no in h file)
{
    return std::vector<uint8_t>(packet->data, packet->data + packet->size);
}
