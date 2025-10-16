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
        // yuv bytes to cpu
        av_image_fill_arrays(frame_cpu_->data, frame_cpu_->linesize,
                           yuv.data(), (AVPixelFormat)frame_cpu_->format,
                           frame_cpu_->width, frame_cpu_->height, 1);

        return cpu2bytes(encode_cpu2cpu(frame_cpu_));
    }

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

        // use cpu frame directly, with codec_ctx_cpu_
        return encode_xxx2cpu(frame, false);
    }

    // Set custom SEI data
    void set_sei(const std::vector<char>& data) {
        sei_data_ = data;
    }

private:
    // Encode drm/cpu yuv to cpu h264
    AVPacket* encode_xxx2cpu(AVFrame* frame, bool is_drm_frame) {
        // auto t1 = std::chrono::steady_clock::now();

        auto codec_ctx = is_drm_frame ? codec_ctx_drm_ : codec_ctx_cpu_;

        // if (!sei_data_.empty()) {
        //     attach_sei_to_frame(frame);
        // }

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

    // transfer cpu frame to bytes
    std::vector<uint8_t> cpu2bytes(AVPacket* packet) {
        if (sei_data_.empty()) {
            return std::vector<uint8_t>(packet->data, packet->data + packet->size);
        }

        // h264_rkmpp doesn't support SEI via AVFrameSideData
        // Manually insert SEI NAL unit into encoded H.264 stream
        return insert_sei_to_packet(packet);
    }

    void attach_sei_to_frame(AVFrame* frame) {
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

    // Manually construct and insert SEI NAL unit
    std::vector<uint8_t> insert_sei_to_packet(AVPacket* packet) {
        // Build SEI payload first (before RBSP encoding)
        std::vector<uint8_t> sei_payload;

        // SEI payload type: 5 (user data unregistered)
        sei_payload.push_back(0x05);

        // SEI payload size
        size_t payload_size = 16 + sei_data_.size();
        while (payload_size >= 255) {
            sei_payload.push_back(0xFF);
            payload_size -= 255;
        }
        sei_payload.push_back(static_cast<uint8_t>(payload_size));

        // UUID for SEI user data unregistered
        static const uint8_t uuid[16] = {
            0x4d, 0x59, 0x55, 0x55, 0x49, 0x44, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
        };
        sei_payload.insert(sei_payload.end(), uuid, uuid + 16);

        // Payload data
        sei_payload.insert(sei_payload.end(), sei_data_.begin(), sei_data_.end());

        // RBSP trailing bits (0x80 for byte alignment)
        sei_payload.push_back(0x80);

        // Apply emulation prevention (escape sequences 0x000000, 0x000001, etc.)
        std::vector<uint8_t> rbsp = apply_emulation_prevention(sei_payload);

        // Now build complete NAL unit
        std::vector<uint8_t> sei_nal;

        // Start code: 0x00 0x00 0x00 0x01
        sei_nal.insert(sei_nal.end(), {0x00, 0x00, 0x00, 0x01});

        // NAL header: type 6 (SEI)
        sei_nal.push_back(0x06);

        // Add RBSP data
        sei_nal.insert(sei_nal.end(), rbsp.begin(), rbsp.end());

        // Find first slice NAL (type 1 or 5) to insert SEI before it
        uint8_t* data = packet->data;
        int size = packet->size;
        int insert_pos = 0;

        for (int i = 0; i < size - 4; i++) {
            if (data[i] == 0 && data[i+1] == 0) {
                int nal_start = -1;
                if (data[i+2] == 1) {
                    nal_start = i + 3;
                } else if (data[i+2] == 0 && data[i+3] == 1) {
                    nal_start = i + 4;
                    i++;
                }

                if (nal_start > 0 && nal_start < size) {
                    uint8_t nal_type = data[nal_start] & 0x1F;
                    // Insert SEI before first slice
                    if (nal_type == 1 || nal_type == 5) {
                        insert_pos = i;
                        break;
                    }
                }
            }
        }

        // Build final packet: [original data before slice] + [SEI] + [slice data]
        std::vector<uint8_t> result;
        result.reserve(insert_pos + sei_nal.size() + (size - insert_pos));
        result.insert(result.end(), data, data + insert_pos);
        result.insert(result.end(), sei_nal.begin(), sei_nal.end());
        result.insert(result.end(), data + insert_pos, data + size);

        return result;
    }

    // Apply emulation prevention to RBSP data
    std::vector<uint8_t> apply_emulation_prevention(const std::vector<uint8_t>& data) {
        std::vector<uint8_t> result;
        result.reserve(data.size() + data.size() / 100); // Reserve extra space

        int zero_count = 0;
        for (size_t i = 0; i < data.size(); i++) {
            if (zero_count == 2 && data[i] <= 0x03) {
                // Insert emulation prevention byte
                result.push_back(0x03);
                zero_count = 0;
            }

            result.push_back(data[i]);

            if (data[i] == 0x00) {
                zero_count++;
            } else {
                zero_count = 0;
            }
        }

        return result;
    }

    const AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_drm_ = nullptr;  // encode from drm
    AVCodecContext* codec_ctx_cpu_ = nullptr;  // encode from cpu
    AVPacket* packet_ = nullptr;
    AVFrame* frame_drm_ = nullptr;
    AVFrame* frame_cpu_ = nullptr;

    std::vector<char> sei_data_;
};

#endif // MJPEG_TO_H264_H