# run these in bin directory

# 1. prepare the input - record jpegs
ffmpeg -f v4l2 -input_format mjpeg -video_size 1600x600 -framerate 30 -i /dev/video9 -ss 00:00:01 -frames:v 60 -c:v copy jpegs/%02d.jpeg
    # check the jpegs: play jpegs
    ffplay -framerate 30 -i jpegs/%02d.jpeg

# 2. check the output h264
    # cannot play h264 with a correct framerate, need to be muxed to mp4
    # ffplay -f h264 -framerate 30 output.h264
ffmpeg -y -r 30 -i output.h264 -c copy output.mp4
ffplay output.mp4
