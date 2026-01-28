#!/usr/bin/env bash
set -e

modprobe v4l2loopback video_nr=2 card_label=robocat_cam exclusive_caps=1 max_buffers=2

rpicam-vid -t 0 --codec mjpeg --width 1640 --height 1232 --framerate 15 -o - \
| ffmpeg -loglevel error -re -i - \
  -f v4l2 -pix_fmt yuyv422 -s 1280x720 -r 15 \
  -vcodec rawvideo /dev/video2
