#!/bin/sh

gst-launch -v v4l2src device="/dev/video0" ! video/x-raw-yuv,format="(fourcc)I420",width=320,height=240,framerate=30/1,bpp=24 ! rtpvrawpay ! udpsink host=127.0.0.1 port=5000 sync=true

