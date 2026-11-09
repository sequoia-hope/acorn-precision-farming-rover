#!/bin/bash
# Usage: broadcast.sh <video device number>
# Streams to ACORN_STREAM_DEST (host:port of the receiver). Required.
: "${ACORN_STREAM_DEST:?set ACORN_STREAM_DEST to the receiver host:port}"
gst-launch-1.0 v4l2src device=/dev/video$1 ! "video/x-raw, format=(string)UYVY, width=(int)4096, height=(int)2160" ! nvvidconv ! "video/x-raw(memory:NVMM), format=(string)I420, width=(int)1920, height=(int)1080" ! omxh264enc ! rtph264pay mtu=60000 ! udpsink clients="$ACORN_STREAM_DEST" sync=false
