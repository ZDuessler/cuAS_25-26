#!/bin/bash
ffmpeg -f flv -listen 1 -i rtmp://0.0.0.0:1935/stream -f sdl -pix_fmt rgb24 "Stream" -c copy -movflags faststart -y output.mp4
