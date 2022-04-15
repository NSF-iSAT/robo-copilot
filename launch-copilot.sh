#!/bin/bash
xhost +local:root
docker run --device /dev/video0 \
	-e DISPLAY=${DISPLAY} \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--env="QT_X11_NO_MITSHM=1" \
	-it robo-copilot bash)