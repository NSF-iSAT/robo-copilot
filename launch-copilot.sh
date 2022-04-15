#!/bin/bash
xhost +local:root
docker run \
    -p 5000:5000 \
	-e DISPLAY=${DISPLAY} \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--env="QT_X11_NO_MITSHM=1" \
    robo-copilot
xhost -local:root