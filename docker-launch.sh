#!/bin/bash
sudo xhost +local:root
sudo docker run -it \
    -p 5000:5000 \
	-e DISPLAY=${DISPLAY} \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v $(pwd)/data:/data \
	--env="QT_X11_NO_MITSHM=1" \
	--device /dev/snd:/dev/snd \
    robo_copilot
sudo xhost -local:root