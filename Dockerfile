FROM ros:noetic

SHELL ["/bin/bash", "-c"]

RUN useradd ros
RUN mkdir /home/ros && chown -R ros: /home/ros
ENV HOME "/home/ros"
# This is useful when adding python stuff in user space
ENV PATH="/home/ros/.local/bin:${PATH}"
# This suppresses some python SSL warnings
ENV PYTHONWARNINGS="ignore:a true SSLContext object"
WORKDIR   /home/ros

# Dependencies
RUN apt-get update && apt-get -y install git-core && apt-get -y install python3 \
 && apt-get -y install python3-pip && apt-get -y install python3-dev
RUN apt-get install ffmpeg libsm6 libxext6  -y
RUN apt-get install -y python3-tk
RUN apt-get install -y ros-noetic-cv-bridge && apt-get install -y ros-noetic-vision-opencv
RUN apt-get install -y libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0
RUN apt-get update && apt-get install -y featherpad

USER ros
RUN mkdir -p ~/ros_ws/src

# locally install GazeTracking python lib and dependencies
RUN git clone https://github.com/antoinelame/GazeTracking.git
RUN echo -e "from setuptools import setup\nsetup(name='gaze_tracking', author='Antoine Lame', packages=['gaze_tracking'])" > GazeTracking/setup.py
# RUN pip install -r GazeTracking/requirements.txt
RUN pip install numpy opencv-python dlib
RUN pip install -e GazeTracking

USER root
COPY ./ros-entrypoint.sh /bin/
RUN ["chmod", "+x", "/bin/ros-entrypoint.sh"]

WORKDIR /home/ros/ros_ws/src

# Add cloning of any ROS packages
RUN git clone https://github.com/tim-fan/gaze_tracking_ros.git
RUN git clone https://github.com/NSF-iSAT/misty_wrapper.git \
    && cd misty_wrapper && pip install -r requirements.txt
RUN git clone https://github.com/HIRO-group/ros_speech2text.git \
    && cd ros_speech2text && git checkout ros-noetic && pip install -r requirements.txt
RUN git clone https://github.com/kalebishop/google_tts.git \
    && cd google_tts && pip install -r requirements.txt

# copy this package & build
RUN mkdir robo_copilot
COPY . robo_copilot/
RUN cd robo_copilot && pip install -r requirements.txt
RUN source /opt/ros/noetic/setup.bash && cd /home/ros/ros_ws && catkin_make
RUN cp robo_copilot/ros-speech2text-google-stt-cred.json ros_speech2text/
RUN cd robo_copilot && chmod +x run-task.sh run-misty-copilot.sh record-data.sh

WORKDIR /home/ros/ros_ws/src/robo_copilot
ENTRYPOINT ["/bin/ros-entrypoint.sh"]

EXPOSE 5000
# CMD ["roslaunch", "robo_copilot", "copilot_demo.launch"]
CMD ["bash"]