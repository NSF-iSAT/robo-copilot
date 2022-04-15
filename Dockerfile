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

RUN apt-get update && apt-get -y install git-core && apt-get -y install python3 \
 && apt-get -y install python3-pip && apt-get -y install python3-dev
RUN apt install ffmpeg libsm6 libxext6  -y
RUN apt install -y python-tk
RUN apt-get install -y ros-noetic-cv-bridge && apt-get install -y ros-noetic-vision-opencv

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
RUN git clone https://github.com/NSF-iSAT/misty_wrapper.git
# copy this package & build
RUN mkdir robo-copilot
COPY . robo-copilot/
RUN source /opt/ros/noetic/setup.bash && cd /home/ros/ros_ws && catkin_make

ENTRYPOINT ["/bin/ros-entrypoint.sh"]

EXPOSE 5000
CMD "roslaunch robo-copilot copilot.launch"
