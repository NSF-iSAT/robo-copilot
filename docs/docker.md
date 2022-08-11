# Dockerizing a ROS environment

Docker is a super useful tool if you want your code to be portable to any device without wrestling with dependency issues. By running the code in a prebuilt container, you can effectively guarantee that your code will work as expected anywhere.

To Dockerize your environment, your first step is to create a file named `Dockerfile` in your package. This file contains the instructions Docker will use to build and run the container for this package.

Below is the Dockerfile I've used, with some annotations:

```Dockerfile
# The base container your container will be built on
FROM ros:noetic
# Your shell environment
SHELL ["/bin/bash", "-c"]

# Basic setup
RUN useradd ros
RUN mkdir /home/ros && chown -R ros: /home/ros
ENV HOME "/home/ros"
# This is useful when adding python stuff in user space
ENV PATH="/home/ros/.local/bin:${PATH}"
# This suppresses some python SSL warnings
ENV PYTHONWARNINGS="ignore:a true SSLContext object"
WORKDIR   /home/ros

# Install dependencies
RUN apt-get update && apt-get -y install git-core && apt-get -y install python3 \
 && apt-get -y install python3-pip && apt-get -y install python3-dev
RUN apt-get install ffmpeg libsm6 libxext6  -y
RUN apt-get install -y python3-tk
RUN apt-get install -y ros-noetic-cv-bridge && apt-get install -y ros-noetic-vision-opencv
RUN apt-get install -y libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0

# Locally install GazeTracking python lib (needed for my project) and dependencies
USER ros
RUN mkdir -p ~/ros_ws/src
RUN git clone https://github.com/antoinelame/GazeTracking.git
RUN echo -e "from setuptools import setup\nsetup(name='gaze_tracking', author='Antoine Lame', packages=['gaze_tracking'])" > GazeTracking/setup.py
RUN pip install numpy opencv-python dlib
RUN pip install -e GazeTracking

# Get ready to install ROS packages
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

# Copy this package & build
RUN mkdir robo_copilot
COPY . robo_copilot/
RUN source /opt/ros/noetic/setup.bash && cd /home/ros/ros_ws && catkin_make
RUN cd robo_copilot && pip install -r requirements.txt

# Set everything up for execution
ENTRYPOINT ["/bin/ros-entrypoint.sh"]
EXPOSE 5000
CMD ["roslaunch", "robo_copilot", "copilot_demo.launch", "use_av:=false"]

```