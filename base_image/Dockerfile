#FROM ros:noetic-ros-desktop-full-l4t-r35.2.1
FROM osrf/ros:noetic-desktop-full
RUN apt-get update && apt-get install -y \
  python3.catkin-tools\
  git\
  wget\
  libtool\
  && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /
COPY catkin_ws /workspaces/dependencies/catkin_ws
WORKDIR /workspaces/dependencies/catkin_ws

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin build'
RUN echo "source /workspaces/dependencies/catkin_ws/devel/setup.bash" >> ~/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
