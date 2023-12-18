FROM spokorny/star-simulation:v0.5

COPY entrypoint.sh /
COPY catkin_ws /workspaces/autsys-projects-student-airrace/catkin_ws
WORKDIR /workspaces/autsys-projects-student-airrace/catkin_ws
RUN /bin/bash -c '. /workspaces/dependencies/catkin_ws/devel/setup.bash; catkin build'
RUN echo "source /workspaces/autsys-projects-student-airrace/catkin_ws/devel/setup.bash" >> ~/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
