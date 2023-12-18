#!/bin/bash

# setup ros environment
source "/workspaces/dependencies/catkin_ws/devel/setup.bash"
source "/workspaces/autsys-projects-student-airrace/catkin_ws/devel/setup.bash"
exec "$@"
