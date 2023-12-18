while getopts d:t:b flag
do
    case "${flag}" in
        d) pDrone=${OPTARG}
        echo "$pDrone ist selected";;
        t) pTrajectory=${OPTARG}
        echo "$pTrajectory ist selected";;
        b) REBUILD="True"
	*) exit;;
    esac
done

xhost +
export DISPLAY=:1
if [ "$REBUILD" = "True" ]; then
    echo "rebuilding docker image locally"
    docker build . -t autsys-student-airrace
fi

docker run --rm\
        --env="ROS_IP=10.0.0.2"\
	--env="ROS_MASTER_URI=http://10.0.0.1:11311" \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --network host \
        --privileged\
        autsys-student-airrace \
        roslaunch /workspaces/autsys-projects-student-airrace/catkin_ws/launch/main.launch sim_type:="hitl" drone:="$pDrone" trajectory:="$pTrajectory"

