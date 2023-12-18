blue=$(tput setaf 4)
green=$(tput setaf 2)
red=$(tput setaf 1)
normal=$(tput sgr0)


echo "   _____ __            __           __     ___    _      ____                  ";
echo "  / ___// /___  ______/ /__  ____  / /_   /   |  (_)____/ __ \____ _________   ";
echo "  \__ \/ __/ / / / __  / _ \/ __ \/ __/  / /| | / / ___/ /_/ / __ \`/ ___/ _ \ ";
echo " ___/ / /_/ /_/ / /_/ /  __/ / / / /_   / ___ |/ / /  / _, _/ /_/ / /__/  __/  ";
echo "/____/\__/\__,_/\__,_/\___/_/ /_/\__/  /_/  |_/_/_/  /_/ |_|\__,_/\___/\___/   ";
echo "                                                                               ";

################################################################################
# Help                                                                         #
################################################################################
Help()
{
   # Display Help
   echo "This script launches the Student AirRace autonomous flight simulation"
   echo
   echo "Syntax: run.sh -d [StarOne|StarOne8|Default] -s [HITL|SITL] -t [3|4|6]  -r [True|False]"
   echo "options:"
   echo "-d     selects the drone that it used in the simulation. There are two options:"
   echo "             1. StarOne  - The official StudentAirRace prototype, which is a octacopter in a coaxial configuration "
   echo "             2. StarOne8 - The official StudentAirRace prototype, which is a octacopter in a coaxial configuration "   
   echo "             3. Default  - The standard Quadcopter of the Autonomous Systems chair at TUM"
   echo "-s     selects the type of simulation. There are to options:"
   echo "             1. HITL - Hardware in the Loop. This option reqiures the drone to be connected via ethernet."
   echo "                       The Simulations runs on this machine, while all other components will be deployed to the connected drone onboard computer."
   echo "             2. SITL - Software in th Loop. This option does not require any dependencies"
   echo "                       The whole project runs on this machine."
   echo "-t     Selects the degree of freedom of the trajectory. There are three options:"
   echo "             1. 3 - The trajectory is constrained by the position only. The drone is able to optimize the roll and pitch angle, but the yaw angle and the position is fixed at the gates."
   echo "             2. 4 - The trajectory is constrained by the position and the yaw angle. The drone is able to optimize the roll and pitch angle, but the position is fixed at the gates." 
   echo "             3. 6 - The trajectory is constrained by the position, the yaw angle and the roll and pitch angle. The drone is able to optimize the roll and pitch angle, but the position is fixed at the gates."
   echo "-r     decides if the docker image should be rebuild instead of downloaded from dockerhub (This option is only necessary if the project needs to be executed on another arhitecture than x86" 

}


################################################################################"
# Parameter validation                                                         #"
################################################################################"
FAILED=0;
REBUILD="False";
TRAJECTORY="6";
#Get options
while getopts s:d:r:t: flag
do
    case "${flag}" in
        d) pDrone=${OPTARG};;
        s) pSimulationType=${OPTARG};;
		t) pTrajectory=${OPTARG} ;;
		r) REBUILD="True"; 
			echo "rebuilding docker image locally";;
		*) Help
	   	   exit;;
    esac
done
DRONE="Default"
#Validate user input for Drone model
case "${pDrone}" in
	StarOne) DRONE="StarOne"
		printf "\U2705 ${green}StarOne - the official StudentAirrace prototype is used in the simulation ${normal} \n ";;
	StarOne8) DRONE="StarOne8"
		printf "\U2705 ${green}StarOne8 - the official StudentAirrace prototype with coaxial octacopter dynamic is used in the simulation ${normal} \n ";;
	Default) DRONE="Default"
		printf "\U2705 ${green}Default - the default Quadrotor is used in the simulation ${normal} \n ";;
	*)
           printf "\U274C ${red}No valid Drone type given. Please provide one of the following options:  StarOne or Default ${normal}\n"
	   echo "Default Quadrotor will is used in the simulation"
           DRONE="Default" ;;
esac


case "$pTrajectory" in
	3) TRAJECTORY="3"
		printf "\U2705 ${green}3 - The trajectory is constrained by the position only. The drone is able to optimize the roll and pitch angle, but the yaw angle and the position is fixed at the gates. ${normal} \n ";;
	4) TRAJECTORY="4"
		printf "\U2705 ${green}4 - The trajectory is constrained by the position and the yaw angle. The drone is able to optimize the roll and pitch angle, but the position is fixed at the gates. ${normal} \n ";;
	6) TRAJECTORY="6"
		printf "\U2705 ${green}6 - The trajectory is constrained by the position, the yaw angle and the roll and pitch angle. The drone is able to optimize the roll and pitch angle, but the position is fixed at the gates. ${normal} \n ";;
	*)
	   printf "\U274C ${red}No valid trajectory type given. Please provide one of the following options:  3, 4 or 6 ${normal}\n"
	   echo "6 - The trajectory is constrained by the position, the yaw angle and the roll and pitch angle."
	   TRAJECTORY="6" ;;
esac

#Vaidate user input for Simulation type
case "${pSimulationType}" in
	HITL) SIMULATION_TYPE="HITL"
		printf "\U2705 ${green}Hardware in the Loop Simulation is selected ${normal} \n ";;
	SITL) SIMULATION_TYPE="SITL"
	    printf "\U2705 ${green}Software in the Loop Simulation is selected ${normal} \n ";;
	*) printf "\U274C ${red}No valid simulation type given. Please provide one of the following options: HITL or SITL ${normal}\n"
	   echo "Software In the Loop Simulation will be used per default";
	   SIMULATION_TYPE="SITL";;
esac

echo ""
echo ""
echo "################################################################################"
echo "# Dependency validation                                                        #"
echo "################################################################################"

xhost +
if ! command -v docker
then

   printf "\U274C ${red}docker is not installed. Please install docker and retry ${normal}\n"
else
   printf "\U2705 ${green}docker found ${normal} \n "
fi

if ! command -v wget
then
    printf "\U274C ${red}wget is not installed. Please install wget and retry${normal}\n"
else
   printf "\U2705 ${green} wget found${normal}\n"
fi

if ! command -v unzip
then
    printf "\U274C ${red}unzip is not installed. Please install unzip and retry${normal}\n"
else
   printf "\U2705 ${green} unzip found${normal}\n"
fi

echo ""
echo ""
echo "################################################################################"
echo "# Download the Simulation                                                      #"
echo "################################################################################"
SIMULATION_BINARY=./simulation/STAR.x86_64
if test -f "$SIMULATION_BINARY"; then
    printf "\U2705 ${green} $SIMULATION_BINARY exists.${normal}\n"
else
    mkdir simulation	
    wget https://syncandshare.lrz.de/dl/fiVjkC1rpmUwdRYp6Spm46/STAR_Data_8.zip -O STAR_Data.zip
	#wget https://syncandshare.lrz.de/dl/fi2x32Upo8MDe6eJj5fXmg/STAR_Data_Fullscreen_Ultra.zip -O STAR_Data.zip
    unzip STAR_Data.zip -d simulation
    rm STAR_Data.zip
    chmod +x ./simulation/STAR.x86_64
fi

echo ""
echo ""
echo "################################################################################"
echo "# Build Image and run Simulation and Docker container                          #"
echo "################################################################################"


if [ "$DRONE" == "StarOne" ]; then
	echo "Starting simulation StarOne Drone"
	./simulation/STAR.x86_64 --drone starone --motors 4 &
	sleep 5
else
	if [ "$DRONE" == "StarOne8" ]; then
		echo "Starting simulation StarOne Drone"
		./simulation/STAR.x86_64 --drone starone --motors 8 &
		sleep 5
	else
		echo "Starting simulation with Default Drone"
		./simulation/STAR.x86_64 --drone default &
		sleep 5
	fi
fi


if [ "$SIMULATION_TYPE" == "HITL" ] ; then
	echo "run hitl simulation";
	echo "deploy project on Drone";
	scp -r ./* star@10.0.0.2:/home/star/projects/autsys/
	echo "run simulation locally"
	docker run --rm\
		--env="ROS_IP=10.0.0.1"\
                --env="DISPLAY" \
                --env="QT_X11_NO_MITSHM=1" \
                --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
                --env="XAUTHORITY=$XAUTH" \
                --volume="$XAUTH:$XAUTH" \
                --network host \
                --privileged\
                spokorny/student-airrace:v0.9 \
                roslaunch /workspaces/dependencies/catkin_ws/src/simulation/launch/simulation.launch drone:="$DRONE" &

	echo "connect to drone on 10.0.0.2"
	echo "run drone software"
	ssh star@10.0.0.2 DISPLAY=:1 /home/star/projects/autsys/DroneRun.bash -d "$DRONE" -t "$TRAJECTORY" -s "$SIMULATION_TYPE" -r "$REBUILD"


else
	if [ "$REBUILD" == "True" ] ; then
		if docker build . -t spokorny/student-airrace:v0.9; then
			printf "\U2705 ${green} Docker Build finished${normal}"
		else
			exit
		fi
	fi

	echo "run sitl simulation"
	docker run -it --rm\
    	--env="DISPLAY" \
		--env="QT_X11_NO_MITSHM=1" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix" \
        --volume="$(pwd)/base_image/catkin_ws/src/controller8_pkg/config/controller_params.yaml:/workspaces/dependencies/catkin_ws/src/controller8_pkg/config/controller_params.yaml" \
		--env="XAUTHORITY=$XAUTH" \
		--volume="$XAUTH:$XAUTH" \
		--network host \
		--privileged\
		--volume="$(pwd)/catkin_ws/src/mapping/map_generation/track:/workspaces/autsys-projects-student-airrace/catkin_ws/src/mapping/map_generation/track" \
		spokorny/student-airrace:v0.9 \
		roslaunch /workspaces/autsys-projects-student-airrace/catkin_ws/launch/main.launch drone:="$DRONE" trajectory:="$TRAJECTORY"
fi