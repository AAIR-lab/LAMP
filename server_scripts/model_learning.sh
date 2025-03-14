#!/usr/bin/env bash
while getopts d:s:p: flag
do
    case "${flag}" in
        d) domain=${OPTARG};;
        s) seed=${OPTARG};;
        p) problem=${OPTARG};;
    esac
done

path="$(pwd)"

declare -A robot_name_dict=( ["Keva"]="MagicGripper" ["CafeWorld"]="Fetch" ["Packing"]="MagneticGripper" ["Jenga"]="Fetch" ["DinnerTable"]="Fetch" )
robot="${robot_name_dict[$domain]}"

if [[ -z "$seed" ]]; then
    seed=0
fi

docker stop env
docker container prune -f

echo "Data Collection for Domain - $domain:"
command="python gen_data.py -d $domain -r $robot"
docker run --name env --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" -w /workspaces/LAMP openrave:working /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command" &
wait

docker stop env
docker container prune -f

echo "Data Augmentation Starting: "
command="python augment_data.py -d $domain"
docker run --name env --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" -w /workspaces/LAMP openrave:working /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command" &
wait

docker stop env
docker container prune -f

echo "Learning RCRs: "
command="python gen_rom.py -d $domain --seed $seed"
docker run --name env --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" -w /workspaces/LAMP openrave:working /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command" &
wait

docker stop env
docker container prune -f

declare -A default_problem_dict=( ["Keva"]="2_pi" ["CafeWorld"]="can_surface" ["Packing"]="can_surface" ["DinnerTable"]="can_surface" ["Jenga"]="2_pi" )
if [[ -z "$problem" ]];then
    problem="${default_problem_dict[$domain]}"
fi

echo "Learning Model and Solving a Problem: $problem"
command="python gen_graph.py -d $domain --seed $seed -t $problem"
docker run --name env --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" -w /workspaces/LAMP openrave:working /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command" &
wait