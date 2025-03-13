#!/usr/bin/env bash
while getopts d:s:p:T:C: flag
do
    case "${flag}" in
        d) domain=${OPTARG};;
        s) seed=${OPTARG};;
        p) problem=${OPTARG};;
        T) total_demo_count=${OPTARG};;
        C) total_traj_percent=${OPTARG};;
    esac
done

path="$(pwd)"

docker stop env
docker container prune -f

declare -A robot_name_dict=( ["Keva"]="MagicGripper" ["CafeWorld"]="Fetch" ["Packing"]="MagneticGripper" )
robot="${robot_name_dict[$domain]}"

array=()
if [[ -z "$seed" ]]; then
    for i in $(seq 1 1 10)
    do
        array+=($i)
    done
else
    IFS=',' read -r -a array <<< "$seed"
fi

declare -A total_traj_percent_dict=( ["Keva"]=0.5 ["CafeWorld"]=0.125 ["Packing"]=0.4 )
array3=()
if [[ -z "$total_traj_percent" ]]; then
    total_traj_percent="${total_traj_percent_dict[$domain]}"
fi
IFS=',' read -r -a array3 <<< "$total_traj_percent"

declare -A mother_pool=( ["Keva"]=320 ["CafeWorld"]=1600 ["Packing"]=500 )
pool_size="${mother_pool[$domain]}"

declare -A default_problem_dict=( ["Keva"]="2_pi" ["CafeWorld"]="can_surface" ["Packing"]="can_surface" ["DinnerTable"]="can_surface" ["Jenga"]="2_pi" )
problem="${default_problem_dict[$domain]}"

for t in "${array3[@]}"
do
    echo "total_traj_percent = $t"

    for j in "${array[@]}"
    do
        seed=$j
        echo "Learning RCRs for Seed: "
        echo "----------- SEED : $seed -------------"
        command="python gen_rom.py -d $domain --seed $seed -c $t"
        docker run --name env --mount "type=bind,src=$path/OLAMP,dst=/workspaces/OLAMP" -w /workspaces/OLAMP openrave:working /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command" &
        wait

        docker stop env
        docker container prune -f

        total_demo_count=$(expr $t*$pool_size | bc)
        command="python gen_graph.py -d $domain --seed $seed -t $problem --total_demo_count $total_demo_count"
        docker run --name env --mount "type=bind,src=$path/OLAMP,dst=/workspaces/OLAMP" -w /workspaces/OLAMP openrave:working /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command" &
        wait
        
        docker stop env
        docker container prune -f
        
    done
done