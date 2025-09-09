#!/usr/bin/env bash
while getopts n:r:d:s:T:P:o:S: flag
do
    case "${flag}" in
        n) name=${OPTARG};;
        d) domain=${OPTARG};;
        s) seed=${OPTARG};;
        p) prefix=${OPTARG};;
        T) total_traj_count=${OPTARG};;
        P) problem_num=${OPTARG};;
    esac
done 

if [[ -z "$name" ]];then
    name=1
fi

docker container prune -f

log_folder=$domain"_logs"
path="$(pwd)"
domain_model_folder=$domain"_model_files"

seed_array=()
if [[ -z "$seed" ]]; then
    for i in $(seq 1 1 10)
    do
        seed_array+=($i)
    done
else
    IFS=',' read -r -a seed_array <<< "$seed"
fi

prefix_array=()
if [[ -z "$prefix" ]]; then
    for i in $(seq 1 1 10)
    do
        prefix_array+=($i)
    done
else
    IFS=',' read -r -a prefix_array <<< "$prefix"
fi

length=${#seed_array[@]}

echo "name = env$name"
echo "primary seed = $seed"

declare -A robot_name_dict=( ["Keva"]="yumi" ["CafeWorld"]="Fetch" ["Packing"]="MagneticGripper" )
robot="${robot_name_dict[$domain]}"

demo_count_array=()
if [[ -z "$total_traj_count" ]]; then
    declare -A total_traj_count_dict=( ["Keva"]=160 ["CafeWorld"]=200 ["Packing"]=200 )
    total_traj_count="${total_traj_count_dict[$domain]}"
fi
IFS=',' read -r -a demo_count_array <<< "$total_traj_count"

for t in "${demo_count_array[@]}"
do
    echo "total_traj_count = $t"
    echo "robot = $robot"
    docker container prune -f  

    for (( j=0; j<$length; j++ )); do
    do
        prefix="${prefix_array[j]}"
        seed="${seed_array[j]}"

        echo "----------- PREFIX : $prefix -------------"
        seed2=0
        prefix_model_folder="prefix_"$prefix"_order_"$seed2

        command1="cp -r /workspaces/$domain/misc/$t""_$prefix""_*.p /workspaces/LAMP/Data/$domain/misc/ && echo $t files added"
        docker run --name env$name --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" --mount "type=bind,src=$path/$domain,dst=/workspaces/$domain" -w /workspaces/LAMP lamp /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command1"
        docker container prune -f   

        echo "different order seed = $seed2"
        docker run --name env$name --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" --mount "type=bind,src=$path/$log_folder,dst=/workspaces/$log_folder" -w /workspaces/LAMP lamp /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && python timed_experiment.py -n $name -r $robot --domain $domain --problem_num $problem_num --seed $seed --total_traj_count $t --seed2 $seed2 --prefix $prefix"
        sleep 2
        docker container prune -f    

        command2="mkdir -p /workspaces/$domain_model_folder/$prefix_model_folder && mv /workspaces/LAMP/Data/$domain/misc/$t""_$prefix""_* /workspaces/$domain_model_folder/$prefix_model_folder/ && echo $t files moved"
        docker run --name env$name --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" --mount "type=bind,src=$path/$domain,dst=/workspaces/$domain" --mount "type=bind,src=$path/$domain_model_folder,dst=/workspaces/$domain_model_folder" -w /workspaces/LAMP lamp /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command2"
        docker container prune -f   
    done
done