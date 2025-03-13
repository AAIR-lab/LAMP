#!/usr/bin/env bash
while getopts n:r:d:s:T:P:o:S: flag
do
    case "${flag}" in
        n) name=${OPTARG};;
        d) domain=${OPTARG};;
        s) seed=${OPTARG};;
        T) total_demo_count=${OPTARG};;
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

array=()
if [[ -z "$seed" ]]; then
    for i in $(seq 1 1 10)
    do
        array+=($i)
    done
else
    IFS=',' read -r -a array <<< "$seed"
fi

echo "name = env$name"
echo "primary seed = $seed"

declare -A robot_name_dict=( ["Keva"]="yumi" ["CafeWorld"]="Fetch" ["Packing"]="MagneticGripper" )
robot="${robot_name_dict[$domain]}"

array3=()
if [[ -z "$total_demo_count" ]]; then
    declare -A total_demo_count_dict=( ["Keva"]=160 ["CafeWorld"]=200 ["Packing"]=200 )
    total_demo_count="${total_demo_count_dict[$domain]}"
fi
IFS=',' read -r -a array3 <<< "$total_demo_count"

for t in "${array3[@]}"
do
    echo "total_demo_count = $t"
    echo "robot = $robot"
    command0="rm -rf /workspaces/LAMP/Data/$domain/misc/$t""_$seed""_* && echo $t files deleted" 
    docker run -it --name env$name --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" --mount "type=bind,src=$path/$domain,dst=/workspaces/$domain" -w /workspaces/LAMP openrave:working /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command0"
    docker container prune -f  

    for j in "${array[@]}"
    do
        seed=$j
        echo "----------- SEED : $seed -------------"
        seed2=0
        seed_model_folder="seed_"$seed"_order_"$seed2

        command1="cp -r /workspaces/$domain/misc/$t""_$seed""_*.p /workspaces/LAMP/Data/$domain/misc/ && echo $t files added"
        docker run -it --name env$name --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" --mount "type=bind,src=$path/$domain,dst=/workspaces/$domain" -w /workspaces/LAMP openrave:working /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command1"
        docker container prune -f   

        # command1="cp -r /workspaces/$domain_model_folder/$seed_model_folder/$t""_*.p /workspaces/LAMP/Data/$domain/misc/"
        # docker run -it --name env$name --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" --mount "type=bind,src=$path/$domain,dst=/workspaces/$domain" --mount "type=bind,src=$path/$domain_model_folder,dst=/workspaces/$domain_model_folder" -w /workspaces/LAMP openrave:working /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command1"
        # docker container prune -f 

        echo "different order seed = $seed2"
        docker run -it --name env$name --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" --mount "type=bind,src=$path/$log_folder,dst=/workspaces/$log_folder" -w /workspaces/LAMP openrave:working /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && python timed_experiment.py -n $name -r $robot_name --domain $domain --problem_num $problem_num --seed $seed --object_count $object_count --total_demo_count $t --seed2 $seed2"
        sleep 2
        docker container prune -f    

        command2="mkdir -p /workspaces/$domain_model_folder/$seed_model_folder && mv /workspaces/LAMP/Data/$domain/misc/$t""_$seed""_* /workspaces/$domain_model_folder/$seed_model_folder/ && echo $t files moved"
        docker run -it --name env$name --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" --mount "type=bind,src=$path/$domain,dst=/workspaces/$domain" --mount "type=bind,src=$path/$domain_model_folder,dst=/workspaces/$domain_model_folder" -w /workspaces/LAMP openrave:working /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command2"
        docker container prune -f   
    done
done