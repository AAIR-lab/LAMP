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

echo "Data Collection for Domain - $domain:"
bash -c "python gen_data.py -d $domain -r $robot"
wait

echo "Data Augmentation Starting: "
bash -c "python augment_data.py -d $domain"
wait

echo "Learning RCRs: "
bash -c "python gen_rom.py -d $domain --seed $seed"
wait

declare -A default_problem_dict=( ["Keva"]="2_pi" ["CafeWorld"]="can_surface" ["Packing"]="can_surface" ["DinnerTable"]="can_surface" ["Jenga"]="2_pi" )
if [[ -z "$problem" ]];then
    problem="${default_problem_dict[$domain]}"
fi

echo "Learning Model and Solving a Problem: $problem"
bash -c "python gen_graph.py -d $domain --seed $seed -t $problem --get_graph"
wait