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
        bash -c "python gen_rom.py -d $domain --seed $seed -c $t"
        wait

        total_demo_count=$(expr $t*$pool_size | bc)
        bash -c "python gen_graph.py -d $domain --seed $seed -t $problem --total_demo_count $total_demo_count --get_graph"
        wait
    done
done