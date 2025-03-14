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

SOURCE="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

log_folder=$domain"_logs"
path="$(pwd)"
domain_model_folder=$domain"_model_files"

bash $path/OLAMP/experiments_setup.sh -d $domain
wait

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

    for j in "${array[@]}"
    do
        seed=$j
        echo "----------- SEED : $seed -------------"
        seed2=0
        seed_model_folder="seed_"$seed"_order_"$seed2

        bash -c "cp -r $SOURCE/$domain/misc/$t""_$seed""_*.p $SOURCE/OLAMP/Data/$domain/misc/ && echo $t files added"
        wait

        echo "different order seed = $seed2"
        bash -c "python $path/OLAMP/timed_experiment.py -n $name -r $robot --domain $domain --problem_num $problem_num --seed $seed --total_demo_count $t --seed2 $seed2"
        wait

        bash -c "mkdir -p $SOURCE/$domain_model_folder/$seed_model_folder && mv $SOURCE/OLAMP/Data/$domain/misc/$t""_$seed""_* $SOURCE/$domain_model_folder/$seed_model_folder/ && echo $t files moved"
        wait
    done
done