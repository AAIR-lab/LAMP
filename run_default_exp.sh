#!/usr/bin/env bash

SOURCE="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

domain_array=( "Keva" "Packing" "CafeWorld")

declare -A keva_seed_dict=( ["160"]="1 2 3 4 5 6 7 8 9 10" ["128"]="1 2 3 4 5 6 7 8 9 10" ["80"]="1 2 3 4 5 6 7 8 9 10" ["48"]="1 2 3 4 5 6 7 8 9 10" )

declare -A packing_seed_dict=( ["200"]="1 2 35 7 9 10 12 14 15 17"  ["150"]="1 2 35 7 9 10 12 14 15 17"  ["100"]="1 2 35 7 9 10 12 14 15 17"  ["50"]="1 2 35 7 9 10 11 12 14 17"  )

declare -A cafeworld_seed_dict=( ["200"]="64 66 35 67 5 42 15 18 58 59"  ["150"]="64 66 35 67 5 10 15 18 58 59"  ["100"]="64 1 35 69 70 6 5 9 58 59"  ["50"]="1 2 6 7 8 10 11 12 13 14"  )

declare -A domain_seed_dict

declare -A total_demo_count_dict=( ["Keva"]="160 128 80 48" ["CafeWorld"]="200 150 100 50" ["Packing"]="200 150 100 50" )

#Prepare Data Dir
# cp -r /root/capsule/data/Data $SOURCE/LAMP/

#Prepare Experimentts for All
for d in "${domain_array[@]}"
do
    echo $d
    log_folder=$d"_logs"
    mkdir -p $SOURCE/$log_folder

    read -r -a demo_count_dict <<< "${total_demo_count_dict[$d]}"
    for t in "${demo_count_dict[@]}"
    do
        echo $t
        domain_seed_dict=( ["Keva"]="${keva_seed_dict[$t]}" ["CafeWorld"]="${cafeworld_seed_dict[$t]}" ["Packing"]="${packing_seed_dict[$t]}" )
        seed_dict="${domain_seed_dict[$d]}"

        bash $SOURCE/run_experiments.sh -d $d -T $t -s "${seed_dict[@]}"

        cp -rf $SOURCE/$log_folder ../results/
    done
done