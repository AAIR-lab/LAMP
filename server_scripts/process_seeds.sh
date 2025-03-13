#!/bin/bash
while getopts c:s:S: flag
do
    case "${flag}" in
        c) command=${OPTARG};;
        s) seed=${OPTARG};;
        S) seed2=${OPTARG};;
    esac
done 
# command="'$*'"
# echo "$command"

array=()
if [[ -z "$seed" ]]; then
    for i in $(seq 1 1 10)
    do
        array+=($i)
    done
else
    IFS=',' read -r -a array <<< "$seed"
fi

array2=()
if [[ -z "$seed2" ]]; then
    for i in $(seq 1 1 5)
    do
        array2+=($i)
    done
else
    IFS=',' read -r -a array2 <<< "$seed2"
fi

for j in "${array[@]}"
do
    seed=$j
    echo "----------- SEED : $seed -------------"
    for i in "${array2[@]}"
    do
        seed2=$i
        command0=$command" --seed $seed -c $seed2"
        echo $command0
        $command0
    done
done

