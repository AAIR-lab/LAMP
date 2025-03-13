#!/bin/bash
while getopts n:s: flag
do
    case "${flag}" in
        n) number=${OPTARG};;
        s) start=${OPTARG};;
    esac
done 

increment=1
echo "environments from $(($start+$increment)) to $(($start+$number))"
for i in $(seq 1 1 $number)
do  
    env_number=$(($start + $i))
    docker stop env$env_number
done

docker container prune -f
