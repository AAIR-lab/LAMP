#!/bin/bash
while getopts n:s:d: flag
do
    case "${flag}" in
        n) number=${OPTARG};;
        s) start=${OPTARG};;
        d) domain=${OPTARG};;
    esac
done 

docker container prune -f
increment=1
echo "environments from $(($start+$increment)) to $(($start+$number))"
echo "domain = $domain"
path="$(pwd)"
for i in $(seq 1 1 $number)
do  
    env_number=$(($start + $i))
    docker run -dit --name env$env_number --mount "type=bind,src=$path/OLAMP,dst=/workspaces/OLAMP" -w /workspaces/OLAMP openrave:working /bin/bash -c "rm /workspaces/OLAMP/Data/$domain/misc/env$env_number/*" 
done
sleep 2
docker container prune -f