#!/bin/bash
while getopts n:c: flag
do
    case "${flag}" in
        n) number=${OPTARG};;
        c) command=${OPTARG};;
    esac
done 
# command="'$*'"
echo "$command"
echo "$number"
path="$(pwd)"
docker container prune -f
docker run -it --name env$number --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" -w /workspaces/LAMP openrave:working /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && python $command"