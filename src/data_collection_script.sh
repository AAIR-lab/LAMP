#!/bin/bash
while getopts n: flag
do
    case "${flag}" in
        n) number=${OPTARG};;
    esac
done 

for i in $(seq 1 1 $number)
do  
    docker run -dit --name env$i --mount 'type=bind,src=/home/local/ASUAD/jnagpal1/git/LAMP,dst=/workspaces/LAMP' -w /workspaces/LAMP openrave:working /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && python gen_data.py -n env$i -a x -d CafeWorld -p 1"
done
