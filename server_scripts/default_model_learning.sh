#!/usr/bin/env bash
while getopts d:s:p: flag
do
    case "${flag}" in
        d) domain=${OPTARG};;
    esac
done

path="$(pwd)"

docker stop env
docker container prune -f

echo "Data Collection for Domain - $domain:"
command="python gen_data.py -d $domain"
docker run --name env --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" -w /workspaces/LAMP lamp /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command" &
wait

docker stop env
docker container prune -f

echo "Data Augmentation Starting: "
command="python augment_data.py -d $domain"
docker run --name env --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" -w /workspaces/LAMP lamp /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command" &
wait

docker stop env
docker container prune -f

echo "Learning RCRs: "
command="python gen_rom.py -d $domain"
docker run --name env --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" -w /workspaces/LAMP lamp /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command" &
wait

docker stop env
docker container prune -f

echo "Learning World Model for $domain"
command="python gen_world_model.py -d $domain -f"
docker run --name env --mount "type=bind,src=$path/LAMP,dst=/workspaces/LAMP" -w /workspaces/LAMP lamp /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/tmp_catkin_ws/devel/setup.bash && $command" &
wait