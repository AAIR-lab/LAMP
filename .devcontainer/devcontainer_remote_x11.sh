#!/bin/sh
rm -rf $HOME/display/
mkdir -p $HOME/display/
cp ./ssh_config $HOME/display/

while getopts i: flag
do
    case "${flag}" in
        i) ip=${OPTARG};;
    esac
done

# export REMOTE_DISPLAY=${ip}:0