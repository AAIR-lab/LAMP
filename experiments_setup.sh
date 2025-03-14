#!/usr/bin/env bash
while getopts d: flag
do
    case "${flag}" in
        d) domain=${OPTARG};;
    esac
done 

#cloning data directory
path="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
log_folder=$domain"_logs"
domain_model_folder=$domain"_model_files"

cp -rf $path/Data/$domain ../
mkdir -p ../$log_folder
mkdir -p ../$domain_model_folder
cp $path/run_experiments.sh ../