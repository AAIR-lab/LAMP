#!/usr/bin/env bash
while getopts d: flag
do
    case "${flag}" in
        d) domain=${OPTARG};;
    esac
done 

#cloning data directory

path="$(pwd)"
log_folder=$domain"_logs"
domain_model_folder=$domain"_model_files"

cp -rf $path/LAMP/Data/$domain $path/
mkdir -p $path/$log_folder
mkdir -p $path/$domain_model_folder