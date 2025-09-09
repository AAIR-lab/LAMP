#!/usr/bin/env bash

SOURCE="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

wget -P $SOURCE/ -O Planners.tar.gz "https://www.dropbox.com/scl/fi/90aff58h3ezfka4kuleif/Planners.tar.gz?rlkey=xcziwwwivrxsm8fxd8e0q41oc&st=vgvdrf02&dl=0"
tar -xzvf $SOURCE/Planners.tar.gz
