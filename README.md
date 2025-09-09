# From Reals to Logic and Back: Learning Generalizable Relational Concepts for Long Horizon Robot Planning

This repository is official implementation for the paper: **From Reals to Logic and Back: Learning Generalizable Relational Concepts for Long Horizon Robot Planning.**

Humans efficiently generalize from limited demonstrations, but robots still struggle to transfer learned knowledge to complex, unseen tasks with longer horizons and increased complexity. We propose the first known method enabling robots to autonomously invent relational concepts directly from small sets of unannotated, unsegmented demonstrations. The learned symbolic concepts are grounded into logic-based world models, facilitating efficient zero-shot generalization to significantly more complex tasks. Empirical results demonstrate that our approach achieves performance comparable to hand-crafted models, successfully scaling execution horizons and handling up to 18 times more objects than seen in training, providing the first autonomous framework for learning transferable symbolic abstractions from raw robot trajectories.

**Authors**: [Naman Shah](https://www.namanshah.net/), [Jayesh Nagpal](https://github.com/jayesh59), [Siddharth Srivastava](https://siddharthsrivastava.net/)<br/>
**Paper**: [link]()<br/>

## Software Dependencies: 
- OS: Ubuntu 22.04 or Ubuntu 24.04 
- Docker: [install instructions](https://docs.docker.com/engine/install/ubuntu/)
- NVidia Container Toolkit: For visualization support, NVidia drivers and container toolkit are expected to be installed. to install run `./setup_container_toolkit.sh` (if not already installed)

## Setup Instructions:
Estimated Setup should take roughly 30 minutes depending on internet connection as large tarballs are needed to be downloaded.
- Build the docker image - `docker build -t lamp .`
- Get the planners used in the file from [here](https://www.dropbox.com/scl/fi/90aff58h3ezfka4kuleif/Planners.tar.gz?rlkey=xcziwwwivrxsm8fxd8e0q41oc&st=vgvdrf02&dl=0)
- Extract them in the root directory of the repo - `tar -xzvf ./Planners.tar.gz`
- To start the development environment in docker container - `docker compose -p lamp up -d`
- Then open a terminal to run scripts - `docker exec -it lamp-container-1 /bin/bash`

## Script Descriptions:
- **gen_data.py**: script to generate data for each domains.
    - **usage**: `python gen_data.py -d <DOMAIN_NAME>`
        - possible domains - `Keva`, `CafeWorld`, `Packing`, `DinnerTable`, `Jenga`.
    - **output directory**: `./Data/<DOMAIN_NAME>/misc/<ENV_DIRS>/`
    - **Optional Args**:
        - `--no_motion_plan`: generate trajectory data without motion plans.
        - `-v, --visualize`: to visualize the simulator during trajectory generation.

- **augment_data.py**: prepares the trajectories for learning RCRs.
    - **usage**: `python augment_data.py -d <DOMAIN_NAME>`
        - possible domains - `Keva`, `CafeWorld`, `Packing`, `DinnerTable`, `Jenga`.
    - **output directory**: `./Data/<DOMAIN_NAME>/misc/<ENV_DIRS>/`

- **gen_rom.py**: uses the processed data to identify Relational Critical Region.
    - **usage**: `python gen_rom.py -d <DOMAIN_NAME>`
        - possible domains - `Keva`, `CafeWorld`, `Packing`, `DinnerTable`, `Jenga`.
    - **output directory**: `./Data/<DOMAIN_NAME>/`
    - **Optional Args**:
        - `--prefix`: the prefix id to use in the files that will be saved after this.. **DEFAULT**: `0`
        - `--seed`: the seed to use while picking the trajectories.
        - `-c, --traj_count`: percentage of trajectories to use as floating poin value between `(0,1]`. 

- **gen_world_model.py**: generates the world model using the trajectories and RCRs generated before.
    - **usage**: `python gen_world_model.py -d <DOMAIN_NAME>`
        - possible domains - `Keva`, `CafeWorld`, `Packing`, `DinnerTable`, `Jenga`.
    - **output directory**: `./Data/<DOMAIN_NAME>/`
    - **Optional Args**:
        - `--prefix`: the prefix id to use in the files that will be saved after this.. **DEFAULT**: `0`
        - `--seed`: the seed to use while picking the trajectories.
        - `--total_traj_count`: the total number of trajectories used for learning RCRs that was saved in earlier step as output of `gen_rom.py`.

- **run_lamp.py**: uses the model generated to solve long horizon problems.
    - **usage**: `python run_lamp.py -d <DOMAIN_NAME> -t <TEST_PROBLEM_NAME>`
        - possible domains - `Keva`, `CafeWorld`, `Packing`, `DinnerTable`, `Jenga`.
        - `-t,--test_structure_name`: the specific problem to run the algorithm for.<br/>
            For possible list of problems for each domain: `python run_lamp.py -d <DOMAIN_NAME> -l`.
    - **Optional Args**:
        - `--prefix`: the prefix id to use in the files that will be saved after this.. **DEFAULT**: `0`
        - `--seed`: the seed to use while picking the trajectories.
        - `-p, --object_count`: number of objects to use in the problem.
        - `-v, --visualize`: to visualize the simulator for execution.
        - `--total_traj_count`: the total number of trajectories used for learning model that was saved in earlier step as output of `gen_rom.py`.