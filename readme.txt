Software Dependencies: 
    OS - Ubuntu 22.04 or Ubuntu 24.04
    Docker - install instructions - https://docs.docker.com/engine/install/ubuntu/

Setup Instructions:
    download the docker image: wget -P ./ -O lamp_docker.tar.gz "https://www.dropbox.com/scl/fi/rl2oop7p9v0qcninow5r4/lamp_docker.tar.gz?rlkey=3487gjku137t1iup8na6n0e2y&st=mq8xybtd&dl=0"
    load the docker image: docker load < lamp_docker.tar.gz

    Get the planners used in the file from here. - wget -P ./ -O Planners.tar.gz "https://www.dropbox.com/scl/fi/90aff58h3ezfka4kuleif/Planners.tar.gz?rlkey=xcziwwwivrxsm8fxd8e0q41oc&st=vgvdrf02&dl=0"
    Extract them in the root directory of the repo - tar -xzvf ./Planners.tar.gz

Estimated Setup should take roughly 30 minutes depending on internet connection as large tarballs are needed to be downloaded.

once the setup is completed, open a terminal and change the directory to inside the repo directoory and run the command: cp -r ./exec_scripts/* ../
this will copy the scripts to run the pipeline to the parent directory of the LAMP repo.

the following are the scripts and their descriptions.

1. model_learning.sh
this script will run the complete pipeline for a domain at a time, starting from data generation to the model learning part.

usage: ./model_learning.sh -d <DOMAIN_NAME> -s <SEED> -p <PROBLEM_NAME> 
output - 
this script will generate following files:
<REPO_DIR>/Data/<DOMAIN_NAME>/misc/<ENV_DIR>/pickle_files (generated and augmented trajectories used for training)
<REPO_DIR>/Data/<DOMAIN_NAME>/misc/rcr_indices_pickle_files (Learnt relational critical region files and model pickle files used for solving tasks)
<REPO_DIR>/Domains/<DOMAIN_NAME>/pddl_files (generated pddl files for the model and the problem to be solved)

SEED and PROBLEM_NAME are optional arguments that are not required for this.

2. experiment_data_collection.sh
this script is for processing the generated data fom previous script for multiple seeds to run the whole experiment suit.

usage: ./experiment_data_collection.sh -d <DOMAIN_NAME> -s [<SEED>] -p <PROBLEM_NAME> -C [TOTAL_TRAJECTORY_PERCENT>]
output - 
<REPO_DIR>/Data/<DOMAIN_NAME>/misc/rcr_indices_pickle_files (Learnt relational critical region files and model pickle files used for solving tasks for multiple seeds)
<REPO_DIR>/Domains/<DOMAIN_NAME>/pddl_files (generated pddl files for the model and the problem to be solved for multiple seeds)

only DOMAIN_NAME is the required argument. rest are optional. 
TOTAL_TRAJECTORY_PERCENT - the percentage of the total data to process and use for the model learning pipeline.

3. run_experiments.sh
this script will run the experiment suit from the paper the given domain.

usage: ./run_experiments.sh -d <DOMAIN_NAME> -s [<SEED>]
again, only DOMAIN_NAME is the required argument

output - 
<REPO_PARENT_DIR>/<DOMAIN_NAME>_logs/log_json_files (logs files for the experiment suits)
<REPO_PARENT_DIR>/<DOMAIN_NAME>_model_files (model files stored as checkpoints after every solved problems)

Using Our Data:
    Get the Data files used in the file from here. - wget -P ./ -O Data.tar.gz "https://www.dropbox.com/scl/fi/j0n88vc8grwdl423ggd9z/Data.tar.gz?rlkey=n52qmg9kt48t0b1ue492zzxq1&st=9qtypwbs&dl=0"
    Extract them in the root directory of the repo - tar -xzvf ./Data.tar.gz

    this should create the Data directories <REPO_DIR>/Data
    to run the experiments using this with 10 different seeds, run - ./run_experiments.sh -d <DOMAIN_NAME>
