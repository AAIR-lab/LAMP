import os
import sys
import tqdm
import argparse

def save_abstract_trajectory_text_set(prefix,data_set):
    s = ""

    for traj in data_set:
        s+= str(traj)
        s+= "\n"
    
    with open(Config.DATA_MISC_DIR+prefix+"text_trajectories.txt","w") as f:
        f.write(s)
        f.close()

def get_parent_with_file(file_name):
    current_dir = os.getcwd()
    while True:
        if file_name in os.listdir(current_dir):
            return current_dir
        else:
            current_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
    
    raise Exception("File Not Found in any Parent")

ROOT_DIR = get_parent_with_file("Config.py")
if ROOT_DIR is not None and ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

from src.data_structures.PDDLTrajectory import PDDLTrajectory
import useful_functions
from Config import Config

class TrajAbstracter():
    file_prefix = ""
    rcr_data = None
    relations = None
    object_dictionary = {}
    object_names = set([])

    @staticmethod
    def set_params(file_prefix):
        TrajAbstracter.file_prefix = file_prefix

    @staticmethod
    def reset_traj_data():
        TrajAbstracter.object_dictionary = {}
        TrajAbstracter.relations = None

    @staticmethod
    def get_abstracted_traj(traj,lifted_relations_dict=None,env_id=-1,traj_id=-1):
        if TrajAbstracter.rcr_data is None and lifted_relations_dict is None:
            TrajAbstracter.rcr_data = useful_functions.load_rcrs(TrajAbstracter.file_prefix)

        object_dictionary = useful_functions.get_object_dictionary(traj[0].object_dict.keys())
        object_names = useful_functions.get_object_names_from_object_dict(object_dictionary)

        if not object_names.issubset(TrajAbstracter.object_names):
            TrajAbstracter.object_names.update(object_names)

        if object_dictionary != TrajAbstracter.object_dictionary:
            TrajAbstracter.object_dictionary = useful_functions.merge_dicts(object_dictionary,TrajAbstracter.object_dictionary)

        if lifted_relations_dict is None:
            if TrajAbstracter.relations is None:
                TrajAbstracter.relations = useful_functions.get_lifted_relations_dict(TrajAbstracter.object_names,TrajAbstracter.rcr_data)

            lifted_relations_dict = TrajAbstracter.relations

        aux_list = useful_functions.get_auxilary_preds(TrajAbstracter.object_dictionary,lifted_relations_dict)
        
        lifted_seq = useful_functions.get_abstract_traj(traj,TrajAbstracter.object_dictionary,lifted_relations_dict,aux_list,env_id=env_id,traj_id=traj_id)
        pddl_traj = PDDLTrajectory(lifted_seq)

        return pddl_traj

    @staticmethod
    def save_abstract_traj(env_name,data):
        with open(Config.DATA_MISC_DIR+env_name+"/{}_abstracted_data.p".format(env_name), "wb") as f:
            cPickle.dump(data,f,protocol=cPickle.HIGHEST_PROTOCOL)
            f.close()

    @staticmethod
    def load_abstract_traj(env_name):
        with open(Config.DATA_MISC_DIR+env_name+"/{}_abstracted_data.p".format(env_name), "rb") as f:
            data = cPickle.load(f)
            f.close()

        return data

    @staticmethod
    def get_abstract_traj(env_name):
        if os.path.isfile(Config.DATA_MISC_DIR + "{}/{}_abstracted_traj.p".format(env_name)):
            return TrajAbstracter.load_abstract_traj(env_name)
        
        return TrajAbstracter.generate_abstract_trajectories(env_name)

    @staticmethod
    def generate_abstract_trajectories(env_name,traj_nums=[]):
        # TrajAbstracter.reset_traj_data()
        TrajAbstracter.relations = None
        env_data = useful_functions.load_traj(env_name)
        env_id = int(env_name[3:])
        if len(traj_nums) == 0:
            trajectories = zip(range(len(env_data["env_states"])),env_data["env_states"])
        else:
            trajectories = [(i,env_data["env_states"][i]) for i in traj_nums]
        
        abstract_trajectories = []
        for t,(t_id,traj) in tqdm.tqdm(enumerate(trajectories)):
            abstract_traj = TrajAbstracter.get_abstracted_traj(traj,env_id=env_id,traj_id=t_id)
            abstract_trajectories.append(abstract_traj)

        return abstract_trajectories

    @staticmethod
    def get_abstracted_transition(transition,lifted_relations_dict=None):
        TrajAbstracter.reset_traj_data()
        if TrajAbstracter.rcr_data is None and lifted_relations_dict is None:
            TrajAbstracter.rcr_data = useful_functions.load_rcrs(file_prefix)

        object_name_list = useful_functions.get_object_list_from_env_state(transition[0])
        object_dictionary = useful_functions.get_object_dictionary(transition[0].object_dict.keys())

        if lifted_relations_dict is None:
            if TrajAbstracter.relations is None:
                TrajAbstracter.relations = useful_functions.get_lifted_relations_dict(object_name_list,TrajAbstracter.rcr_data)

            lifted_relations_dict = TrajAbstracter.relations
            
        aux_list = useful_functions.get_auxilary_preds(object_dictionary,lifted_relations_dict)
        
        lifted_seq = tuple(useful_functions.get_abstract_traj(transition,object_dictionary,lifted_relations_dict,aux_list))

        return lifted_seq

if __name__ == "__main__":

    ROOT_DIR = get_parent_with_file("Config.py")
    if ROOT_DIR is not None and ROOT_DIR not in sys.path:
        sys.path.append(ROOT_DIR)

    from src.data_structures.PDDLTrajectory import PDDLTrajectory
    import useful_functions
    from Config import Config

    argParser = argparse.ArgumentParser() 
    argParser.add_argument("-n","--name",                     help = "name of env",                          nargs='*')
    argParser.add_argument("-d","--domain",                    help = "name of domain",                        )
    argParser.add_argument(     "--seed",                     help = "seed to fix",                          nargs='?',default=0)
    argParser.add_argument("-c","--total_traj_count",         help = "total num of demonstrations",          nargs="?")

    args, unknown_args = argParser.parse_known_args()
    
    domain = args.domain
    Config.declare_config(domain)

    seed = int(args.seed)
    env_list = args.name
    if args.total_traj_count is not None:
        total_traj_count = int(args.total_traj_count.split(".")[0])

    file_prefix = "{}_{}_".format(total_traj_count,seed)
    TrajAbstracter.set_params(file_prefix)

    model_data = None
    # model_data = WorldModel.load_model(file_prefix)

    all_objects = useful_functions.get_all_objects(env_list)
    TrajAbstracter.object_names = all_objects
    TrajAbstracter.object_dictionary = useful_functions.get_object_dictionary(all_objects)

    lifted_trajectories_set = set([])
    for env in env_list:
        env_trajectories = useful_functions.load_traj(env)["env_states"]
        env_id = int(env[3:])
        for t,traj in tqdm.tqdm(enumerate(env_trajectories)):
            lifted_relations_dict = None
            if model_data is not None:
                lifted_relations_dict = model_data["og_relations"]

            pddl_traj = TrajAbstracter.get_abstracted_traj(traj,lifted_relations_dict,env_id=env_id,traj_id=t)
            lifted_trajectories_set.add(pddl_traj)

    print(len(lifted_trajectories_set))
    # save_abstract_trajectory_text_set(file_prefix,lifted_trajectories_set)