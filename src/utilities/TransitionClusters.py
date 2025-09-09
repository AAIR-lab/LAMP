import os
import sys
import tqdm
import copy
import argparse
import cPickle

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

import useful_functions
from src.data_structures.ParameterizedLiftedRelation import ParameterizedLiftedRelation
from src.data_structures.ChangedRelations import ChangedRelations
from src.data_structures.TransitionGraph import TransitionGraph
from Config import Config

class TransitionClusters():
    file_prefix = ""
    force_generation = False

    @staticmethod
    def set_params(file_prefix,force_generation=False):
        TransitionClusters.file_prefix = file_prefix
        TransitionClusters.force_generation = force_generation

    @staticmethod
    def generate_transition_clusters(transitions):
        print("generating clusters")
        transition_clusters = {}
        for transition in tqdm.tqdm(transitions):
            added_relations, \
            deleted_relations, \
            added_auxilaries, \
            deleted_auxilaries = useful_functions.changed_relations_from_transition(*transition)

            added_relations.sort()
            deleted_relations.sort()
            added_auxilaries.sort()
            deleted_auxilaries.sort()

            param_added_relations = []
            param_deleted_relations = []
            param_added_aux_relations = []
            param_deleted_aux_relations = []

            param_dict = {}
            for re in added_relations[::-1]:
                if re.parameter1 not in param_dict.keys():
                    param_dict[re.parameter1] = "p{}".format(len(param_dict.values()))
                    if re.parameter1_type in Config.CONST_TYPES:
                        param_dict[re.parameter1] = useful_functions.update_to_const(re.parameter1)

                if re.parameter2 not in param_dict.keys():
                    param_dict[re.parameter2] = "p{}".format(len(param_dict.values()))
                    if re.parameter2_type in Config.CONST_TYPES:
                        param_dict[re.parameter2] = useful_functions.update_to_const(re.parameter2)
                        
            for re in deleted_relations[::-1]:
                if re.parameter1 not in param_dict.keys():
                    param_dict[re.parameter1] = "p{}".format(len(param_dict.values()))
                    if re.parameter1_type in Config.CONST_TYPES:
                        param_dict[re.parameter1] = useful_functions.update_to_const(re.parameter1)

                if re.parameter2 not in param_dict.keys():
                    param_dict[re.parameter2] = "p{}".format(len(param_dict.values()))
                    if re.parameter2_type in Config.CONST_TYPES:
                        param_dict[re.parameter2] = useful_functions.update_to_const(re.parameter2)
            
            for re in added_auxilaries[::-1]:
                if re.parameter not in param_dict.keys():
                    param_dict[re.parameter] = "p{}".format(len(param_dict.values()))
                    if re.parameter.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES:
                        param_dict[re.parameter] = useful_functions.update_to_const(re.parameter)
                
                a = copy.deepcopy(re)
                a.parameter = param_dict[re.parameter]
                param_added_aux_relations.append(a)

            for re in deleted_auxilaries[::-1]:
                if re.parameter not in param_dict.keys():
                    param_dict[re.parameter] = "p{}".format(len(param_dict.values()))
                    if re.parameter.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES:
                        param_dict[re.parameter] = useful_functions.update_to_const(re.parameter)
                
                a = copy.deepcopy(re)
                a.parameter = param_dict[re.parameter]
                param_deleted_aux_relations.append(a)
            
            for re in added_relations:
                r = ParameterizedLiftedRelation(param_dict[re.parameter1],
                                                param_dict[re.parameter2],
                                                re.get_lifted_relation())
                
                param_added_relations.append(r)

            for re in deleted_relations:
                r = ParameterizedLiftedRelation(param_dict[re.parameter1],
                                                param_dict[re.parameter2],
                                                re.get_lifted_relation())
                
                param_deleted_relations.append(r)
            
            param_added_relations.sort()
            param_deleted_relations.sort()
            param_added_aux_relations.sort()
            param_deleted_aux_relations.sort()

            changed_relations_key = ChangedRelations(changed_lifted_added_relations=list(set(param_added_relations)),
                                                    changed_lifted_deleted_relations=list(set(param_deleted_relations)),
                                                    added_auxilary_relations=list(set(param_added_aux_relations)),
                                                    deleted_auxilary_relations=list(set(param_deleted_aux_relations)))
            
            if changed_relations_key not in transition_clusters.keys():
                transition_clusters[changed_relations_key] = set([])
            
            transition_clusters[changed_relations_key].add(transition)
        
        return transition_clusters

    @staticmethod
    def load_clusters():
        with open(Config.DATA_MISC_DIR+TransitionClusters.file_prefix+"transition_clusters.p","rb") as f:
            transition_clusters = cPickle.load(f)
            f.close()
        
        return transition_clusters

    @staticmethod
    def save_clusters(transition_clusters):
        with open(Config.DATA_MISC_DIR+TransitionClusters.file_prefix+"transition_clusters.p","wb") as f:
            cPickle.dump(transition_clusters,f,protocol=cPickle.HIGHEST_PROTOCOL)
            f.close()

    @staticmethod
    def get_clusters(transitions=None):
        exists = os.path.isfile(Config.DATA_MISC_DIR+TransitionClusters.file_prefix+"transition_clusters.p")
        if not exists or TransitionClusters.force_generation:
            if transitions is None:
                TransitionGraph.set_params(TransitionClusters.file_prefix,TransitionClusters.force_generation)
                _, _, transitions = TransitionGraph.get_graph()

            tc = TransitionClusters.generate_transition_clusters(transitions)
            TransitionClusters.save_clusters(tc)

            return tc

        else:
            return TransitionClusters.load_clusters()

if __name__ == "__main__":
    ROOT_DIR = get_parent_with_file("Config.py")
    if ROOT_DIR is not None and ROOT_DIR not in sys.path:
        sys.path.append(ROOT_DIR)

    from Config import Config
    from src.data_structures.PDDLTrajectory import PDDLTrajectory
    import useful_functions

    argParser = argparse.ArgumentParser() 
    argParser.add_argument("-n","--name",                     help = "name of env",                          nargs='*')
    argParser.add_argument("-d","--domain",                    help = "name of domain",                        )
    argParser.add_argument(     "--seed",                     help = "seed to fix",                          nargs='?',default=0)
    argParser.add_argument("-c","--total_traj_count",         help = "total num of demonstrations",          nargs="?")
    argParser.add_argument("-f","--force",                     help = "force_generation of model",           action="store_true")

    args, unknown_args = argParser.parse_known_args()

    domain = args.domain
    Config.declare_config(domain)

    force = args.force
    seed = int(args.seed)
    env_list = args.name
    if args.total_traj_count is not None:
        total_traj_count = int(args.total_traj_count.split(".")[0])

    file_prefix = "{}_{}_".format(total_traj_count,seed)

    TransitionClusters.set_params(file_prefix,force)
    tc = TransitionClusters.get_clusters()

    required_string = ""
    for i,cluster_key in enumerate(tc.keys()):
        required_string += "action_{} :\n".format(i+1)
        for current_state,next_state in tc[cluster_key]:
            required_string += "\t"
            required_string += "[{}]".format(str(current_state))
            required_string += " --> "
            required_string += "[{}]".format(str(next_state))
            required_string += "\n"
        
        required_string += "\n"

    with open(Config.ROOT_DIR+"abstract_trajectories/{}_".format(domain)+"action_names.txt","w") as f:
        f.write(required_string)
        f.close() 