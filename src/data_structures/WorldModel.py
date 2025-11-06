import os
import sys
import copy
import argparse
from itertools import product

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

from Config import Config
from src.utilities.TrajAbstracter import TrajAbstracter
from src.utilities.TransitionClusters import TransitionClusters
from src.data_structures.TransitionGraph import TransitionGraph
from src.data_structures.PDDLAction import LiftedPDDLAction
import useful_functions

import pickle

class WorldModel():
    file_prefix = ""
    force_generation = False

    def __init__(self,og_clusters=set([]), low_level_transitions=[], rcrs={}, relations={}, actions=set([]), added_relations={}):
        self.og_clusters = og_clusters
        self.low_level_transitions = low_level_transitions
        self.rcrs = rcrs
        self.relations = relations
        self.actions = actions
        self.added_relations = added_relations
    
    def get_domain_pddl(self,edited_action_dict={},additional_constants=[],actions_to_use=None):       
        s = "(define (domain {})".format(Config.DOMAIN_NAME)
        s += "\n(:requirements :strips :typing :equality :conditional-effects :existential-preconditions :universal-preconditions)\n"

        object_dictionary = useful_functions.get_default_object_dictionary(useful_functions.get_object_types_from_rcr_dict(self.rcrs))
        types_string = "(:types \n"
        for obj_type in object_dictionary.keys():
            types_string += "\t{}\n".format(obj_type)
        types_string+=")\n"
        
        constants_string = "\n(:constants \n"
        for constant_type,constant_name in product(Config.CONST_TYPES,Config.CONST_NAMES):
            constants_string += "\t{0}_{1}_Const - {0}\n".format(constant_type,constant_name)
        for constant in additional_constants:
            if constant.split("_")[Config.OBJ_TYPE_IND] not in Config.CONST_TYPES:
                constants_string += "\t{} - {}\n".format(constant,constant.split("_")[Config.OBJ_TYPE_IND])
        constants_string += ")\n"
        
        predicate_string = "\n(:predicates \n"
        predicates = self.relations.keys()
        for p in predicates:
            for p_val in self.relations[p].values():
                predicate_string += "\t{}\n".format(p_val.__str__())

        aux_predicate_set = set()
        for ap in useful_functions.get_auxilary_preds(object_dictionary,self.relations):
            if ap.id == 3:
                s_ap = str(ap).split()[0] + " ?x - {}".format(ap.parameter1_type)
            elif ap.id == 4:
                s_ap = str(ap).split()[0] + " ?x - {}".format(ap.parameter2_type)
            else:
                s_ap = str(ap)
            
            aux_predicate_set.add(s_ap)

        for s_ap in aux_predicate_set:        
            predicate_string += "\t({})\n".format(s_ap)

        predicate_string += ")\n\n"

        if actions_to_use is None:
            actions_to_use = self.actions

        action_string = ""
        for a in actions_to_use:
            if a.action_id not in edited_action_dict:
                action_string += "{}\n".format(a.__str__())
            else:
                action_string += "{}\n".format(edited_action_dict[a.action_id].__str__())

        s = s + types_string + constants_string + predicate_string + action_string + "\n)"

        return s
    
    def save_domain_file(self,domain_name,domain=None):        
        if not os.path.exists(Config.PDDL_DIR):
            os.makedirs(Config.PDDL_DIR)

        if domain is None:
            domain = self.get_domain_pddl()

        with open(Config.PDDL_DIR+domain_name,"w") as f:
            f.writelines(domain)
            f.close()

    @staticmethod
    def set_params(file_prefix,force_generation=False):
        WorldModel.file_prefix = file_prefix
        WorldModel.force_generation = force_generation

    @staticmethod
    def load_model(model_num=1):
        name = "{}{}_{}".format(WorldModel.file_prefix,Config.DOMAIN_NAME,model_num)
        with open(Config.DATA_MISC_DIR+name+Config.PICKLE_SUFFIX,"rb") as f:
            model = useful_functions.load_pickle(f)
            f.close()
        
        return model

    @staticmethod
    def save_model(model,model_num=1,name=None):
        with open(Config.DATA_MISC_DIR+WorldModel.file_prefix+"{}_{}{}".format(Config.DOMAIN_NAME,model_num,Config.PICKLE_SUFFIX),"wb") as f:
            pickle.dump(model,f,protocol=Config.PICKLE_PROTOCOL )
            f.close()

        if model_num == 0:
            with open(Config.DATA_MISC_DIR+WorldModel.file_prefix+"{}_{}{}".format(Config.DOMAIN_NAME,1,Config.PICKLE_SUFFIX),"wb") as f:
                pickle.dump(model,f,protocol=Config.PICKLE_PROTOCOL )
                f.close()
        
        if name is not None:
            if not os.path.exists(Config.DATA_MISC_DIR+"{}problem_models".format(WorldModel.file_prefix)):
                os.makedirs(Config.DATA_MISC_DIR+"{}problem_models".format(WorldModel.file_prefix))

            with open(Config.DATA_MISC_DIR+"{}problem_models/".format(WorldModel.file_prefix)+"{}_{}{}".format(Config.DOMAIN_NAME,name,Config.PICKLE_SUFFIX),"wb") as f:
                pickle.dump(model,f,protocol=Config.PICKLE_PROTOCOL)
                f.close()

    @staticmethod
    def get_model(model_num=1):
        exists = os.path.isfile(Config.DATA_MISC_DIR+WorldModel.file_prefix+"{}_{}{}".format(Config.DOMAIN_NAME,model_num,Config.PICKLE_SUFFIX))
        if WorldModel.force_generation or not exists:
            return WorldModel.generate_model()
        else:
            model = WorldModel.load_model(model_num)
            if type(model) == dict:
                return WorldModel(og_clusters=model["og_clusters"],
                                  low_level_transitions=model["low_level_transitions"], 
                                  rcrs=model["rcrs"], 
                                  relations=model["og_relations"], 
                                  actions=model["actions"], 
                                  added_relations={}
                                  )
            return model

    @staticmethod
    def update_model(current_model,low_level_transitions):
        new_abstract_transitions = []
        for ll_transition in low_level_transitions:
            new_abstract_transitions.append(TrajAbstracter.get_abstracted_transition(ll_transition,
                                                                                     lifted_relations_dict=current_model.relations))

        current_added_relations = copy.deepcopy(current_model.added_relations)
        for current_state,next_state in new_abstract_transitions:
            if current_state not in current_added_relations.keys():
                current_added_relations[current_state] = set([])
            if next_state not in current_added_relations.keys():
                current_added_relations[next_state] = set([])
            
            local_added_relations = TransitionGraph.get_local_added_relations(current_added_relations[next_state], current_state, next_state)
            if len(current_added_relations[next_state]) == 0:
                current_added_relations[next_state] = local_added_relations 

            current_added_relations[next_state].intersection_update(local_added_relations)

        current_clusters = copy.deepcopy(current_model.og_clusters)
        clusters = TransitionClusters.generate_transition_clusters(new_abstract_transitions)
        
        old_cluster_keys = set(clusters.keys()).intersection(set(current_model.og_clusters.keys()))
        for cluster_key in old_cluster_keys:
            current_clusters[cluster_key].update(clusters[cluster_key])
            
        for cluster_key in clusters.keys():
            if cluster_key not in current_clusters.keys():
                current_clusters[cluster_key] = copy.deepcopy(clusters[cluster_key])
                del clusters[cluster_key]
        
        actions = []
        for cluster_transitions in current_clusters.values():
            actions.append(LiftedPDDLAction.get_action_from_cluster(cluster=list(cluster_transitions),
                                                                    added_relations=current_added_relations))

        current_model.actions = copy.deepcopy(actions)
        current_model.added_relations = copy.deepcopy(current_added_relations)

        return current_model

    @staticmethod
    def generate_model():
        TransitionClusters.set_params(WorldModel.file_prefix,WorldModel.force_generation)

        rcrs = useful_functions.load_rcrs(WorldModel.file_prefix)
        og_clusters = TransitionClusters.get_clusters()

        if len(TransitionGraph.added_relations.keys()) == 0:
            TransitionGraph.set_params(WorldModel.file_prefix,WorldModel.force_generation)
            TransitionGraph.get_graph()
        added_relations = TransitionGraph.added_relations

        # object_list = useful_functions.get_object_types_from_rcr_dict(rcrs)
        object_list = TrajAbstracter.object_names
        relations = useful_functions.get_lifted_relations_dict(rcrs)

        actions = [LiftedPDDLAction.get_action_from_cluster(list(cluster),added_relations) for cluster in og_clusters.values()]
        low_level_transitions=[]

        return WorldModel(og_clusters=og_clusters,
                          low_level_transitions=[], 
                          rcrs=rcrs, 
                          relations=relations, 
                          actions=actions, 
                          added_relations=added_relations
                          )

    @staticmethod
    def get_latest_model_num():
        files = [f.split("/")[-1] for f in os.listdir(Config.DATA_MISC_DIR) if os.path.isfile(Config.DATA_MISC_DIR+f)]
        prefix = WorldModel.file_prefix+"{}_".format(Config.DOMAIN_NAME)

        return len([f for f in files if f.startswith(prefix)])

if __name__ == "__main__":
    ROOT_DIR = get_parent_with_file("Config.py")
    if ROOT_DIR is not None and ROOT_DIR not in sys.path:
        sys.path.append(ROOT_DIR)

    from Config import Config
    from src.utilities.TrajAbstracter import TrajAbstracter
    from src.utilities.TransitionClusters import TransitionClusters
    from src.data_structures.TransitionGraph import TransitionGraph
    from src.data_structures.PDDLAction import LiftedPDDLAction
    import useful_functions

    argParser = argparse.ArgumentParser() 
    argParser.add_argument("-n","--name",                     help = "name of env",                          nargs='*')
    argParser.add_argument("-d","--domain",                    help = "name of domain",                        )
    argParser.add_argument(     "--prefix",                     help = "prefix to use",                          nargs='?',default=0)
    argParser.add_argument("--total_traj_count",         help = "total num of demonstrations",          nargs="?")
    argParser.add_argument("-f","--force",                     help = "force_generation of model",           action="store_false")
    argParser.add_argument("-g","--gen_model",                     help = "generate only model",           action="store_true")

    args, unknown_args = argParser.parse_known_args()

    domain = args.domain
    Config.declare_config(domain)
    
    force = not (args.force)
    prefix = int(args.prefix)
    env_list = args.name
    gen_model = args.gen_model
    
    if args.total_traj_count is not None:
        total_traj_count = int(args.total_traj_count.split(".")[0])

    file_prefix = "{}_{}_".format(total_traj_count,prefix)

    WorldModel.set_params(file_prefix,force)

    if gen_model:
        WorldModel.generate_model()

    model = WorldModel.get_model()
    print(model.get_domain_pddl())
    WorldModel.save_model(model,0)
    
