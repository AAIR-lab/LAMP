import os
import sys
from networkx import DiGraph
import argparse
import copy

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
from src.data_structures.PDDLTrajectory import PDDLTrajectory
from src.utilities.TrajAbstracter import TrajAbstracter
import useful_functions

import pickle

class TransitionGraph(DiGraph):
    file_prefix = ""
    added_relations = {}
    transitions_used = []
    graph = None
    force_generation = False

    def __init__(self,):
        super(TransitionGraph, self).__init__()

    def add_state(self,state):
        self.add_node(state)

    def add_transition(self,state1, state2):
        self.add_edge(state1,state2) 
    
    def save(self,name=None,added_relations={},transitions_used=[]):
        if name is None:
            name = "transition_graph"

        data_dict = {}
        data_dict["graph"] = self
        data_dict["added_relations"] = added_relations
        data_dict["transitions_used"] = transitions_used

        with open(Config.DATA_MISC_DIR+name+Config.PICKLE_SUFFIX,"wb") as f:
            pickle.dump(data_dict,f,protocol=Config.PICKLE_PROTOCOL )

    @staticmethod
    def get_local_added_relations(local_added_relations, prev_state, next_state):
        new_local_added_relations = copy.deepcopy(local_added_relations)
        for re in next_state.true_set: 
            if re not in prev_state.true_set: 
                new_local_added_relations.add(re)
        
        return new_local_added_relations

    @staticmethod
    def set_params(prefix,force_generation=False):
        TransitionGraph.file_prefix = prefix
        TransitionGraph.force_generation = force_generation

    @staticmethod
    def load_graph_data(prefix=None,name="transition_graph"):
        if prefix is None:
            prefix = TransitionGraph.file_prefix
        
        with open(Config.DATA_MISC_DIR+prefix+name+Config.PICKLE_SUFFIX,"rb") as f:
            graph_data = useful_functions.load_pickle(f)
            f.close()
        
        TransitionGraph.graph = graph_data["graph"]
        TransitionGraph.added_relations = graph_data["added_relations"]
        TransitionGraph.transitions_used = graph_data["transitions_used"] 
 
    @staticmethod
    def get_graph(name="transition_graph"):
        exists = os.path.isfile(Config.DATA_MISC_DIR+TransitionGraph.file_prefix+name+Config.PICKLE_SUFFIX)
        if not exists or TransitionGraph.force_generation:
            TransitionGraph.generate_graph()
        else:
            TransitionGraph.load_graph_data()

        return TransitionGraph.graph, TransitionGraph.added_relations, TransitionGraph.transitions_used

    @staticmethod
    def get_abstract_trajectories(env_dict = None):
        TrajAbstracter.set_params(TransitionGraph.file_prefix)
        abstract_set = set([])

        if env_dict is None:
            mod_name = "gen_graph"
            env_dict = useful_functions.get_used_trajectories(TransitionGraph.file_prefix)       

        all_objects = useful_functions.get_all_objects(env_dict.keys())
        TrajAbstracter.object_names = all_objects
        TrajAbstracter.object_dictionary = useful_functions.get_object_dictionary(all_objects)

        for env in env_dict.keys():
            abstract_trajectories = TrajAbstracter.generate_abstract_trajectories("env{}".format(env),traj_nums=env_dict[env])
            abstract_set.update(set(abstract_trajectories))

        return abstract_set

    @staticmethod
    def get_transitions_from_trajectories(abstract_trajectories):
        abstract_transitions = set([])
        for abstract_traj in abstract_trajectories:
            abstract_transitions.update(set(zip(abstract_traj.traj[:-1],abstract_traj.traj[1:])))

        return abstract_transitions

    @staticmethod
    def generate_graph(env_dict = None):
        TransitionGraph.graph = TransitionGraph()

        required_trajectories = TransitionGraph.get_abstract_trajectories(env_dict)
        for traj in required_trajectories:
            local_added_relations = set([])
            for current_state,next_state in zip(traj.traj[:-1],traj.traj[1:]):
                if current_state not in TransitionGraph.graph.nodes:
                    if current_state not in TransitionGraph.graph.nodes:
                        TransitionGraph.graph.add_state(current_state)
                    if next_state not in TransitionGraph.graph.nodes:
                        TransitionGraph.graph.add_state(next_state)
                    
                    if current_state not in TransitionGraph.added_relations.keys():
                        TransitionGraph.added_relations[current_state] = set([])
                    if next_state not in TransitionGraph.added_relations.keys():
                        TransitionGraph.added_relations[next_state] = set([])
                    
                    local_added_relations = TransitionGraph.get_local_added_relations(local_added_relations, current_state, next_state)
                    if len(TransitionGraph.added_relations[next_state]) == 0:
                        TransitionGraph.added_relations[next_state] = local_added_relations
                    
                    if ((current_state,next_state) not in TransitionGraph.graph.edges):
                        TransitionGraph.graph.add_transition(current_state,next_state)
                    
                    TransitionGraph.added_relations[next_state].intersection_update(local_added_relations)
        
        TransitionGraph.transitions_used = TransitionGraph.get_transitions_from_trajectories(required_trajectories)

        TransitionGraph.graph.save(name=TransitionGraph.file_prefix+"transition_graph",
                                   added_relations=TransitionGraph.added_relations,
                                   transitions_used=TransitionGraph.transitions_used)

if __name__ == "__main__":

    ROOT_DIR = get_parent_with_file("Config.py")
    if ROOT_DIR is not None and ROOT_DIR not in sys.path:
        sys.path.append(ROOT_DIR)

    from Config import Config
    from src.data_structures.PDDLTrajectory import PDDLTrajectory
    from src.utilities.TrajAbstracter import TrajAbstracter
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

    TransitionGraph.set_params(file_prefix,force)
    g,added_relations,transitions_used = TransitionGraph.get_graph()
    g.save(added_relations=added_relations,
           transitions_used=transitions_used)