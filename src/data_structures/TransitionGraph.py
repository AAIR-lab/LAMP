import networkx as nx
import Config
import cPickle
import pickle as cPikcle


class TransitionGraph(object):
    def __init__(self):
        self._g = nx.DiGraph()

    def add_state(self,state):
        self._g.add_node(state)

    def add_transition(self,state1, state2):
        self._g.add_edge(state1,state2) 
    
    def save(self,name=None,added_relations=[],transitions=[]):
        if name is None:
            name = "transition_graph"
        # nx.nx_pydot.write_dot(self._g,name+".dot")
        data_dict = {}
        data_dict["graph"] = self._g
        data_dict["added_relations"] = added_relations
        data_dict["transitions_used"] = transitions

        with open(Config.DATA_MISC_DIR+name+".p","wb") as f:
            cPickle.dump(data_dict,f,protocol=cPickle.HIGHEST_PROTOCOL)
