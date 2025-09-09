from src.data_structures.PDDLState import PDDLState
from src.data_structures.Relation import Relation
from src.data_structures.Link import Link
from src.data_structures.ClearPredicate import ClearPredicate
import sys
import os
from Config import Config
import numpy as np
import heapq
import pickle as cPickle
import subprocess
import json
import copy
from itertools import product
import importlib
import inspect

sim_utils = Config.get_simulator_module("sim_utils")

def import_all_methods_from_module(module, current_module=sys.modules[__name__]):
    for atr in dir(module):
        if inspect.isfunction(getattr(module,atr)) and not hasattr(current_module,atr):
            setattr(current_module,atr,getattr(module,atr))

def get_domain_class(domain_name):
    if domain_name[0] == "h":
        domain_name = domain_name[1:]
        
    mod = importlib.import_module("src.data_gen.{}".format(domain_name))
    DomainClass = getattr(mod,domain_name)

    return DomainClass

def start_process(command_list,file_name): 
    p = subprocess.Popen(command_list, stdout = open("{}.log".format(file_name),"w"), stderr = open("{}.err".format(file_name),"w"))
    return p 

#TODO: replace this with python3 version
def blockPrinting(func):
    def func_wrapper(*args, **kwargs):
        # block all printing to the console
        sys.stdout = open(os.devnull, 'w')
        # call the method in question
        value = func(*args, **kwargs)
        # enable all printing to the console
        sys.stdout = sys.__stdout__
        # pass the return value of the method back
        return value

    return func_wrapper

def get_abstract_state(env_state,object_dictionary, lifted_relations_dictionary, aux_list,ll_state_index=(-1,-1,-1)):
    return PDDLState.get_from_ll(lifted_relations_dict = lifted_relations_dictionary,
                                 object_dict = object_dictionary,
                                 ll_state = env_state,
                                 aux_list = aux_list,
                                 ll_state_index = ll_state_index)

def print_set(set):
    for i in sorted(list(set)):
        print(i)

def get_euclidean_distance(transform1, transform2):
    pos1 = np.array([transform1[:3,3]])
    pos2 = np.array([transform2[:3,3]])

    return np.linalg.norm(pos1-pos2)

def get_relative_bin_arr_size(discretizer):
    relative_bin_arr_size = discretizer.relational_n_bins[:3]
    relative_bin_arr_size.append(sum(np.asarray(discretizer.relational_n_bins[3:]))+1)
    relative_bin_arr_size.extend(Config.SENSOR_BIN_LIST)

    return relative_bin_arr_size

def get_object_dictionary(object_list):
    object_dictionary = {}
    for obj in object_list:
        if obj not in Config.NON_RELATION_OBJECTS:
            obj_type = obj.split("_")[Config.OBJ_TYPE_IND]
            if obj.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES:
                # obj_type = "{}_{}".format(*obj.split("_")[:Config.OBJ_ID_IND])
                obj_type = obj.split("_")[Config.OBJ_TYPE_IND]

            if obj_type not in object_dictionary.keys():
                object_dictionary[obj_type] = set()

            object_name = obj
            if obj.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES:
                object_name = update_to_const(obj)

            link = Link(link_name=object_name,
                        link_type=obj_type)
            object_dictionary[obj_type].add(link)

    return object_dictionary

def get_object_names_from_object_dict(obj_dict):
    obj_set = set([])
    all_objects = []

    for l in obj_dict.values():
        all_objects.extend(list(l)) 
        
    for o in all_objects:
        obj_set.add(o.name)

    return obj_set    
    
def get_object_list_from_env_state(env_state,objects_not_found=[]):
    object_name_list = []
    for obj in env_state.object_dict.keys():
        if obj not in objects_not_found:
            object_name_list.append(obj)
    
    return object_name_list

def get_lifted_relations_dict(env_object_list,rcrs):    
    lifted_relations_dict = {}
    for link1 in rcrs.keys():
        for link2 in rcrs[link1].keys(): 
            link1_list = []
            link2_list = []
            for obj in env_object_list:
                if obj.split("_")[Config.OBJ_TYPE_IND] == link1:
                    link1_list.append(obj)
                if obj.split("_")[Config.OBJ_TYPE_IND] == link2:
                    link2_list.append(obj)
                    
            lifted_regions = rcrs[link1][link2]
            for obj1 in link1_list:
                for obj2 in link2_list:
                    if obj2 != obj1:
                        obj1_type = obj1.split("_")[Config.OBJ_TYPE_IND]
                        obj2_type = obj2.split("_")[Config.OBJ_TYPE_IND]

                        key_string = "{}_{}".format(obj1_type,obj2_type)
                        if key_string not in lifted_relations_dict.keys():
                            lifted_relations_dict[key_string] = {}                            
                            for cr in range(len(lifted_regions)+1):
                                obj1_type, obj2_type = key_string.split("_")
                                if cr == 0:
                                    region = []
                                    for r in lifted_regions:
                                        region.extend(r)
                                else:
                                    region = lifted_regions[cr-1]
                                
                                discretizer = Config.get_discretizer(obj1=obj1_type,obj2=obj2_type)

                                relation = Relation(parameter1_type=obj1_type,
                                                    parameter2_type=obj2_type,
                                                    cr = cr,
                                                    region = region,
                                                    discretizer=discretizer)
                                
                                lifted_relations_dict[key_string][cr] = relation

    return lifted_relations_dict

def merge_dicts(dict1,dict2):
    value_type = type(dict1.values()[0])
    new_dict = dict1
    for k in dict2.keys():
        if k in new_dict:
            new_values = set(dict1[k]).union(set(dict2[k]))
            new_dict[k] = value_type(new_values)

        else:
            new_dict[k] = dict2[k]

    return new_dict

def get_auxilary_preds(object_dictionary,lifted_relations_dict):
    auxilary_set = set()
    non_auxilary_object_set = Config.NON_AUX_OBJECTS_SET
    for object_pair in lifted_relations_dict: 
        obj1, obj2 = object_pair.split("_")[0],object_pair.split("_")[1]
        if not (obj1 in non_auxilary_object_set and obj2 in non_auxilary_object_set):
            for cr in lifted_relations_dict[object_pair]: 
                if cr != 0:
                    lr = lifted_relations_dict[object_pair][cr]
                    # if lr.parameter1_type not in Config.CONST_TYPES and lr.parameter2_type not in Config.CONST_TYPES:
                    for obj in object_dictionary[lr.parameter1_type]:
                        if obj.name.split("_")[Config.OBJ_TYPE_IND] not in non_auxilary_object_set:
                            aux3 = ClearPredicate(parameter1_type=lr.parameter1_type,
                                                  parameter2_type=lr.parameter2_type,
                                                  cr = lr.cr,
                                                  id = 3,
                                                  parameter=obj.name)
                        
                            auxilary_set.add(aux3)
                    
    return auxilary_set

def get_abstract_traj(traj,object_dictionary,lifted_relations_dict,aux_list,env_id=-1,traj_id=-1):
    abstract_traj = []
    abstract_state = get_abstract_state(env_state=traj[0],
                                        object_dictionary=object_dictionary,
                                        lifted_relations_dictionary=lifted_relations_dict,
                                        aux_list = aux_list,
                                        ll_state_index=(traj_id,0))
    abstract_traj.append(abstract_state)

    for e,env_state in enumerate(traj):
        abstract_state = get_abstract_state(env_state=env_state,
                                            object_dictionary=object_dictionary,
                                            lifted_relations_dictionary=lifted_relations_dict,
                                            aux_list = aux_list,
                                            ll_state_index=(env_id,traj_id,e))
        
        if abstract_state != abstract_traj[-1]:
            abstract_traj.append(abstract_state)
    
    return abstract_traj

def load_traj(env):
    with open(Config.DATA_MISC_DIR+"{0}/{0}_data.p".format(env)) as f:
        traj_data = cPickle.load(f)
        f.close()
    return traj_data

def load_augmented_traj(env):
    with open(Config.DATA_MISC_DIR+"{0}/{0}_binned_data.p".format(env)) as f:
        traj_data = cPickle.load(f)
        f.close()
    return traj_data

def load_augmented_traj(env):
    with open(Config.DATA_MISC_DIR+"{0}/{0}abstract_data.p".format(env)) as f:
        traj_data = cPickle.load(f)
        f.close()
    return traj_data

def load_rcrs(file_prefix):
    data_load = cPickle.load(open(Config.DATA_MISC_DIR+"{}rcr_indices.p".format(file_prefix)))
    return data_load

def load_model(file_prefix,model_num=1):
    name = "{}{}_{}".format(file_prefix,Config.DOMAIN_NAME,model_num)
    with open(Config.DATA_MISC_DIR+name+".p","rb") as f:
        data = cPickle.load(f)
        f.close()
    return data

def get_problem_list(module_name):
    file_name = Config.ROOT_DIR + "argument_files/" + module_name + ".json"

    with open(file_name,"r") as f:
        data_dict = json.load(f)
        f.close()
    
    return data_dict[Config.DOMAIN_NAME]["problems"].keys()

def get_argument_dict(module_name,problem_name="",only_problems=False):
    file_name = Config.ROOT_DIR + "argument_files/" + module_name + ".json"

    with open(file_name,"r") as f:
        data_dict = json.load(f)
        f.close()
    
    domain_dict = data_dict[Config.DOMAIN_NAME]["general"]
    if problem_name != "":
        problems_dict = data_dict[Config.DOMAIN_NAME]["problems"][problem_name]      
        if not only_problems:
            domain_dict.update(problems_dict)
        else:
            domain_dict = problems_dict

    return domain_dict
    
def changed_relations_from_transition(state1,state2):
    added_relations = []
    deleted_relations =[]
    added_auxilaries = []
    deleted_auxilaries = []

    for re2 in state2.true_set:
        if re2 in state1.false_set:
            added_relations.append(re2)
    
    for re2 in state2.false_set:
        if re2 in state1.true_set:
            deleted_relations.append(re2)

    for a_re in state2.aux_true_set:
        if a_re not in state1.aux_true_set:
            added_auxilaries.append(a_re)
        
    for a_re in state1.aux_true_set:
        if a_re not in state2.aux_true_set:
            deleted_auxilaries.append(a_re)

    return added_relations,deleted_relations,added_auxilaries,deleted_auxilaries

def get_used_trajectories(file_prefix):
    with open(Config.DATA_MISC_DIR+file_prefix+"trajs_used.p") as f:
        data = cPickle.load(f)
        f.close()

    return data

def get_object_types_from_rcr_dict(rcr_dict):
    object_types = set([])

    for obj in rcr_dict.keys():
        object_types.add(obj)
        object_types.update(set(rcr_dict[obj].keys()))

    return object_types

def get_default_object_dictionary(object_types):
    object_list = [obj+"_1" for obj in object_types if obj not in Config.CONST_TYPES]

    const_names = product(Config.CONST_TYPES,Config.CONST_NAMES)
    object_list.extend(["{}_{}_1".format(c_type,c_name) for c_type,c_name in const_names])
            
    return get_object_dictionary(object_list)

def get_action_list(action_strings,model_actions,object_dictionary,planner,object_list=[]):
    grounded_solutions = []
    for actions in action_strings:
        lifted_action_list = []
        grounded_action_list = []
        for a_string in actions:
            action_id = int(a_string[1:-1].split()[0][1:])
            parameters = a_string[1:-1].split()[1:]
            for action in model_actions:
                if action.action_id == action_id:
                    lifted_action_list.append(action)
                    break
            grounding = { }
            for i,param in enumerate(action.parameters):
                for l in object_dictionary[param.type]:
                    if planner == "FF":
                        name = l.name.upper()
                    else:
                        name = l.name
                    if name.upper() == parameters[i].upper():
                        grounding[param.pid] = l
                        break
            grounded_action_list.append(action.get_grounded_action(grounding,
                                        lifted_action_id=action_id,
                                        object_list=object_list))
        
        grounded_solutions.append(grounded_action_list)

    return grounded_solutions

def get_object_list_from_dict(object_dictionary):
    object_list = []
    for link_list in object_dictionary.values():
        object_list.extend([o.name for o in link_list])

    return object_list

def get_obj_type(obj_name): #TODO: think of a better name
    return ("{}_"*len(obj_name.split("_")[:Config.OBJ_ID_IND])).format(*obj_name.split("_")[:Config.OBJ_ID_IND])[:-1]

def update_to_const(param_name):
    return ("{}_"*len(param_name.split("_")[:Config.OBJ_ID_IND])).format(*param_name.split("_")[:Config.OBJ_ID_IND]) + "Const"

def get_lifted_relations_in_state(env_state,lifted_relations_dict):
    re_dict = {}
    aux_present = set([])

    obj_list = get_object_list_from_env_state(env_state)
    object_dictionary = get_object_dictionary(obj_list)
    aux_list = get_auxilary_preds(object_dictionary=object_dictionary, lifted_relations_dict=lifted_relations_dict)
    
    abstract_state = get_abstract_state(env_state,object_dictionary, lifted_relations_dict, aux_list)

    for key in lifted_relations_dict.keys():
        if key != "{0}_{0}".format(Config.OBJECT_NAME[0]):
            re_dict[key] = copy.deepcopy(lifted_relations_dict[key])
        else:
            re_dict[key] = {}

    if "{0}_{0}".format(Config.OBJECT_NAME[0]) in lifted_relations_dict.keys():
        for re in abstract_state.true_set:
            if re.get_lifted_relation() in lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])].values():
                re_dict["{0}_{0}".format(Config.OBJECT_NAME[0])][re.cr] = re.get_lifted_relation()
        
        cr_null_region = []
        for re in lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])].values():
            if re.cr != 0:
                cr_null_region.extend(re.region)
        re_dict["{0}_{0}".format(Config.OBJECT_NAME[0])][0].region = copy.deepcopy(cr_null_region)

    for re in aux_list:
        if re.cr in re_dict["{}_{}".format(re.parameter1_type,re.parameter2_type)].keys():
            aux_present.add(re)
    
    return re_dict, aux_present

def get_all_objects(env_list):
    objects_set = set([])
    for env in env_list:
        if "env" not in str(env):
            env_name = "env{}".format(env)
        else:
            env_name = env

        with open(Config.DATA_MISC_DIR+"/{0}/{0}_object_list.p".format(env_name),"rb") as f:
            objects_set.update(cPickle.load(f))

    return objects_set

class Node(object):
    def __init__(self,name,parent):
        self.name = name
        self.parent = parent
    
class Stack:
    "A container with a last-in-first-out (LIFO) queuing policy."
    def __init__(self):
        self.list = []

    def push(self,item):
        "Push 'item' onto the stack"
        self.list.append(item)

    def pop(self):
        "Pop the most recently pushed item from the stack"
        return self.list.pop()

    def isEmpty(self):
        "Returns true if the stack is empty"
        return len(self.list) == 0

import_all_methods_from_module(sim_utils)
