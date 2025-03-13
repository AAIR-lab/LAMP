import cPickle
from params import *
from itertools import product

from src.data_structures.Link import Link
from src.data_structures.Relation import Relation
from src.data_structures.discretizer import Discretizer

def get_discretizer(obj1,obj2):
    key_set = frozenset([obj1,obj2])
    return Discretizer(*DOMAIN_DISCRETIZER_PARAMS[DOMAIN_NAME][key_set])

def get_lifted_relations_dict(env_object_list,rcrs):
    lifted_relations_dict = {}
    for link1 in rcrs.keys(): 
        for link2 in rcrs[link1].keys(): 
            link1_list = []
            link2_list = []
            for obj in env_object_list:
                if obj.split("_")[0] == link1:
                    link1_list.append(obj)
                if obj.split("_")[0] == link2:
                    link2_list.append(obj)
                    
            lifted_regions = rcrs[link1][link2]
            for obj1 in link1_list:
                for obj2 in link2_list:
                    if obj2 != obj1:
                        obj1_type = obj1.split("_")[0]
                        obj2_type = obj2.split("_")[0]

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
                                
                                discretizer = get_discretizer(obj1=obj1_type,obj2=obj2_type)

                                relation = Relation(parameter1_type=obj1_type,
                                                    parameter2_type=obj2_type,
                                                    cr = cr,
                                                    region = region,
                                                    discretizer=discretizer)
                                
                                lifted_relations_dict[key_string][cr] = relation

    return lifted_relations_dict

def get_grounded_relations_dict(lifted_relations_dict,object_dict):
    grounded_relations_dict = {}
    for object_pair in lifted_relations_dict: 
        if object_pair not in grounded_relations_dict.keys():
            grounded_relations_dict[object_pair] = {}
        for cr in lifted_relations_dict[object_pair]: 
            if cr not in grounded_relations_dict[object_pair].keys():
                grounded_relations_dict[object_pair][cr] = []
            relation = lifted_relations_dict[object_pair][cr]
            l1 = object_dict[relation.parameter1_type]
            l2 = object_dict[relation.parameter2_type]
            combinations = product(l1,l2)
            for combination in combinations:
                grounded_relation = relation.get_grounded_relation(combination[0],combination[1])
                grounded_relations_dict[object_pair][cr].append(grounded_relation)

    return grounded_relations_dict
    
def load_rcrs():
    with open(DATA_DIR+"rcr_indices.p","r") as f:
        rcrs = cPickle.load(f)
        f.close()
    return rcrs["rcr_dict"]

def get_object_dictionary(object_list):
    object_dictionary = {}
    for obj in object_list:
        if obj not in NON_RELATION_OBJECTS:
            obj_type = obj.split("_")[0]
            if obj_type not in object_dictionary.keys():
                object_dictionary[obj_type] = set()

            object_name = obj
            if obj_type in CONST_TYPES[DOMAIN_NAME]:
                object_name = obj_type+"_Const"

            link = Link(link_name=object_name,
                        link_type=obj_type)
            object_dictionary[obj_type].add(link)

    return object_dictionary