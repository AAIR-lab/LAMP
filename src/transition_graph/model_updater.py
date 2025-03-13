from itertools import product

import Config
from src.data_structures.Relation import Relation
from src.useful_functions import get_abstract_state
import copy

class ModelUpdater(object): 


    @staticmethod
    def find_missing_relations(goal_abs_state,goal_env_state,current_lifted_relations,current_rcrs, object_dict, iteration_count=1):
        print("finding missing relations")
        current_regions = copy.deepcopy(current_rcrs)
        rel_obj_set = set() 
        for relation in goal_abs_state.true_set: 
            if relation.cr != 0:
                if relation.p1.name not in Config.ROBOT_TYPES and relation.p1.name not in Config.ROBOT_TYPES and relation.p2.name not in Config.DUMMY_TYPES and relation.p2.name not in Config.DUMMY_TYPES:
                    rel_obj_set.add((relation.p1.name,relation.p2.name))
                    rel_obj_set.add((relation.p2.name,relation.p1.name))
        
        goal_rel_obj_set = set()
        obj_set = set()
        for obj in goal_env_state.object_dict: 
            obj_type = obj.split("_")[0]
            if obj_type not in Config.ROBOT_TYPES and obj_type not in Config.DUMMY_TYPES and len(obj.split("_")) != 1:
                obj_set.add(obj)
        obj_pairs = list(product(obj_set,repeat = 2))

        obj_pairs = ModelUpdater.filter_pairs(obj_pairs,goal_env_state)

        cr_num_dict = {}
        for obj1,obj2 in obj_pairs: 
            obj1_type = obj.split("_")[0]
            obj2_type = obj.split("_")[0]
            if (obj1_type,obj2_type) not in cr_num_dict:
                cr_num_dict[(obj1_type,obj2_type)] = None
            

        used_regions = { }
        new_relation_freq = {}
        new_relations_set = set()

        for obj1, obj2 in obj_pairs: 
            if obj1 != obj2 and (obj1,obj2) not in rel_obj_set and (obj1,obj2) not in new_relations_set and (obj2,obj1) not in new_relations_set :
                obj1_type = obj1.split("_")[0]
                obj2_type = obj2.split("_")[0]
                if frozenset([obj1_type,obj2_type]) not in Config.DOMAIN_DISCRETIZER_PARAMS[Config.DOMAIN_NAME].keys():
                    continue
                discretizer = Config.get_discretizer(obj1=obj1_type, obj2 = obj2_type)
                obj1_pose = goal_env_state.object_dict[obj1]
                obj2_pose = goal_env_state.object_dict[obj2]
                rel_pose = goal_env_state.get_relative_pose(obj1_pose,obj2_pose)
                discretized_rel_pose = discretizer.get_discretized_pose(rel_pose,is_relative = True)
                if -1 in discretized_rel_pose:
                    continue
                if (obj1_type,obj2_type) not in used_regions or (obj2_type,obj1_type) not in used_regions: 
                    used_regions[(obj1_type,obj2_type)] = []
                if (obj1_type,obj2_type) in used_regions: 
                    if rel_pose in used_regions[(obj1_type,obj2_type)]:
                        continue
                if (obj2_type,obj1_type) in used_regions: 
                    if rel_pose in used_regions[(obj2_type,obj1_type)]:
                        continue

                if (obj1_type,obj2_type) not in cr_num_dict and (obj2_type,obj1_type) not in cr_num_dict:
                    cr_num_dict[(obj1_type,obj2_type)] = None

                if cr_num_dict[(obj1_type,obj2_type)] is None: 
                    if obj1_type not in current_regions.keys():
                        current_regions[obj1_type] = {}
                    if obj2_type not in current_regions[obj1_type].keys():
                        current_regions[obj1_type][obj2_type] = []

                    cr = len(current_regions[obj1_type][obj2_type])+1
                    cr_num_dict[(obj1_type,obj2_type)] = cr
                else: 
                    cr = cr_num_dict[(obj1_type,obj2_type)] + 1
                    cr_num_dict[(obj1_type,obj2_type)] = cr
                used_regions[(obj1_type,obj2_type)].append(discretized_rel_pose)
                discretized_rel_pose.append(0)
                new_relation = Relation(parameter1_type = obj1_type, parameter2_type = obj2_type, cr = cr, region = [discretized_rel_pose], discretizer = discretizer)
                
                temp_relation_dict = {(obj1_type,obj2_type): {cr:new_relation}}
                ts = get_abstract_state(goal_env_state, object_dict, temp_relation_dict, aux_list = []).true_set
                freq = len(ts)
                # if freq == 3:
                #     print "Freq: ",freq
                #     ModelUpdater.help_print(ts)
                
                # if freq not in new_relation_freq: 
                #     new_relation_freq[freq] = []
                # new_relation_freq[freq].append(temp_relation_dict)

                if freq not in new_relation_freq: 
                    new_relation_freq[freq] = {}
                if (obj1_type,obj2_type) not in new_relation_freq[freq].keys():
                    new_relation_freq[freq][(obj1_type,obj2_type)] = {}
                
                new_relation_freq[freq][(obj1_type,obj2_type)][cr] = new_relation              
                new_relations_set = new_relations_set.union(ModelUpdater.get_object_pairs(ts))

                # print("Freq {}".format(freq))
                # ModelUpdater.help_print(ts)
                
        return new_relation_freq, obj_pairs

    @staticmethod
    def filter_pairs(pairs,env_state): 
        filtered_pairs = set()
        for pair in pairs:
            p1 = pair[0]
            p2 = pair[1]
            p1_type = p1.split("_")[0] 
            p1_num = int(p1.split("_")[1])
            p2_type = p2.split("_")[0]
            p2_num = int(p2.split("_")[1])
            p1_h = round(env_state.object_dict[p1][2],3)
            p2_h = round(env_state.object_dict[p2][2],3)
            if p1 != p2: 
                if p1_type == p2_type:
                    # if p1_num < p2_num and (p1,p2) not in filtered_pairs and (p2,p1) not in filtered_pairs: 
                    if p1_h < p2_h and (p1,p2) not in filtered_pairs and (p2,p1) not in filtered_pairs: 
                        filtered_pairs.add((p1,p2))
                else: 
                    if p1_type in Config.ROBOT_TYPES:
                        filtered_pairs.add((p1,p2))
                    elif p2_type in Config.ROBOT_TYPES:
                        filtered_pairs.add((p2,p1))
                    elif (p1,p2) not in filtered_pairs and (p2,p1) not in filtered_pairs: 
                        if p1_type == "surface" and p2_type == "can":
                            filtered_pairs.add((p2,p1))
                            continue
                        filtered_pairs.add((p1,p2))
        return filtered_pairs


    @staticmethod
    def get_object_pairs(relations):
        obj_pairs = set() 
        for r in relations: 
            obj_pairs.add((r.p1,r.p2))
            obj_pairs.add((r.p2,r.p1))
        return obj_pairs

    @staticmethod
    def help_print(relations):
        print "************"
        print list(relations)[0].region 
        for r in relations: 
            print "----"
            print r.p1
            print r.p2 
            print "----"
        print "***********"
        








        
