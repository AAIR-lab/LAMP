from src.data_structures.ParameterizedLiftedRelation import ParameterizedLiftedRelation
from src.data_structures.Parameter import Parameter
from src.data_structures.Link import Link
from src.data_structures.PDDLPrecondition import LiftedPDDLPrecondition
from src.data_structures.PDDLEffect import LiftedPDDLEffect
import copy
from Config import Config
import numpy as np
import functools
# from openravepy.misc import DrawAxes

@functools.total_ordering
class LiftedPDDLAction(object): 
    action_id  = 0
    def __init__(self,id, parameters, preconditions, effects, required_planks=set([]),states_to_neglect=set([])):
        self.action_id = id
        self.parameters = parameters
        self.preconditions = preconditions 
        self.effects = effects 
        self.required_planks = required_planks
        self.states_to_neglect = states_to_neglect

    @staticmethod
    def get_robot_id_set(relation):
        id_set = set([])
        if relation.parameter1_type in Config.ROBOT_TYPES:
            id_set.add(int(relation.parameter1.split("_")[Config.OBJ_ID_IND]))

        if relation.parameter2_type in Config.ROBOT_TYPES:
            id_set.add(int(relation.parameter2.split("_")[Config.OBJ_ID_IND]))

        return id_set
    
    @staticmethod
    def get_param_objects(param_objects_set,additional_param_objects_dict):
        param_objects = copy.deepcopy(param_objects_set)
        for obj_type in additional_param_objects_dict.keys():
            param_objects = param_objects.union(set(additional_param_objects_dict[obj_type]))

        return param_objects

    @staticmethod
    def get_action_from_cluster(cluster,added_relations):
        cluster_e_add = set()
        cluster_e_delete = set()
        changed_relations = set()
        cluster_aux_true = set()
        cluster_aux_add = set()
        cluster_aux_delete = set()

        temp_added = set()
        temp_deleted = set()

        for r1 in cluster[0][0].true_set: 
            if r1 not in cluster[0][1].true_set:
                changed_relations.add(r1)
                temp_deleted.add(r1)

        for r1 in cluster[0][1].true_set: 
            if r1 not in cluster[0][0].true_set: 
                changed_relations.add(r1)
                temp_added.add(r1)

        for a_re in cluster[0][1].aux_true_set:
            if a_re not in cluster[0][0].aux_true_set:
                cluster_aux_add.add(copy.deepcopy(a_re))
            
        for a_re in cluster[0][0].aux_true_set:
            if a_re not in cluster[0][1].aux_true_set:
                cluster_aux_delete.add(copy.deepcopy(a_re))

        for a_re in cluster[0][0].aux_true_set:
            cluster_aux_true.add(copy.deepcopy(a_re))

        param_ids = {}
        param_mapping = { }
        relation_param_mapping = {}

        for relation in changed_relations: 
            if relation.parameter1_type not in param_ids:
                param_key = relation.parameter1_type
                if relation.parameter1_type in Config.CONST_TYPES:
                    param_key = "{}_{}".format(*relation.parameter1.split("_")[:Config.OBJ_ID_IND])
                param_ids[param_key] = 1

            if relation.parameter2_type not in param_ids: 
                param_key = relation.parameter2_type
                if relation.parameter2_type in Config.CONST_TYPES:
                    param_key = "{}_{}".format(*relation.parameter2.split("_")[:Config.OBJ_ID_IND])
                param_ids[param_key] = 1

            if relation.parameter1 not in param_mapping:
                param_key = relation.parameter1_type
                if relation.parameter1_type in Config.CONST_TYPES:
                    param_key = "{}_{}".format(*relation.parameter1.split("_")[:Config.OBJ_ID_IND])
                pid1 = param_ids[param_key]
                param_ids[param_key] += 1
                param_mapping[relation.parameter1] = param_key + "_p" + str(pid1)

            if relation.parameter2 not in param_mapping:
                param_key = relation.parameter2_type
                if relation.parameter2_type in Config.CONST_TYPES:
                    param_key = "{}_{}".format(*relation.parameter2.split("_")[:Config.OBJ_ID_IND])
                pid2 = param_ids[param_key]
                param_ids[param_key] += 1
                param_mapping[relation.parameter2] = param_key + "_p" + str(pid2)

            lr = relation.get_lifted_relation()
            if lr not in relation_param_mapping:
                relation_param_mapping[lr] = [ [param_mapping[relation.parameter1], param_mapping[relation.parameter2]] ]
            else: 
                relation_param_mapping[lr].append([param_mapping[relation.parameter1], param_mapping[relation.parameter2]])

        robot_id_set = set([])
        for relation in cluster[0][0].true_set:
            if relation in changed_relations:
                id_set = LiftedPDDLAction.get_robot_id_set(relation)
                if len(id_set) > 0:
                    robot_id_set = robot_id_set.union(id_set)
            
        for relation in temp_added:
            lr = relation.get_lifted_relation()
            pid1 = param_mapping[relation.parameter1]
            pid2 = param_mapping[relation.parameter2]
            cluster_e_add.add(ParameterizedLiftedRelation(pid1,pid2,lr))
        
        for relation in temp_deleted:
            lr = relation.get_lifted_relation()
            pid1 = param_mapping[relation.parameter1]
            pid2 = param_mapping[relation.parameter2]
            cluster_e_delete.add(ParameterizedLiftedRelation(pid1,pid2,lr))

        relations_union = cluster[0][0].true_set.union(cluster[0][0].false_set)
        for relation in temp_added:
            p1 = relation.parameter1
            p2 = relation.parameter2
            for p in relations_union:
                if p.parameter1 == p1 and p.parameter2 == p2 and relation != p:
                    pa = param_mapping[p.parameter1]
                    pb = param_mapping[p.parameter2]
                    lifted_relation = p.get_lifted_relation()
                    parameterized_relation = ParameterizedLiftedRelation(pa,pb,lifted_relation)
                    cluster_e_delete.add(parameterized_relation)
        
        for re in cluster_e_add:
            if re.pid1.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES:
                re.pid1 = "{}_{}_Const".format(*re.pid1.split("_")[:Config.OBJ_ID_IND])
            if re.pid2.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES:
                re.pid2 = "{}_{}_Const".format(*re.pid2.split("_")[:Config.OBJ_ID_IND])

        for re in cluster_e_delete:
            if re.pid1.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES:
                re.pid1 = "{}_{}_Const".format(*re.pid1.split("_")[:Config.OBJ_ID_IND])
            if re.pid2.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES:
                re.pid2 = "{}_{}_Const".format(*re.pid2.split("_")[:Config.OBJ_ID_IND])

        common_relations = set()
        additional_param_mappings = { }
        additional_param_ids = { }
        extra_robot_id_set = set()        
        param_objects = set([])

        additional_param_objects = {}
        sorted_true_set = list(cluster[0][0].true_set)
        sorted_true_set.sort()
        # sorted_true_set = sorted_true_set[::-1]

        for relation in sorted_true_set: 
            lr = relation.get_lifted_relation()
            if relation in changed_relations:
                if len(relation_param_mapping[lr]) == 1:
                    lr_index = 0
                else: 
                    lr_index = -1 
                    for lr_i in range(len(relation_param_mapping[lr])):
                        ps = relation_param_mapping[lr][lr_i]
                        if ps[0] == param_mapping[relation.parameter1] and ps[1] == param_mapping[relation.parameter2]: 
                            lr_index = lr_i 
                            break 
                    assert(lr_index != -1)

                pid1 = copy.deepcopy(relation_param_mapping[lr][lr_index][0])
                pid2 = copy.deepcopy(relation_param_mapping[lr][lr_index][1])
                parameterized_relation = ParameterizedLiftedRelation(pid1,pid2,lr)
                common_relations.add(parameterized_relation)
        
        for relation in sorted_true_set:
            lr = relation.get_lifted_relation()
            if relation not in changed_relations:
                if ( \
                        ( \
                            ( \
                                (relation.parameter1 in param_mapping and relation.parameter1_type not in Config.CONST_TYPES and (relation.parameter2_type not in Config.OBJECT_NAME)) or \
                                ((relation.parameter2 in param_mapping and relation.parameter2_type not in Config.CONST_TYPES) and (relation.parameter1_type not in Config.OBJECT_NAME)) \
                            ) and \
                            (relation.parameter1 != relation.parameter2) \
                        ) or \
                        ( \
                            ( \
                                (relation.parameter1_type in Config.ROBOT_TYPES and int(relation.parameter1.split("_")[Config.OBJ_ID_IND]) in robot_id_set) or \
                                (relation.parameter2_type in Config.ROBOT_TYPES and int(relation.parameter2.split("_")[Config.OBJ_ID_IND]) in robot_id_set) \
                            ) and \
                            (relation.cr != 0) \
                        ) \
                    ):

                    if relation.parameter1 not in param_mapping: 
                        param_key = relation.parameter1_type
                        if relation.parameter1_type in Config.CONST_TYPES:
                            param_key = "{}_{}".format(*relation.parameter1.split("_")[:Config.OBJ_ID_IND])

                        if param_key not in additional_param_objects:
                            additional_param_objects[param_key] = []

                        if relation.parameter1 not in additional_param_objects[param_key]:
                            additional_param_objects[param_key].append(relation.parameter1)

                    if relation.parameter2 not in param_mapping:
                        param_key = relation.parameter2_type
                        if relation.parameter2_type in Config.CONST_TYPES:
                            param_key = "{}_{}".format(*relation.parameter2.split("_")[:Config.OBJ_ID_IND])

                        if param_key not in additional_param_objects:
                            additional_param_objects[param_key] = []
                            
                        if relation.parameter2 not in additional_param_objects[param_key]:
                            additional_param_objects[param_key].append(relation.parameter2)
                    
                    id_set = LiftedPDDLAction.get_robot_id_set(relation)
                    if len(id_set.intersection(robot_id_set)) == 0:
                        if len(id_set) > 0:
                            extra_robot_id_set = extra_robot_id_set.union(id_set)

        if len(extra_robot_id_set) > 0:
            for relation in sorted_true_set:
                lr = relation.get_lifted_relation()
                if ( \
                        ( \
                            (\
                                (relation.parameter1_type in Config.ROBOT_TYPES and int(relation.parameter1.split("_")[Config.OBJ_ID_IND]) in extra_robot_id_set) or \
                                (relation.parameter2_type in Config.ROBOT_TYPES and int(relation.parameter2.split("_")[Config.OBJ_ID_IND]) in extra_robot_id_set)\
                            ) and \
                            (relation.cr != 0)\
                        ) or \
                        ( \
                            (relation.parameter1_type in Config.ROBOT_TYPES) and \
                            (relation.parameter2_type in Config.ROBOT_TYPES) and \
                            (int(relation.parameter1.split("_")[-1]) in extra_robot_id_set) and \
                            (int(relation.parameter1.split("_")[-1]) in extra_robot_id_set) \
                        ) \
                    ):
                    if relation.parameter1 not in param_mapping: 
                        param_key = relation.parameter1_type
                        if relation.parameter1_type in Config.CONST_TYPES:
                            param_key = "{}_{}".format(*relation.parameter1.split("_")[:Config.OBJ_ID_IND])

                        if param_key not in additional_param_objects:
                            additional_param_objects[param_key] = []

                        if relation.parameter1 not in additional_param_objects[param_key]:
                            additional_param_objects[param_key].append(relation.parameter1)

                    if relation.parameter2 not in param_mapping:
                        param_key = relation.parameter2_type
                        if relation.parameter2_type in Config.CONST_TYPES:
                            param_key = "{}_{}".format(*relation.parameter2.split("_")[:Config.OBJ_ID_IND])

                        if param_key not in additional_param_objects:
                            additional_param_objects[param_key] = []
                            
                        if relation.parameter2 not in additional_param_objects[param_key]:
                            additional_param_objects[param_key].append(relation.parameter2)
        
        param_objects = set(param_mapping.keys())
        param_objects = LiftedPDDLAction.get_param_objects(param_objects,additional_param_objects)
        for relation in cluster[0][0].true_set: 
            lr = relation.get_lifted_relation()
            if relation not in changed_relations:
                if set([relation.parameter1,relation.parameter2]).issubset(param_objects):
                    if relation.parameter1 in param_mapping: 
                        pid1 = param_mapping[relation.parameter1]
                    else:
                        if relation.parameter1 not in additional_param_mappings: 
                            param_key = relation.parameter1_type
                            if relation.parameter1_type in Config.CONST_TYPES:
                                param_key = "{}_{}".format(*relation.parameter1.split("_")[:Config.OBJ_ID_IND])
                            pid1 = additional_param_objects[param_key].index(relation.parameter1)+1
                            
                            additional_param_mappings[relation.parameter1] = param_key + "_" +  "extra" + "_p" + str(pid1)

                        pid1 = additional_param_mappings[relation.parameter1]

                        if len(robot_id_set) > 0:
                            if (relation.parameter1_type in Config.ROBOT_TYPES and int(relation.parameter1.split("_")[Config.OBJ_ID_IND]) in robot_id_set):
                                if "extra" in pid1:
                                    pid1 = pid1.split("_")[Config.OBJ_TYPE_IND] + "_" + pid1.split("_")[-1]

                    if relation.parameter2 in param_mapping:
                        pid2 = param_mapping[relation.parameter2]
                    else:
                        if relation.parameter2 not in additional_param_mappings: 
                            param_key = relation.parameter2_type
                            if relation.parameter2_type in Config.CONST_TYPES:
                                param_key = "{}_{}".format(*relation.parameter2.split("_")[:Config.OBJ_ID_IND])
                            pid2 = additional_param_objects[param_key].index(relation.parameter2)+1
                                                        
                            additional_param_mappings[relation.parameter2] = param_key + "_" +  "extra" + "_p" + str(pid2)
                        
                        pid2 = additional_param_mappings[relation.parameter2]

                        if len(robot_id_set) > 0:
                            if (relation.parameter2_type in Config.ROBOT_TYPES and int(relation.parameter2.split("_")[Config.OBJ_ID_IND]) in robot_id_set):
                                if "extra" in pid2:
                                    pid2 = pid2.split("_")[Config.OBJ_TYPE_IND] + "_" + pid2.split("_")[-1]

                    parameterized_relation = ParameterizedLiftedRelation(pid1,pid2,lr)
                    common_relations.add(parameterized_relation)

        if len(extra_robot_id_set) > 0:
            for relation in cluster[0][0].true_set:
                lr = relation.get_lifted_relation()
                if set([relation.parameter1,relation.parameter2]).issubset(param_objects):
                    if relation.parameter1 in param_mapping: 
                        pid1 = param_mapping[relation.parameter1]
                    else:
                        if relation.parameter1 not in additional_param_mappings:
                            param_key = relation.parameter1_type
                            if relation.parameter1_type in Config.CONST_TYPES:
                                param_key = "{}_{}".format(*relation.parameter1.split("_")[:Config.OBJ_ID_IND])
                            pid1 = additional_param_objects[param_key].index(relation.parameter1)+1
                            
                            additional_param_mappings[relation.parameter1] = param_key + "_" +  "extra" + "_p" + str(pid1)

                        pid1 = additional_param_mappings[relation.parameter1]

                    if relation.parameter2 in param_mapping:
                        pid2 = param_mapping[relation.parameter2]
                    else:
                        if relation.parameter2 not in additional_param_mappings: 
                            param_key = relation.parameter2_type
                            if relation.parameter2_type in Config.CONST_TYPES:
                                param_key = "{}_{}".format(*relation.parameter2.split("_")[:Config.OBJ_ID_IND])
                            pid2 = additional_param_objects[param_key].index(relation.parameter2)+1
                                                        
                            additional_param_mappings[relation.parameter2] = param_key + "_" +  "extra" + "_p" + str(pid2)
                        
                        pid2 = additional_param_mappings[relation.parameter2]

                    parameterized_relation = ParameterizedLiftedRelation(pid1,pid2,lr)
                    common_relations.add(parameterized_relation)
        
        for a in cluster_aux_add:
            if a.id > 2:
                if a.parameter in param_mapping.keys():
                    a.parameter = param_mapping[a.parameter]

        for a in cluster_aux_delete:
            if a.id > 2:
                if a.parameter in param_mapping.keys():
                    a.parameter = param_mapping[a.parameter]

        cluster_aux_true_to_remove = set()
        for a in cluster_aux_true:
            if a.id > 2:
                if a.parameter not in param_mapping.keys():
                    cluster_aux_true_to_remove.add(a)
                else:
                    a.parameter = param_mapping[a.parameter]

        for a in cluster_aux_true_to_remove:
            cluster_aux_true.remove(a)

        for transition in cluster[1:]:
            state1, state2 = transition
            local_changed = set()

            for r1 in state1.true_set: 
                if r1 not in state2.true_set:
                    local_changed.add(r1)
            for r1 in state2.true_set: 
                if r1 not in state1.true_set: 
                    local_changed.add(r1)

            local_additional_param_mappings = { }
            additional_param_ids = { }
            relation_set = set()
            local_param_mapping = { }
            robot_id_set = set([])
            extra_robot_id_set = set([])
            local_param_objects = set([])
                
            for relation in state1.true_set:
                if relation in local_changed:
                    id_set = LiftedPDDLAction.get_robot_id_set(relation)
                    if len(id_set) > 0:
                        robot_id_set = robot_id_set.union(id_set)  

            local_additional_param_objects = {}
            local_sorted_true_set = list(state1.true_set)
            local_sorted_true_set.sort()
            # local_sorted_true_set = local_sorted_true_set[::-1]

            local_changed = list(local_changed)
            local_changed.sort()

            lifted_local_changed_set = set() 
            for relation in local_changed: 
                if relation.cr != 0: 
                    lr = relation.get_lifted_relation()
                    if len(relation_param_mapping[lr]) == 1:
                        lr_index = 0
                    else: 
                        lr_index = -1
                        if relation.parameter1 in local_param_mapping and relation.parameter2 not in local_param_mapping: 
                            for lr_i in range(len(relation_param_mapping[lr])):
                                ps = relation_param_mapping[lr][lr_i]
                                if ps[0] == local_param_mapping[relation.parameter1]:
                                    lr_index = lr_i
                                    break

                        elif relation.parameter1 not in local_param_mapping and relation.parameter2 in local_param_mapping:
                            for lr_i in range(len(relation_param_mapping[lr])):
                                ps = relation_param_mapping[lr][lr_i]
                                if ps[1] == local_param_mapping[relation.parameter2]:
                                    lr_index = lr_i
                                    break

                        else:
                            for lr_i in range(len(relation_param_mapping[lr])):
                                ps = relation_param_mapping[lr][lr_i]
                                if ps[0] == param_mapping[relation.parameter1] and ps[1] == param_mapping[relation.parameter2]: 
                                    lr_index = lr_i 
                                    break 

                        assert(lr_index != -1) 
                    
                    pid1 = copy.deepcopy(relation_param_mapping[lr][lr_index][0])
                    pid2 = copy.deepcopy(relation_param_mapping[lr][lr_index][1])                                          
                    parameterized_relation = ParameterizedLiftedRelation(pid1,pid2,lr)
                    id_set = LiftedPDDLAction.get_robot_id_set(relation)
                    if len(id_set) > 0:
                        robot_id_set = robot_id_set.union(id_set)
                        
                    if relation in local_sorted_true_set:
                        relation_set.add(parameterized_relation)
                    lifted_local_changed_set.add(parameterized_relation)

                    if relation.parameter1 not in local_param_mapping:
                        local_param_mapping[relation.parameter1] = pid1
                    if relation.parameter2 not in local_param_mapping:
                        local_param_mapping[relation.parameter2] = pid2      

            for relation in local_changed: 
                if relation.cr == 0:
                    lr = relation.get_lifted_relation()
                    pid1 = copy.deepcopy(local_param_mapping[relation.parameter1])
                    pid2 = copy.deepcopy(local_param_mapping[relation.parameter2])                                          
                
                    parameterized_relation = ParameterizedLiftedRelation(pid1,pid2,lr)
                    id_set = LiftedPDDLAction.get_robot_id_set(relation)
                    if len(id_set) > 0:
                        robot_id_set = robot_id_set.union(id_set)
                    
                    if relation in local_sorted_true_set:
                        relation_set.add(parameterized_relation)
                    lifted_local_changed_set.add(parameterized_relation)

                if relation in local_changed:
                    lr = relation.get_lifted_relation()
                    if relation.cr == 0: 

                        if len(relation_param_mapping[lr]) == 1:
                            lr_index = 0
                        
                        else: 
                            pid1 = copy.deepcopy(local_param_mapping[relation.parameter1])
                            pid2 = copy.deepcopy(local_param_mapping[relation.parameter2])                                          
                        
                        parameterized_relation = ParameterizedLiftedRelation(pid1,pid2,lr)
                        id_set = LiftedPDDLAction.get_robot_id_set(relation)
                        if len(id_set) > 0:
                            robot_id_set = robot_id_set.union(id_set)

            for relation in local_sorted_true_set:
                if relation not in local_changed:
                    lr = relation.get_lifted_relation()
                    if ( \
                            ( \
                                ( \
                                    (relation.parameter1 in local_param_mapping and relation.parameter1_type not in Config.CONST_TYPES and (relation.parameter2_type not in Config.OBJECT_NAME)) or \
                                    (relation.parameter2 in local_param_mapping and relation.parameter2_type not in Config.CONST_TYPES and (relation.parameter1_type not in Config.OBJECT_NAME)) 
                                ) and \
                                (relation.parameter1 != relation.parameter2) \
                            ) or \
                            ( \
                                ( \
                                    (relation.parameter1_type in Config.ROBOT_TYPES and int(relation.parameter1.split("_")[Config.OBJ_ID_IND]) in robot_id_set) or \
                                    (relation.parameter2_type in Config.ROBOT_TYPES and int(relation.parameter2.split("_")[Config.OBJ_ID_IND]) in robot_id_set) \
                                ) and \
                                (relation.cr != 0) \
                            ) \
                        ):
                        if relation.parameter1 not in local_param_mapping:
                            param_key = relation.parameter1_type
                            if relation.parameter1_type in Config.CONST_TYPES:
                                param_key = "{}_{}".format(*relation.parameter1.split("_")[:Config.OBJ_ID_IND])

                            if param_key not in local_additional_param_objects:
                                local_additional_param_objects[param_key] = []
                            if relation.parameter1 not in local_additional_param_objects[param_key]:
                                local_additional_param_objects[param_key].append(relation.parameter1)

                        if relation.parameter2 not in local_param_mapping:
                            param_key = relation.parameter2_type
                            if relation.parameter2_type in Config.CONST_TYPES:
                                param_key = "{}_{}".format(*relation.parameter2.split("_")[:Config.OBJ_ID_IND])

                            if param_key not in local_additional_param_objects:
                                local_additional_param_objects[param_key] = []
                            if relation.parameter2 not in local_additional_param_objects[param_key]:
                                local_additional_param_objects[param_key].append(relation.parameter2)

                        id_set = LiftedPDDLAction.get_robot_id_set(relation)
                        if len(id_set.intersection(robot_id_set)) == 0:
                            if len(id_set) > 0:
                                extra_robot_id_set = extra_robot_id_set.union(id_set)

            if len(extra_robot_id_set) > 0:
                for relation in local_sorted_true_set:
                    lr = relation.get_lifted_relation()
                    if ( \
                            ( \
                                ( \
                                    (relation.parameter1_type in Config.ROBOT_TYPES and int(relation.parameter1.split("_")[Config.OBJ_ID_IND]) in extra_robot_id_set) or \
                                    (relation.parameter2_type in Config.ROBOT_TYPES and int(relation.parameter2.split("_")[Config.OBJ_ID_IND]) in extra_robot_id_set) \
                                ) and \
                                (relation.cr != 0) \
                            ) or \
                            ( \
                                (relation.parameter1_type in Config.ROBOT_TYPES) and \
                                (relation.parameter2_type in Config.ROBOT_TYPES) and \
                                (int(relation.parameter1.split("_")[-1]) in extra_robot_id_set) and \
                                (int(relation.parameter1.split("_")[-1]) in extra_robot_id_set) \
                            ) \
                        ):
                        if relation.parameter1 not in local_param_mapping:
                            param_key = relation.parameter1_type
                            if relation.parameter1_type in Config.CONST_TYPES:
                                param_key = "{}_{}".format(*relation.parameter1.split("_")[:Config.OBJ_ID_IND])

                            if param_key not in local_additional_param_objects:
                                local_additional_param_objects[param_key] = []
                            if relation.parameter1 not in local_additional_param_objects[param_key]:
                                local_additional_param_objects[param_key].append(relation.parameter1)

                        if relation.parameter2 not in local_param_mapping:
                            param_key = relation.parameter2_type
                            if relation.parameter2_type in Config.CONST_TYPES:
                                param_key = "{}_{}".format(*relation.parameter2.split("_")[:Config.OBJ_ID_IND])

                            if param_key not in local_additional_param_objects:
                                local_additional_param_objects[param_key] = []
                            if relation.parameter2 not in local_additional_param_objects[param_key]:
                                local_additional_param_objects[param_key].append(relation.parameter2)

            local_param_objects = set(local_param_mapping.keys())            
            local_param_objects = LiftedPDDLAction.get_param_objects(local_param_objects,local_additional_param_objects)
            for relation in state1.true_set:
                if relation not in local_changed:
                    lr = relation.get_lifted_relation()
                    if set([relation.parameter1,relation.parameter2]).issubset(local_param_objects):
                        if relation.parameter1 in local_param_mapping:
                            pid1 = local_param_mapping[relation.parameter1]
                        else:
                            if relation.parameter1 not in local_additional_param_mappings:
                                param_key = relation.parameter1_type
                                if relation.parameter1_type in Config.CONST_TYPES:
                                    param_key = "{}_{}".format(*relation.parameter1.split("_")[:Config.OBJ_ID_IND])

                                pid1 = local_additional_param_objects[param_key].index(relation.parameter1) + 1

                                local_additional_param_mappings[relation.parameter1] = param_key + "_" +  "extra" + "_p" + str(pid1)

                            pid1 = local_additional_param_mappings[relation.parameter1]

                            if len(robot_id_set) > 0:
                                if (relation.parameter1_type in Config.ROBOT_TYPES and int(relation.parameter1.split("_")[Config.OBJ_ID_IND]) in robot_id_set):
                                    if "extra" in pid1:
                                        pid1 = pid1.split("_")[Config.OBJ_TYPE_IND] + "_" + pid1.split("_")[-1]

                        if relation.parameter2 in local_param_mapping:
                            pid2 = local_param_mapping[relation.parameter2]
                        else:
                            if relation.parameter2 not in local_additional_param_mappings:
                                param_key = relation.parameter2_type
                                if relation.parameter2_type in Config.CONST_TYPES:
                                    param_key = "{}_{}".format(*relation.parameter2.split("_")[:Config.OBJ_ID_IND])

                                pid2 = local_additional_param_objects[param_key].index(relation.parameter2) + 1

                                local_additional_param_mappings[relation.parameter2] = param_key + "_" +  "extra" + "_p" + str(pid2)

                            pid2 = local_additional_param_mappings[relation.parameter2]
                            
                            if len(robot_id_set) > 0:
                                if (relation.parameter2_type in Config.ROBOT_TYPES and int(relation.parameter2.split("_")[Config.OBJ_ID_IND]) in robot_id_set):
                                    if "extra" in pid2:
                                        pid2 = pid2.split("_")[Config.OBJ_TYPE_IND] + "_" + pid2.split("_")[-1]

                        parameterized_relation = ParameterizedLiftedRelation(pid1,pid2,lr)
                        relation_set.add(parameterized_relation)

            if len(extra_robot_id_set) > 0:
                for relation in state1.true_set:
                    lr = relation.get_lifted_relation()
                    if set([relation.parameter1,relation.parameter2]).issubset(local_param_objects):
                        if relation.parameter1 in local_param_mapping: 
                            pid1 = local_param_mapping[relation.parameter1]
                        else:
                            if relation.parameter1 not in local_additional_param_mappings:
                                param_key = relation.parameter1_type
                                if relation.parameter1_type in Config.CONST_TYPES:
                                    param_key = "{}_{}".format(*relation.parameter1.split("_")[:Config.OBJ_ID_IND])

                                pid1 = local_additional_param_objects[param_key].index(relation.parameter1) + 1

                                local_additional_param_mappings[relation.parameter1] = param_key + "_" +  "extra" + "_p" + str(pid1)

                            pid1 = local_additional_param_mappings[relation.parameter1]

                        if relation.parameter2 in local_param_mapping:
                            pid2 = local_param_mapping[relation.parameter2]
                        else:
                            if relation.parameter2 not in local_additional_param_mappings: 
                                param_key = relation.parameter2_type
                                if relation.parameter2_type in Config.CONST_TYPES:
                                    param_key = "{}_{}".format(*relation.parameter2.split("_")[:Config.OBJ_ID_IND])

                                pid2 = local_additional_param_objects[param_key].index(relation.parameter2) + 1

                                local_additional_param_mappings[relation.parameter2] = param_key + "_" +  "extra" + "_p" + str(pid2)

                            pid2 = local_additional_param_mappings[relation.parameter2]

                        parameterized_relation = ParameterizedLiftedRelation(pid1,pid2,lr)
                        relation_set.add(parameterized_relation)

            new_set = set()
            for relation in relation_set: 
                for relation2 in common_relations: 
                    if relation == relation2:
                        new_set.add(relation)
                        break
            common_relations = copy.deepcopy(new_set)

            new_aux_true = set()
            for a_re in cluster_aux_true:
                for a2 in state1.aux_true_set:
                    a = copy.deepcopy(a2)
                    if a.id > 2:
                        if a.parameter in local_param_mapping.keys():
                            a.parameter = local_param_mapping[a.parameter]

                    if a_re == a:
                        new_aux_true.add(a_re)
                        break
                    
            cluster_aux_true = copy.deepcopy(new_aux_true)

        common_relations_to_remove = set()
        for re in common_relations:
            extra_flag = (("extra" in re.pid1 and re.pid1.split("_")[Config.OBJ_TYPE_IND] not in Config.ROBOT_TYPES) or ("extra" in re.pid2 and re.pid2.split("_")[Config.OBJ_TYPE_IND] not in Config.ROBOT_TYPES))
            if extra_flag and not (re.parent_relation.parameter1_type in Config.ROBOT_TYPES and re.parent_relation.parameter2_type in Config.ROBOT_TYPES):
                pid1 = re.pid1
                if re.pid1.split("_")[Config.OBJ_TYPE_IND] in Config.ROBOT_TYPES:
                    pid1 = re.pid1.split("_")[Config.OBJ_TYPE_IND] + "_extra_" + re.pid1.split("_")[Config.OBJ_ID_IND]
                if pid1 in additional_param_mappings.values(): 
                    for o in additional_param_mappings.keys():
                        if additional_param_mappings[o] == pid1:
                            o1 = o 
                            break
                else:
                    for o in param_mapping.keys():
                        if param_mapping[o] == re.pid1:
                            o1 = o 
                            break
                
                pid2 = re.pid2
                if re.pid2.split("_")[Config.OBJ_TYPE_IND] in Config.ROBOT_TYPES:
                    pid2 = re.pid2.split("_")[Config.OBJ_TYPE_IND] + "_extra_" + re.pid2.split("_")[Config.OBJ_ID_IND]
                if re.pid2 in additional_param_mappings.values(): 
                    for o in additional_param_mappings.keys():
                        if additional_param_mappings[o] == pid2:
                            o2 = o 
                            break
                else:
                    for o in param_mapping.keys():
                        if param_mapping[o] == re.pid2:
                            o2 = o 
                            break
                
                l1 = Link(o1,re.parent_relation.parameter1_type)
                l2 = Link(o2,re.parent_relation.parameter2_type)
                gr = re.parent_relation.get_grounded_relation(l1,l2)

                if re.parent_relation.cr == 0 and gr not in added_relations[cluster[0][0]]:
                    common_relations_to_remove.add(re)

        for re in common_relations_to_remove:
            common_relations.remove(re)

        for re in common_relations:
            if re.pid1.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES:
                re.pid1 = "{}_{}_Const".format(*re.pid1.split("_")[:Config.OBJ_ID_IND])
            if re.pid2.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES:
                re.pid2 = "{}_{}_Const".format(*re.pid2.split("_")[:Config.OBJ_ID_IND])

        lifted_changed_relations = set([a.get_lifted_relation() for a in changed_relations])
        parameterized_changed_relations = set([a for a in common_relations.union(cluster_e_add) if a.parent_relation in lifted_changed_relations])

        aux_to_remove = set()
        for aux in cluster_aux_true:
            if aux.parameter.split("_")[Config.OBJ_TYPE_IND] not in Config.ROBOT_TYPES:
                flag = 0
                for re in changed_relations:
                    if aux.parameter1_type == re.parameter1_type and aux.parameter2_type == re.parameter2_type:
                        if not (aux.cr != re.cr and not (aux.parameter1_type in Config.ROBOT_TYPES or aux.parameter2_type in Config.ROBOT_TYPES)):
                            flag = 1
                            break
                if flag == 0:
                    aux_to_remove.add(aux)
                    continue
                
                flag = 0
                for re in parameterized_changed_relations:
                    if ((re.pid1 == aux.parameter and re.parent_relation.cr == aux.cr) or (aux.parameter1_type in Config.ROBOT_TYPES or aux.parameter2_type in Config.ROBOT_TYPES)):
                        flag = 1
                        break
                
                if flag == 0:
                    aux_to_remove.add(aux)

        new_cluster_aux_true = set()
        for aux in cluster_aux_true:
            flag = 0
            for ax in aux_to_remove:
                if aux == ax:
                    flag = 1
                    break
            if flag == 0:
                new_cluster_aux_true.add(aux)

        cluster_aux_true = copy.deepcopy(new_cluster_aux_true)

        # aux_to_add = set()
        # for re in common_relations:
        #     for ax in all_aux:
        #         if re.parameter1_type == ax.parameter1_type and re.parameter2_type == ax.parameter2_type and re.cr != ax.cr:
        #             aux_to_add.add(re)
    
        param_set = set()
        for relation in common_relations: 
            if relation.parent_relation.parameter1_type not in Config.CONST_TYPES:
                param1 = Parameter(relation.pid1,relation.parent_relation.parameter1_type)
                param_set.add(param1)

            if relation.parent_relation.parameter2_type not in Config.CONST_TYPES:
                param2 = Parameter(relation.pid2,relation.parent_relation.parameter2_type)
                param_set.add(param2)

        for ax in cluster_aux_true:
            if ax.id > 2:
                if ax.id == 3 and ax.parameter1_type not in Config.CONST_TYPES:
                    param = Parameter(ax.parameter,ax.parameter1_type)
                elif ax.id == 4 and ax.parameter2_type not in Config.CONST_TYPES:
                    param = Parameter(ax.parameter,ax.parameter2_type)
                
                param_set.add(param)
        
        for relation in cluster_e_add:
            if relation.parent_relation.parameter1_type not in Config.CONST_TYPES:
                param1 = Parameter(relation.pid1,relation.parent_relation.parameter1_type)
                param_set.add(param1)

            if relation.parent_relation.parameter2_type not in Config.CONST_TYPES:
                param2 = Parameter(relation.pid2,relation.parent_relation.parameter2_type)
                param_set.add(param2)

        for relation in cluster_e_delete:
            if relation.parent_relation.parameter1_type not in Config.CONST_TYPES:
                param1 = Parameter(relation.pid1,relation.parent_relation.parameter1_type)
                param_set.add(param1)

            if relation.parent_relation.parameter2_type not in Config.CONST_TYPES:
                param2 = Parameter(relation.pid2,relation.parent_relation.parameter2_type)
                param_set.add(param2)

        preconditions = LiftedPDDLPrecondition(true_set=common_relations, false_set=set(),true_aux_set=cluster_aux_true)
        effects = LiftedPDDLEffect(cluster_e_add,cluster_e_delete,cluster_aux_add, cluster_aux_delete)
        LiftedPDDLAction.action_id += 1

        return LiftedPDDLAction(LiftedPDDLAction.action_id, list(param_set), preconditions, effects)
    
    def __str__(self):
        s = "(:action a{} \n".format(self.action_id)
        param_string = ""
        for param in self.parameters:
            param_string += " ?{} - {} ".format(param.pid,param.type)
        s += ":parameters ({})\n".format(param_string)
        precondition_string = ""
        for i,param in enumerate(self.parameters):
            for j,param2 in enumerate(self.parameters):
                if j>i and param.type == param2.type:
                    precondition_string += "\t(not (= ?{} ?{}))\n".format(param.pid,param2.pid)
        
        precondition_string+=str(self.preconditions)

        states_to_neglect_str = ""
        for state in self.states_to_neglect:
            state_string = ""
            for i,prop in enumerate(state.true_set): 
                state_string += str(prop)
                state_string += " "
                if i > 0 and i % 4 == 0: 
                    state_string += "\n"
            states_to_neglect_str += "(not (and {}))\n".format(state_string)
        
        precondition_string += states_to_neglect_str

        s += ":precondition (and \n{}) \n".format(precondition_string)

        effect_string = str(self.effects)

        s += ":effect (and \n {} ) \n".format(effect_string)
        s+= ")\n"

        return s
    
    def get_grounded_action(self, grounding,lifted_action_id, object_list): 
        grounded_precondition = self.preconditions.get_grounded_precondition(grounding,object_list)
        grounded_effect = self.effects.get_grounded_effect(grounding,object_list)
        return GroundedPDDLAction(grounded_precondition,grounded_effect,lifted_action_id)

    def __eq__(self,o):
        if (self.preconditions != o.preconditions) or (self.effects != o.effects): #or (self.parameters != o.parameters):
            return False
        return True
    
    def __lt__(self,o):
        return self.__str__() < o.__str__()
    
    def __hash__(self):
        return hash(self.__str__().split(":parameters")[-1])

class GroundedPDDLAction(object):
    def __init__(self,precondition,effect,lifted_action_id):
        self.precondition = precondition
        self.effect = effect 
        self.changed_relations = self.get_changed_relations()
        self.lifted_action_id = lifted_action_id
        self.sampling_region = None
        self.mapping_gen_dict = {}

        self.try_next_region = True
        self.use_next_relation = True

    def check_applicability(self,pddl_state):
        return self.precondition.check_in_state(pddl_state)

    def apply(self,pddl_state):
        return self.effect.apply(pddl_state)

    def get_changed_relations(self):
        changed_relations = []
        for rel in list(self.precondition.true_set):
            if rel in self.effect.delete_set:
                changed_relations.append(rel)

        for rel in list(self.precondition.false_set):
            if rel in self.effect.add_set:
                changed_relations.append(rel)

        return changed_relations

    def update_relations_mapping_generator_dict(self,relation_options):
        initialized_generator_keys = self.mapping_gen_dict.keys()
        for re in initialized_generator_keys:
            if re not in relation_options:
                del self.mapping_gen_dict[re]

        for re in relation_options:
            if re not in self.mapping_gen_dict.keys():
                self.mapping_gen_dict[re] = re.get_mapping_generator()
        
    def compute_refinement(self,env_state,sim_object,previous_instances,action_info,compute_motion_plan=True): 
        traj = None
        flag = 0

        sim_object.set_env_state(env_state)

        current_grabbed_flag, grabbed_relation = self.get_current_flags(env_state)
        
        grabbed_bool = None
        used_relations = set([])
        relation_selected = False
        while not relation_selected:
            relation_options = self.effect.add_set.difference(used_relations)
            if len(relation_options) == 0:
                break
            
            self.update_relations_mapping_generator_dict(relation_options)

            relation_to_use = self.get_relation_to_use(relation_options,env_state)
            valid_sample_found = False

            while not self.use_next_relation: 
                sim_object.set_env_state(env_state)
                self.set_sampling_region(relation_to_use) # sets the self.try_next_region flag internally
                
                while self.sampling_region is not None: 
                    try: 
                        next_sample,lifted_region_used,object_with_transform,rob, static_list, eef_transform = relation_to_use.get_next_sample()
                    except StopIteration: # region out of samples. move to different region 
                        valid_sample_found = False
                        if not self.set_next_mapping(relation_to_use,env_state,sim_object,action_info):
                            continue                        
                        self.try_next_region = True
                        break
                    
                    if len(next_sample) == Config.SENSOR_COUNT:
                        continue

                    static_object = static_list[0]
                    grabbed_bool = self.get_grabbed_bool(grabbed_relation,next_sample,relation_to_use,current_grabbed_flag)

                    # check if this sample is valid or not
                    satisfactory_flag,new_env_state,failed_relation = self.satisfactory_sample(object_with_transform,env_state,grabbed_bool,sim_object,rob,next_sample)

                    if satisfactory_flag: 
                        valid_sample_found = True 
                        break 
                
                if valid_sample_found or self.sampling_region is None: 
                    break 

            if valid_sample_found: 
                if self.motion_plan_flag_check(env_state,relation_to_use,current_grabbed_flag):
                    traj, flag = self.get_traj_from_sample(next_sample,sim_object,rob,compute_motion_plan)
            else: 
                used_relations.add(relation_to_use)

            if traj is not None or grabbed_bool is not None:
                relation_selected = True
                break
            
        if len(relation_options) == 0:
            return None, None, (None, None, None), None, None, None
        
        lifted_region_used_sampling_count = 1
        if relation_to_use in previous_instances.keys():
            for (prev_region,region_sampling_count) in previous_instances[relation_to_use]:
                if prev_region == tuple(lifted_region_used):
                    region_sampling_count+=1
                    lifted_region_used_sampling_count = region_sampling_count
                    break
                    
        prev_instance_tuple = (relation_to_use,lifted_region_used,lifted_region_used_sampling_count)
        
        if (traj is not None or flag == 1) and grabbed_bool is not None:
            return None, None, prev_instance_tuple, rob, static_object, eef_transform
        
        else:
            traj_list = traj
            grab_list = grabbed_bool

        return traj_list,grab_list,prev_instance_tuple, rob, static_object, eef_transform
    
    def get_grabbed_in_state(self,env_state):
        grabbed = False
        for r in range(1,env_state.num_robots+1):
            grabbed = (grabbed or getattr(env_state,"grabbed_flag_{}".format(r)))

        return grabbed

    def get_current_flags(self,env_state):
        grabbed_relation = None
        current_grabbed_flag = 0

        grabbed = self.get_grabbed_in_state(env_state)
        if grabbed:
            for re in self.changed_relations:
                grabbed_object_flag = False
                for r in range(1,env_state.num_robots+1):
                    if (re.parameter1 == getattr(env_state,"grabbed_object_{}".format(r)) or re.parameter2 == getattr(env_state,"grabbed_object_{}".format(r))):
                        grabbed_object_flag = True
                        
                if (Config.GRIPPER_NAME in re.parameter1 or Config.GRIPPER_NAME in re.parameter2) and grabbed_object_flag:
                    grabbed_relation = re
                    if (re.parameter1_type == Config.GRIPPER_NAME):
                        id = re.parameter1.split("_")[Config.OBJ_ID_IND]                        
                        if grabbed:
                            if re.parameter2 == getattr(env_state,"grabbed_object_{}".format(id)):
                                current_grabbed_flag = 1
                            else:
                                current_grabbed_flag = 2
                        else:
                            current_grabbed_flag = 0

                    elif (re.parameter2_type == Config.GRIPPER_NAME):
                        id = re.parameter2.split("_")[Config.OBJ_ID_IND]                        
                        if grabbed:
                            if re.parameter1 == getattr(env_state,"grabbed_object_{}".format(id)):
                                current_grabbed_flag = 1
                            else:
                                current_grabbed_flag = 2
                        else:
                            current_grabbed_flag = 0                  
                    break

        return current_grabbed_flag, grabbed_relation

    def get_relation_to_use(self,relation_options,env_state):
        relation_to_use = None
        grabbed = self.get_grabbed_in_state(env_state)
        
        for re in relation_options:
            if re.parameter1.split("_")[Config.OBJ_TYPE_IND] in Config.IMMOVABLE_OBJECTS or re.parameter2.split("_")[Config.OBJ_TYPE_IND] in Config.IMMOVABLE_OBJECTS: 
                relation_to_use = re
                break
            
            if grabbed:
                grabbed_object_flag = False
                for r in range(1,env_state.num_robots+1):
                    if (re.parameter1 == getattr(env_state,"grabbed_object_{}".format(r)) or re.parameter2 == getattr(env_state,"grabbed_object_{}".format(r))):
                        grabbed_object_flag = True

                if (Config.GRIPPER_NAME in re.parameter1 or Config.GRIPPER_NAME in re.parameter2): #and grabbed_object_flag:
                    relation_to_use = re
                    break
                if grabbed_object_flag:
                    relation_to_use = re

            else:
                if re.parameter1.split("_")[Config.OBJ_TYPE_IND] in Config.ROBOT_TYPES.keys() or re.parameter2.split("_")[Config.OBJ_TYPE_IND] in Config.ROBOT_TYPES.keys():
                    relation_to_use = re
                    break

        if relation_to_use is None:
            relation_to_use = np.random.choice(list(relation_options))

        self.use_next_relation = False

        return relation_to_use

    def set_next_mapping(self,relation_to_use,env_state,sim_object,action_info):
        reset_flag = False
        try:
            mapping_to_use = self.mapping_gen_dict[relation_to_use].next()
            relation_to_use.set_current_mapping_pair(mapping_to_use)
        except StopIteration:
            reset_flag = True
            relation_to_use.reset_current_mapping()
            self.mapping_gen_dict[relation_to_use] = relation_to_use.get_mapping_generator()
        else:
            relation_to_use.init_sample_generator(env_state, sim_object, self.sampling_region, action_info)
        
        return reset_flag

    def check_reset_sampling_flag(self,relation_to_use):
        if self.sampling_region is None:
            return True
        
        return self.sampling_region >= len(relation_to_use.useful_components_list)

    def set_sampling_region(self,relation_to_use):
        reset_sampling_flag = self.check_reset_sampling_flag(relation_to_use)
        if ((not self.use_next_relation) and self.try_next_region) or reset_sampling_flag: 
            try: 
                self.sampling_region = relation_to_use.sample_region()
                self.try_next_region = False
            except StopIteration: # relation out of regions. move to different relation
                self.sampling_region = None
                self.use_next_relation = True

    def get_grabbed_bool(self,grabbed_relation,next_sample,relation_to_use,current_grabbed_flag):
        grabbed_bool = None             
        next_grabbed_flag = 0

        if grabbed_relation is None:
            next_grabbed_flag = next_sample[Config.GRAB_INDEX]
        else:
            if len(relation_to_use.region)==1:
                sampled_lifted_region = relation_to_use.region[0]
            else:
                index = np.random.randint(len(relation_to_use.region))
                sampled_lifted_region = relation_to_use.region[index]

            next_grabbed_flag = sampled_lifted_region[Config.GRAB_INDEX]         
            
        if (not (bool(current_grabbed_flag and next_grabbed_flag))) and (2 not in [current_grabbed_flag,next_grabbed_flag]):
            grabbed_action = current_grabbed_flag - next_grabbed_flag

            if grabbed_action > 0:
                grabbed_bool=False
            elif grabbed_action < 0:
                grabbed_bool=True
            else:
                grabbed_bool = None                  
        else:
            grabbed_bool = None

        return grabbed_bool

    def get_traj_from_sample(self,next_sample,sim_object,rob,compute_motion_plan):
        traj = None
        flag = 0
        counter = 0
        while traj is None:
            if compute_motion_plan:
                index = len(next_sample) - 1 
                traj = sim_object.compute_motion_plan(goal=next_sample[:index],robot=rob.robot)
            else:
                traj = next_sample

            counter+=1
            if counter >= Config.MP_MAX_COUNT and traj is None:
                flag = 1
                break
        
        return traj, flag

    def satisfactory_sample(self,object_with_transform,env_state,grabbed_bool,sim_object,rob,robot_dof_vals):        
        grabbed_obj_name = None
        if grabbed_bool is not None:
            if grabbed_bool:
                for relation in self.effect.add_set:
                    if Config.GRIPPER_NAME in relation.parameter1:
                        if relation.parameter2.split("_")[Config.OBJ_TYPE_IND] not in Config.IMMOVABLE_OBJECTS and relation.parameter2.split("_")[Config.OBJ_TYPE_IND] != Config.GRIPPER_NAME:
                            grabbed_obj_name = relation.parameter2
                            id = relation.parameter1.split("_")[Config.OBJ_ID_IND]
            else:
                grabbed_obj_name = getattr(env_state,"grabbed_object_{}".format(rob.id))
            
            traj = grabbed_bool
            
        else:
            traj = robot_dof_vals[:-Config.SENSOR_COUNT]
        
        new_env_state = sim_object.execute_refinement(traj=traj,robot=rob,obj_name=grabbed_obj_name)

        for relation in self.effect.add_set:
            if relation.evaluate_in_ll_state(new_env_state) is not True:
                sim_object.set_env_state(env_state)
                return False,new_env_state,relation
            # else:
            #     print("debug")
        
        sim_object.set_env_state(env_state)
        return True,new_env_state,None

    def motion_plan_flag_check(self,env_state,relation_to_use,current_grabbed_flag):
        new_region_list = []
        new_relation = copy.deepcopy(relation_to_use)
        for re in new_relation.region:
            new_region = re[:-Config.SENSOR_COUNT]
            new_region.extend(Config.SENSOR_COUNT*[0])
            new_region[Config.GRAB_INDEX] = current_grabbed_flag
            new_region_list.append(new_region)

        new_relation.region = new_region_list

        return not (new_relation.evaluate_in_ll_state(env_state))

    def check_collisions_in_env_state(self,sim_object,env_state):
        current_env_state = sim_object.get_current_state()
        sim_object.set_env_state(env_state)
        collision_flag = sim_object.collision_check(env_state.object_dict.keys())

        sim_object.set_env_state(current_env_state)
        return collision_flag
        
    def __str__(self):
        add_string = "adding -> "
        if len(self.effect.add_set) > 0:
            for rel in self.effect.add_set:
                add_string += rel.__str__()
                add_string += ", "
        else:
            add_string += "NOTHING"
        
        delete_string = "|| deleting -> "
        if len(self.effect.delete_set) > 0:
            for rel in self.effect.delete_set:
                delete_string += rel.__str__()
                delete_string += ", "
        else:
            delete_string += "NOTHING"
        
        id_string = "lifted_action_id:{};".format(self.lifted_action_id)
        return id_string+add_string+delete_string
