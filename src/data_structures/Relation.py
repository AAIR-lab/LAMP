import functools
from Config import Config
import numpy as np
import copy
import useful_functions
from itertools import product
from src.GMM.MVGFit import MultiVariateGaussian

Object = Config.get_simulator_module("Object").Object

@functools.total_ordering
class Relation(object):    
    def __init__(self, parameter1_type, parameter2_type, cr, region, discretizer, mvg=None, sampling_region_list=[]): 
        self.parameter1_type = parameter1_type
        self.parameter2_type = parameter2_type
        self.cr = cr
        self.region = region
        self.sampling_region_list = sampling_region_list
        self.discretizer = discretizer
        self.mvg = mvg
        self.get_sampling_region_list()
        self.get_mvg()

    def get_sampling_region_list(self):
        if self.sampling_region_list == []:
            self.sampling_region_list = copy.deepcopy(self.region)

    def get_mvg(self):
        if self.cr != 0 and self.mvg is None:  
            samples = self.get_fit_samples()
            self.mvg = MultiVariateGaussian.fit(samples,n_components = len(self.region))

    def get_fit_samples(self): 
        samples = [] 
        for region in self.sampling_region_list:
            for _ in range(1000): 
                if len(self.sampling_region_list) > 1:
                    ind = np.random.choice(len(self.sampling_region_list))
                else:
                    ind = 0
                region = self.sampling_region_list[ind]

                region_to_use = region[:6]
                samples.append(self.discretizer.convert_sample(region_to_use, is_relative = True))
        
        return samples      

    def get_grounded_relation(self,parameter1, parameter2, object_list=[]): 
        return GroundedRelation(parameter1,parameter2,self.cr,self.region,self.discretizer, object_list,mvg=self.mvg)
    
    def __str__(self): 
        return "({}_{}_{} ?x - {} ?y - {})".format(self.parameter1_type,self.parameter2_type, str(self.cr), self.parameter1_type, self.parameter2_type )

    def __eq__(self,o): 
        if self.parameter1_type == o.parameter1_type and self.parameter2_type == o.parameter2_type and self.region == o.region:
            if self.cr == 0 or o.cr == 0:
                return self.cr == o.cr
            return True
        elif self.parameter1_type == o.parameter2_type and self.parameter2_type == o.parameter1_type and self.region == o.region:
            if self.cr == 0 or o.cr == 0:
                return self.cr == o.cr
            return True
        else: 
            return False

    def is_equivalent(self,o): 
        if set(self.region).issubset(set(o.region)) or set(o.region).issubset(set(self.region)):
            return True
        else: 
            return False
        
    def __lt__(self,o): 
        return self.__str__() < o.__str__()
        
    def __hash__(self):
        # return hash(self.__str__())
        if self.cr != 0:
            return hash("({}_{}_{} ?x - {} ?y - {})".format(self.parameter1_type,self.parameter2_type, str(self.region), self.parameter1_type, self.parameter2_type ))
        else:
            return hash("({}_{}_{} ?x - {} ?y - {})".format(self.parameter1_type,self.parameter2_type, str(len(self.region)), self.parameter1_type, self.parameter2_type ))

    def __deepcopy__(self,memodict={}):
        region_to_copy = copy.deepcopy(self.region)
        sampling_region_to_copy = copy.deepcopy(self.sampling_region_list)
        new_relation = Relation(self.parameter1_type, self.parameter2_type, self.cr, region_to_copy, self.discretizer, self.mvg, sampling_region_to_copy)
        return new_relation
            
@functools.total_ordering
class GroundedRelation(Relation): 
    def __init__(self,parameter1,parameter2,cr,region,discretizer,object_list,mvg=None):
        super(GroundedRelation,self).__init__(parameter1.type, parameter2.type,cr,region,discretizer,mvg=mvg)
        self.p1 = parameter1
        self.p2 = parameter2
        self.parameter1 = parameter1.name
        self.parameter2 = parameter2.name
        self.relational = True if parameter1.type != "world" else False
        self.region_generator = None 
        self.sample_fn = None 
        self.env_state = None 
        self.sim_object = None 
        self.region_to_use  = None
        self.mapping = dict.fromkeys([self.parameter1,self.parameter2], [])
        self.current_mapping_pair = None

        if len(object_list) > 0:
            self.get_mapping(self.filter_mapping_options(object_list))

    def set_current_mapping_pair(self,mapping_pair):
        self.current_mapping_pair = mapping_pair

    def reset_current_mapping(self,):
        self.current_mapping_pair = None

    def get_mapping_generator(self):
        return self.mapping_generator()

    def mapping_generator(self):
        possible_pairs = product(self.mapping[self.parameter1],self.mapping[self.parameter2])
        for pair in possible_pairs:
            yield pair

        raise StopIteration
    
    def get_mapping(self,object_list):
        if self.parameter1.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES:
            self.mapping[self.parameter1] = [obj for obj in object_list if useful_functions.get_obj_type(obj) == useful_functions.get_obj_type(self.parameter1)]
        else:
            self.mapping[self.parameter1] = [self.parameter1]
        
        if self.parameter2.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES:
            self.mapping[self.parameter2] = [obj for obj in object_list if useful_functions.get_obj_type(obj) == useful_functions.get_obj_type(self.parameter2)]
        else:
            self.mapping[self.parameter2] = [self.parameter2]

    def filter_mapping_options(self,object_list):
        new_object_list = object_list
        if (self.parameter1.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES) and \
           (self.parameter2_type in Config.OBJECT_NAME):
            new_object_list = [o for o in object_list \
                               if o.split("_")[Config.OBJ_ID_IND] == self.parameter2.split("_")[Config.OBJ_ID_IND] and \
                                    self.parameter2.split("_")[Config.OBJ_TYPE_IND] in o.split("_")[Config.CONST_TYPE_IND]]

        elif (self.parameter2.split("_")[Config.OBJ_TYPE_IND] in Config.CONST_TYPES) and \
           (self.parameter1_type in Config.OBJECT_NAME):
            new_object_list = [o for o in object_list \
                               if o.split("_")[Config.OBJ_ID_IND] == self.parameter1.split("_")[Config.OBJ_ID_IND] and \
                                    self.parameter1.split("_")[Config.OBJ_TYPE_IND] in o.split("_")[Config.CONST_TYPE_IND]]

        return new_object_list

    def __deepcopy__(self,memodict={}): 
        region_to_copy = copy.deepcopy(self.region)

        object_list = []
        for vals in self.mapping.values():
            object_list.extend(vals)

        new_relation = GroundedRelation(self.p1, self.p2, self.cr,region_to_copy,self.discretizer,object_list,mvg=self.mvg)
        return new_relation

    def __str__(self):
        parameter1_str = self.parameter1
        if self.parameter1_type in Config.CONST_TYPES:
            parameter1_str = useful_functions.update_to_const(self.parameter1)
        
        parameter2_str = self.parameter2
        if self.parameter2_type in Config.CONST_TYPES:
            parameter2_str = useful_functions.update_to_const(self.parameter2)
    
        return "({}_{}_{} {} {})".format(self.parameter1_type, self.parameter2_type, str(self.cr), parameter1_str, parameter2_str) 
    
    def evaluate(self, state): 
        s = self.__str__()
        if s in state: 
            return True
        else:
            return False
    
    def __eq__(self,o):
        if not super(GroundedRelation,self).__eq__(o):
            return False 
        elif self.parameter1 == o.parameter1 and self.parameter2 == o.parameter2 and self.region == o.region:
            return True
        elif self.parameter1 == o.parameter2 and self.parameter2 == o.parameter1 and self.region == o.region:
            return True
        else: 
            return False
        
    def __lt__(self,o): 
        return super(GroundedRelation,self).__lt__(o)
        
    def __hash__(self):
        parameter1_str = self.parameter1
        if self.parameter1_type in Config.CONST_TYPES:
            parameter1_str = useful_functions.update_to_const(self.parameter1)
        
        parameter2_str = self.parameter2
        if self.parameter2_type in Config.CONST_TYPES:
            parameter2_str = useful_functions.update_to_const(self.parameter2)

        if self.cr != 0:
            return hash("({}_{}_{} {} {})".format(self.parameter1_type, self.parameter2_type, str(self.region), parameter1_str, parameter2_str))        
        else:
            return hash("({}_{}_{} {} {})".format(self.parameter1_type, self.parameter2_type, str(len(self.region)), parameter1_str, parameter2_str))        

    def get_lifted_relation(self):
        return Relation(self.parameter1_type,self.parameter2_type,self.cr, self.region,self.discretizer,self.mvg, self.sampling_region_list)

    def evaluate_in_ll_state(self,ll_state):
        if self.current_mapping_pair is None:
            possible_pairs = product(self.mapping[self.parameter1],self.mapping[self.parameter2])
        else:
            possible_pairs = [self.current_mapping_pair]

        truth_list = []

        for mapping_pair in possible_pairs:
            truth_list.append(self.evaluate_for_map_in_ll_state(ll_state,mapping_pair))
        
        if self.cr == 0:
            return all(truth_list)
        else:
            return any(truth_list)

    def evaluate_for_map_in_ll_state(self,ll_state,mapping_pair=None):
        if mapping_pair is not None:
            param1,param2 = mapping_pair
        else:
            param1,param2 = self.current_mapping_pair
        
        if param1 not in ll_state.object_dict.keys() or param2 not in ll_state.object_dict.keys():
            if self.cr == 0:
                return True
            
            return False

        link1_relative_pose, link2_relative_pose = self.get_relative_pose(ll_state,mapping_pair)
        link1_relative_discretized_pose = self.discretizer.get_discretized_pose(link1_relative_pose,is_relative = self.relational)
        link2_relative_discretized_pose = self.discretizer.get_discretized_pose(link2_relative_pose,is_relative = self.relational)
        
        link1_relative_discretized_pose.extend(Config.SENSOR_COUNT*[0])
        link2_relative_discretized_pose.extend(Config.SENSOR_COUNT*[0])
    
        grab_flag = 0
        grabbed = False
        
        for n in range(1,ll_state.num_robots+1):
            grabbed = (grabbed or getattr(ll_state,"grabbed_flag_{}".format(n)))

        if grabbed:
            grabbed_object_flag = False
            for r in range(1,ll_state.num_robots+1):
                if (param1 == getattr(ll_state,"grabbed_object_{}".format(r)) or param2 == getattr(ll_state,"grabbed_object_{}".format(r))) or (self.parameter1_type in Config.ROBOT_TYPES and self.parameter2_type in Config.ROBOT_TYPES):
                    grabbed_object_flag = True

            if (self.parameter1_type == Config.GRIPPER_NAME or self.parameter2_type == Config.GRIPPER_NAME) and grabbed_object_flag:
                grab_flag = 0
                if (self.parameter1_type == Config.GRIPPER_NAME) and (self.parameter2_type in Config.OBJECT_NAME):
                    id = param1.split("_")[Config.OBJ_ID_IND]

                    if grabbed:
                        if param2 == getattr(ll_state,"grabbed_object_{}".format(id)):
                            grab_flag = 1
                        else:
                            grab_flag = 2
                    else:
                        grab_flag = 0

                elif (self.parameter2_type == Config.GRIPPER_NAME) and (self.parameter1_type in Config.OBJECT_NAME):
                    id = param2.split("_")[Config.OBJ_ID_IND]
                    
                    if grabbed:
                        if param1 == getattr(ll_state,"grabbed_object_{}".format(id)):
                            grab_flag = 1
                        else:
                            grab_flag = 2
                    else:
                        grab_flag = 0                  

        link1_relative_discretized_pose[Config.GRAB_INDEX] = grab_flag
        link2_relative_discretized_pose[Config.GRAB_INDEX] = grab_flag

        if self.cr == 0: 
            if self.parameter1_type == self.parameter2_type:
                return not (link2_relative_discretized_pose in self.region)
            else:
                return not (link1_relative_discretized_pose in self.region or link2_relative_discretized_pose in self.region)
        else:
            if self.parameter1_type == self.parameter2_type:
                return (link2_relative_discretized_pose in self.region)
            else:
                return (link1_relative_discretized_pose in self.region or link2_relative_discretized_pose in self.region)

    def get_grounded_pose(self,lifted_transform, env_state, switch=False, mapping_pair=None):
        if mapping_pair is not None:
            param1,param2 = mapping_pair
        else:
            param1,param2 = self.current_mapping_pair

        object_dic = env_state.object_dict

        if switch:
            object_name = param2

        else:
            object_name = param1
        
        current_link1_pose = object_dic[object_name]
        if object_name.split("_")[Config.OBJ_TYPE_IND] in Config.ROBOT_TYPES.keys():
            current_link1_pose = object_dic[object_name][1]
        current_link1_transform = useful_functions.transform_from_pose(current_link1_pose)
        return current_link1_transform.dot(lifted_transform)
    
    def get_relative_pose(self, env_state, mapping_pair=None):
        object_dic = env_state.object_dict

        if mapping_pair is not None:
            param1,param2 = mapping_pair
        else:
            param1,param2 = self.current_mapping_pair

        if self.parameter1_type in Config.ROBOT_TYPES.keys():
            link1_pose = object_dic[param1][1]
        else:
            link1_pose = object_dic[param1]

        if self.parameter2_type in Config.ROBOT_TYPES.keys():
            link2_pose = object_dic[param2][1]
        else:
            link2_pose = object_dic[param2]

        relative_pose_1 = useful_functions.get_relative_pose(link2_pose,link1_pose)
        relative_pose_2 = useful_functions.get_relative_pose(link1_pose,link2_pose)

        return relative_pose_1,relative_pose_2

    def get_next_region(self):
        if self.cr!= 0: 
            for r, region in enumerate(self.sampling_region_list): 
                yield r
        else:
            yield 0

    def sample_region(self): 
        if self.region_generator is None: 
            self.region_generator = self.get_next_region() 
        try: 
            region = self.region_generator.next() 
        except StopIteration:
            self.region_generator = None  
            raise StopIteration
        else: 
            return region 
        
    def init_sample_generator(self,env_state,sim_object, region,action_info):
        samples = [] 
        self.env_state = env_state
        self.sim_object = sim_object
        self.region_to_use = region
        self.sample_fn = self.sampler(action_info)
    
    def sampler(self,action_info):
        if self.cr == 0:
            n = 1
        else:
            n = Config.SAMPLE_COUNT

        for i in range(n):
            yield self.sample_config(action_info)
    
    def get_next_sample(self):
        if self.sample_fn is None: 
            raise StopIteration
        try: 
            sample = self.sample_fn.next() 
        except StopIteration:
            self.sample_fn = None 
            raise StopIteration
        else:
            return sample 

    def sample_config(self,action_info,mapping_pair=None):
        if mapping_pair is not None:
            param1,param2 = mapping_pair
        else:
            param1,param2 = self.current_mapping_pair

        env_state = self.env_state 
        sim_object = self.sim_object 
        region = self.region[self.region_to_use]
        switch = False
        object_with_transform = param2
        static_object = param1
        static_param_num = 1
        obj_list = []

        action_type, action_axis, action_order = action_info

        if self.switch_check(object_with_transform=object_with_transform,env_state=env_state):
            object_with_transform = param1
            static_object = param2
            static_param_num = 2
            switch = True

        static_list = [static_object,static_param_num]

        t_robot=None
        for obj in env_state.object_dict.keys():
            if (self.parameter1_type == Config.BASE_NAME or self.parameter2_type == Config.BASE_NAME) and (self.parameter1_type != Config.GRIPPER_NAME and self.parameter2_type != Config.GRIPPER_NAME):
                if Config.BASE_NAME in obj:
                    t_robot = useful_functions.transform_from_pose(env_state.object_dict[obj][1])
                    robot_type = Config.BASE_NAME          
                    rob_id = obj.split("_")[Config.OBJ_ID_IND]
                    
            if obj == object_with_transform:
                pose = env_state.object_dict[obj]
                if object_with_transform.split("_")[Config.OBJ_TYPE_IND] in Config.ROBOT_TYPES.keys():
                    pose = env_state.object_dict[obj][1]
                t_obj = useful_functions.transform_from_pose(pose)
        
        if t_robot is None:
            if Config.GRIPPER_NAME in object_with_transform:
                rob_id = int(object_with_transform.split("_")[Config.OBJ_ID_IND])
            
            else:
                obj = object_with_transform
                for r in range(1,env_state.num_robots + 1):
                    if obj == getattr(env_state,"grabbed_object_{}".format(r)):
                        rob_id = r
                        break                                      
        
            for obj in env_state.object_dict.keys():
                if Config.GRIPPER_NAME in obj and int(obj.split("_")[Config.OBJ_ID_IND]) == rob_id:
                    t_robot = useful_functions.transform_from_pose(env_state.object_dict[obj][1])
                    break
            
            robot_type = Config.GRIPPER_NAME
        
        for rob in sim_object.robots:
            if rob.id == rob_id:
                break

        if self.cr == 0:
            current_dof = env_state.object_dict[robot_type+"_{}".format(rob.id)][0]
            sampled_config,sampled_end_effector_transform = sim_object.robot.random_config_robot(current_dof=current_dof,collision_fn=sim_object.collision_check)
            sampled_config.extend(Config.SENSOR_COUNT*[0])
            sampled_lifted_region = np.zeros(shape=Config.BIN_COUNT.shape[0]+Config.SENSOR_COUNT)
            sampled_refined_grounded_region = sampled_config[:-Config.SENSOR_COUNT]
            object_with_transform = robot_type+"_{}".format(rob.id)
            delta_mp = None
            # robot = rob
        
        else:
            sampled_lifted_region = region 

            grab_flag = sampled_lifted_region[Config.GRAB_INDEX]
            sampled_refined_lifted_region = useful_functions.transform_from_pose(self.mvg.sample_from_component(component=self.region_to_use)[0])
            
            if switch:
                sampled_refined_lifted_region = np.linalg.pinv(sampled_refined_lifted_region)

            sampled_refined_grounded_region = self.get_grounded_pose(sampled_refined_lifted_region,env_state,switch=switch)

            relative_t = np.linalg.pinv(t_obj).dot(t_robot)
            sampled_end_effector_transform = sampled_refined_grounded_region.dot(relative_t)
            og_end_effector_transform = sampled_end_effector_transform
            
            sampled_config = self.get_robot_config(sampled_end_effector_transform, object_with_transform, rob, sim_object)
             
            delta_mp = None
            obj_list = []
            
            sampled_config = list(sampled_config)
            sampled_config.extend(Config.SENSOR_COUNT*[0])
            sampled_config[Config.GRAB_INDEX] = grab_flag

        return sampled_config,sampled_lifted_region,object_with_transform,sampled_refined_grounded_region,rob, static_list, sampled_end_effector_transform, obj_list, delta_mp
        
    def switch_check(self,object_with_transform,env_state,mapping_pair=None):
        if mapping_pair is not None:
            param1,param2 = mapping_pair
        else:
            param1,param2 = self.current_mapping_pair

        if (object_with_transform.split("_")[Config.OBJ_TYPE_IND] in Config.IMMOVABLE_OBJECTS):
            return True
        else:
            if ((self.parameter1_type == self.parameter2_type) and (self.parameter1_type in Config.ROBOT_TYPES)):
                return False
            else:
                grabbed_object_flag = False
                for r in range(1,env_state.num_robots+1):
                    if getattr(env_state,"grabbed_object_{}".format(r)) in [param1, param2]:
                        grabbed_object_flag = True
                        grabbed_object = getattr(env_state,"grabbed_object_{}".format(r))
                        break

                if grabbed_object_flag:
                    if Config.GRIPPER_NAME in self.parameter1_type:
                        id = param1.split("_")[Config.OBJ_ID_IND]
                        if object_with_transform == getattr(env_state,"grabbed_object_{}".format(id)):
                            return True
                        else:
                            return False                    
                    elif grabbed_object == param1:
                        return True
                    else:
                        return False

                else:
                    if self.parameter1_type in Config.ROBOT_TYPES.keys() and self.parameter2_type not in Config.ROBOT_TYPES.keys():
                        return True
                    else:
                        return False

    def get_robot_config(self,sampled_end_effector_transform, object_with_transform, rob, sim_object, check_collisions = True):
        sampled_config = []
        ik_count = 0
        while ik_count < Config.MAX_IK_ATTEMPTS and len(sampled_config) == 0:
            sampled_config = rob.get_ik_solutions(sampled_end_effector_transform,robot_param=object_with_transform.split("_")[Config.OBJ_TYPE_IND],collision_fn = sim_object.collision_check,check_collisions=check_collisions)
            ik_count += 1
        
        return sampled_config