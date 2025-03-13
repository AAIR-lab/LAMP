from copy import deepcopy
import functools
import Config
import numpy as np
import copy
from src.useful_functions import *

@functools.total_ordering
class Relation(object):    
    def __init__(self, parameter1_type, parameter2_type, cr, region, discretizer): 
        self.parameter1_type = parameter1_type
        self.parameter2_type = parameter2_type
        self.cr = cr
        self.region = region
        self.discretizer = discretizer

    def get_grounded_relation(self,parameter1, parameter2): 
        return GroundedRelation(parameter1,parameter2,self.cr,self.region,self.discretizer)
    
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
        new_relation = Relation(self.parameter1_type, self.parameter2_type, self.cr, region_to_copy, self.discretizer)
        return new_relation
            
@functools.total_ordering
class GroundedRelation(Relation): 
    def __init__(self,parameter1,parameter2,cr,region,discretizer):
        super(GroundedRelation,self).__init__(parameter1.type, parameter2.type,cr,region,discretizer)
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

    def __deepcopy__(self,memodict={}): 
        region_to_copy = copy.deepcopy(self.region)
        new_relation = GroundedRelation(self.p1, self.p2, self.cr,region_to_copy,self.discretizer)
        return new_relation

    def __str__(self):
        parameter1_str = self.parameter1
        if self.parameter1_type in Config.CONST_TYPES[Config.DOMAIN_NAME]:
            parameter1_str = self.parameter1_type + "_Const"
        
        parameter2_str = self.parameter2
        if self.parameter2_type in Config.CONST_TYPES[Config.DOMAIN_NAME]:
            parameter2_str = self.parameter2_type + "_Const"
    
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
        # return hash(self.__str__())
        parameter1_str = self.parameter1
        if self.parameter1_type in Config.CONST_TYPES[Config.DOMAIN_NAME]:
            parameter1_str = self.parameter1_type + "_Const"
        
        parameter2_str = self.parameter2
        if self.parameter2_type in Config.CONST_TYPES[Config.DOMAIN_NAME]:
            parameter2_str = self.parameter2_type + "_Const"

        if self.cr != 0:
            return hash("({}_{}_{} {} {})".format(self.parameter1_type, self.parameter2_type, str(self.region), parameter1_str, parameter2_str))        
        else:
            return hash("({}_{}_{} {} {})".format(self.parameter1_type, self.parameter2_type, str(len(self.region)), parameter1_str, parameter2_str))        

    def get_lifted_relation(self):
        return Relation(self.parameter1_type,self.parameter2_type,self.cr, self.region,self.discretizer)

    def evaluate_in_ll_state(self,ll_state):
        link1_relative_pose, link2_relative_pose = self.get_relative_pose(ll_state)
        link1_relative_discretized_pose = self.discretizer.get_discretized_pose(link1_relative_pose,is_relative = self.relational)
        link2_relative_discretized_pose = self.discretizer.get_discretized_pose(link2_relative_pose,is_relative = self.relational)
        grab_flag = 0
        grabbed = False
        for n in range(1,ll_state.num_robots+1):
            grabbed = (grabbed or getattr(ll_state,"grabbed_flag_{}".format(n)))

        if grabbed:
            grabbed_object_flag = False
            for r in range(1,ll_state.num_robots+1):
                if (self.parameter1 == getattr(ll_state,"grabbed_object_{}".format(r)) or self.parameter2 == getattr(ll_state,"grabbed_object_{}".format(r))) or (self.parameter1_type in Config.ROBOT_TYPES and self.parameter2_type in Config.ROBOT_TYPES):
                    grabbed_object_flag = True

            if (self.parameter1_type == Config.GRIPPER_NAME or self.parameter2_type == Config.GRIPPER_NAME) and grabbed_object_flag:
                grab_flag = 0
                if (self.parameter1_type == Config.GRIPPER_NAME) and (self.parameter2_type in Config.OBJECT_NAME):
                    id = self.parameter1.split("_")[1]

                    if grabbed:
                        if self.parameter2 == getattr(ll_state,"grabbed_object_{}".format(id)):
                            grab_flag = 1
                        else:
                            grab_flag = 2
                    else:
                        grab_flag = 0

                elif (self.parameter2_type == Config.GRIPPER_NAME) and (self.parameter1_type in Config.OBJECT_NAME):
                    id = self.parameter2.split("_")[1]
                    
                    if grabbed:
                        if self.parameter1 == getattr(ll_state,"grabbed_object_{}".format(id)):
                            grab_flag = 1
                        else:
                            grab_flag = 2
                    else:
                        grab_flag = 0                  

        link1_relative_discretized_pose.append(grab_flag)
        link2_relative_discretized_pose.append(grab_flag)
        if self.cr == 0: 
            if self.parameter1_type == self.parameter2_type:
                return not (link2_relative_discretized_pose in self.region) #or (link1_relative_discretized_pose in self.region)
            else:
                return not (link1_relative_discretized_pose in self.region or link2_relative_discretized_pose in self.region)
        else:
            if self.parameter1_type == self.parameter2_type:
                return (link2_relative_discretized_pose in self.region) #or (link1_relative_discretized_pose in self.region)
            else:
                return (link1_relative_discretized_pose in self.region or link2_relative_discretized_pose in self.region)

    def get_grounded_pose(self,lifted_transform, env_state,switch=False):
        object_dic = env_state.object_dict
        if switch:
            object_name = self.parameter2
            if "Const" in self.parameter2:
                object_name = self.parameter2_type + "_" + self.parameter1.split("_")[-1]

        else:
            object_name = self.parameter1
            if "Const" in self.parameter1:
                object_name = self.parameter1_type + "_" + self.parameter2.split("_")[-1]
        
        current_link1_pose = object_dic[object_name]
        if object_name.split("_")[0] in Config.ROBOT_TYPES.keys():
            current_link1_pose = object_dic[object_name][1]
        current_link1_transform = transform_from_pose(current_link1_pose)
        return current_link1_transform.dot(lifted_transform)
    
    def get_relative_pose(self, env_state):
        object_dic = env_state.object_dict
        parameter1 = self.parameter1
        parameter2 = self.parameter2

        if self.parameter1_type in Config.CONST_TYPES[Config.DOMAIN_NAME]:
            parameter1 = deepcopy(self.parameter1_type + "_" + parameter2.split("_")[-1])

        if self.parameter2_type in Config.CONST_TYPES[Config.DOMAIN_NAME]:
            parameter2 = deepcopy(self.parameter2_type + "_" + parameter1.split("_")[-1])

        if self.parameter1_type in Config.ROBOT_TYPES.keys():
            link1_pose = object_dic[parameter1][1]
        else:
            link1_pose = object_dic[parameter1]

        if self.parameter2_type in Config.ROBOT_TYPES.keys():
            link2_pose = object_dic[parameter2][1]
        else:
            link2_pose = object_dic[parameter2]

        relative_pose_1 = env_state.get_relative_pose(link2_pose,link1_pose)
        relative_pose_2 = env_state.get_relative_pose(link1_pose,link2_pose)
        return relative_pose_1,relative_pose_2

    def get_next_region(self):
        if self.cr!= 0: 
            for region in self.region: 
                yield region 
        else:
            yield self.region[0]

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

    def sample_config(self,action_info):
        env_state = self.env_state 
        sim_object = self.sim_object 
        region = self.region_to_use
        switch = False
        object_with_transform = self.parameter2
        static_object = self.parameter1
        static_param_num = 1
        obj_list = []

        action_type, action_axis, action_order = action_info

        if self.switch_check(object_with_transform=object_with_transform,env_state=env_state):
            object_with_transform = self.parameter1
            static_object = self.parameter2
            static_param_num = 2
            switch = True

        static_list = [static_object,static_param_num]

        t_robot=None
        for obj in env_state.object_dict.keys():
            if (self.parameter1_type == Config.BASE_NAME or self.parameter2_type == Config.BASE_NAME) and (self.parameter1_type != Config.GRIPPER_NAME and self.parameter2_type != Config.GRIPPER_NAME):
                if Config.BASE_NAME in obj:
                    t_robot = env_state.transform_from_pose(env_state.object_dict[obj][1])
                    robot_type = Config.BASE_NAME          
                    rob_id = obj.split("_")[1]
                    
            if obj == object_with_transform:
                pose = env_state.object_dict[obj]
                if object_with_transform.split("_")[0] in Config.ROBOT_TYPES.keys():
                    pose = env_state.object_dict[obj][1]
                t_obj = env_state.transform_from_pose(pose)
        
        if t_robot is None:
            if Config.GRIPPER_NAME in object_with_transform:
                rob_id = int(object_with_transform.split("_")[1])
            
            else:
                obj = object_with_transform
                for r in range(1,env_state.num_robots + 1):
                    if obj == getattr(env_state,"grabbed_object_{}".format(r)):
                        rob_id = r
                        break                                      
        
            for obj in env_state.object_dict.keys():
                if Config.GRIPPER_NAME in obj and int(obj.split("_")[1]) == rob_id:
                    t_robot = env_state.transform_from_pose(env_state.object_dict[obj][1])
                    break
            
            robot_type = Config.GRIPPER_NAME
        
        for rob in sim_object.robots:
            if rob.id == rob_id:
                break

        if self.cr == 0:
            current_dof = env_state.object_dict[robot_type+"_{}".format(rob.id)][0]
            sampled_config,sampled_end_effector_transform = sim_object.random_config_robot(robot=rob,current_dof=current_dof,exp=True) #TODO: set "exp" to False again
            sampled_config.append(0)
            sampled_lifted_region = np.zeros(shape=Config.BIN_COUNT.shape[0]+1)
            sampled_refined_grounded_region = sampled_config[:-1]
            object_with_transform = robot_type+"_{}".format(rob.id)
            delta_mp = None
            # robot = rob
        
        else:
            sampled_lifted_region = region 

            grab_flag = sampled_lifted_region[-1]
            if ( not switch and "Loc" in self.parameter1 ) or (switch and "Loc" in self.parameter2): 
                sampled_refined_lifted_region = np.eye(4)
            else: 
                sampled_refined_lifted_region = transform_from_pose(self.discretizer.convert_sample(sampled_lifted_region[:6], is_relative = True))

            if switch:
                sampled_refined_lifted_region = np.linalg.pinv(sampled_refined_lifted_region)

            sampled_refined_grounded_region = self.get_grounded_pose(sampled_refined_lifted_region,env_state,switch=switch)

            relative_t = np.linalg.pinv(t_obj).dot(t_robot)
            sampled_end_effector_transform = sampled_refined_grounded_region.dot(relative_t)
            sampled_config = []
            ik_count = 0

            while ik_count < Config.MAX_IK_ATTEMPTS and len(sampled_config) == 0:
                sampled_config = rob.get_ik_solutions(sampled_end_effector_transform,robot_param=object_with_transform.split("_")[0],collision_fn = sim_object.collision_check)
                ik_count += 1

            delta_mp = None
            obj_list = []
            
            sampled_config = list(sampled_config)
            sampled_config.append(grab_flag)

        return sampled_config,sampled_lifted_region,object_with_transform,sampled_refined_grounded_region,rob, static_list, sampled_end_effector_transform, obj_list, delta_mp
        
    def switch_check(self,object_with_transform,env_state):
        if (object_with_transform.split("_")[0] in Config.IMMOVABLE_OBJECTS):
            return True
        else:
            if ((self.parameter1_type == self.parameter2_type) and (self.parameter1_type in Config.ROBOT_TYPES)):
                return False
            else:
                grabbed_object_flag = False
                for r in range(1,env_state.num_robots+1):
                    if getattr(env_state,"grabbed_object_{}".format(r)) in [self.parameter1, self.parameter2]:
                        grabbed_object_flag = True
                        grabbed_object = getattr(env_state,"grabbed_object_{}".format(r))
                        break

                if grabbed_object_flag:
                    if Config.GRIPPER_NAME in self.parameter1_type:
                        id = self.parameter1.split("_")[1]
                        if object_with_transform == getattr(env_state,"grabbed_object_{}".format(id)):
                            return True
                        else:
                            return False                    
                    elif grabbed_object == self.parameter1:
                        return True
                    else:
                        return False

                else:
                    if self.parameter1_type in Config.ROBOT_TYPES.keys() and self.parameter2_type not in Config.ROBOT_TYPES.keys():
                        return True
                    else:
                        return False