import numpy as np
import cPickle
import tqdm
import math
import os
import time
from itertools import product
from Config import Config
import useful_functions
from copy import deepcopy

SimClass = Config.get_simulator_module("SimClass").SimClass
Object = Config.get_simulator_module("Object").Object
model_gen_utils = Config.get_simulator_module("model_gen_utils")

class CafeWorld(object):
    def __init__(self,env_name,number_of_configs=1,number_of_mp=1,axis_for_offset="x",file_name="_data.p",reference_structure_name=None,visualize=False,object_count=None,random=False,order=False,surface="",objects_in_init_state=0,quadrant=None,grasp_num=None,minimum_object_count=None,mp=True,num_robots=1,structure_dependance=False,object_list=[],experiment_flag=False,real_world_experiment=False,set_y=False,complete_random=False,data_gen=False,robot_name=Config.ROBOT_NAME):
        self.sim_object = SimClass(robot_name=robot_name,
                                   visualize=visualize)
            
        self.order = order
        self.env_name = env_name
        print(self.env_name)
        
        self.n = number_of_configs
        self.j = number_of_mp

        self.data = []
        self.envs = []
        self.seeds = []
        self.object_names = set()
        self.init_config = {}
        self.sim_object.load_env(env_name + ".dae")
        
        self.bound_object_name = ['world_final']
        self.sim_object.collision_set = set([self.sim_object.robot])
        for obj_name in self.bound_object_name:
            self.sim_object.collision_set.add(self.sim_object.get_obj(obj_name))

        self.clearance = 0.005
        self.obj_h = 0.05
        self.env_limits = self.sim_object.get_env_limits(self.bound_object_name)
        self.env_y_limits = self.env_limits[1] #[y_min,y_max]
        self.env_x_limits = self.env_limits[0] #[x_min,x_max]
        self.env_z_limits = [self.env_limits[-1],1.0]

        self.collision = False
        self.can_list = []
        self.table_list = []
        self.goalLoc_list = []
        self.quadrant = quadrant
        self.grasp_num = grasp_num
        self.match_quadrants = False

        self.object_count = object_count
        self.minimum_object_count = self.object_count

        self.file_name = file_name

        self.can_radius = 0.045
        self.can_h = 0.2

        self.countertop_h = 1.15
        
        self.default_countertop_range = [[-0.175,0.175], [-0.175,0.175]]
        self.countertop_quadrant_ranges = [[[0.1,0.175],[0.1,0.175]],
                                            [[-0.175,-0.1],[0.1,0.175]],
                                            [[-0.175,-0.1],[-0.175,-0.1]],
                                            [[0.1,0.175],[-0.175,-0.1]]]
        self.large_range_x = [-0.5,7]
        self.large_range_y = [-12,-4]

        self.table_spawn_x_range = [3,7]
        self.table_spawn_y_range = [-4,3]

        self.near_countertop_x = [-1.3,-0.3]
        self.near_countertop_y = [-0.4,0.8]

        self.table_h = 0.63
        self.table_top_thickness = 0.1
        
        self.default_table_range = [[-0.175,0.175], [-0.175,0.175]]
        self.table_quadrant_ranges = [[[0.1,0.175],[0.1,0.175]],
                                        [[-0.175,-0.1],[0.1,0.175]],
                                        [[-0.175,-0.1],[-0.175,-0.1]],
                                        [[0.1,0.175],[-0.175,-0.1]]]
        
        self.possible_samplers = ["table","countertop"]
        self.goalLoc_relative_pose = {"table":[1,0,0,0,0,0,(self.table_top_thickness/2.0)+self.clearance],
                                        "countertop":[1,0,0,0,0,0,0] }

        self.random = random
        self.added_objects = []
        self.surface = surface
        self.num_robots = num_robots

        if "problem" not in env_name:
            self.sim_object.change_obj_name("countertop","surface_0")
            
            for i in range(self.object_count):
                self.spawn_object()

            table_count = min(2,max(2,self.object_count))
            # table_count = 1
            for _ in range(table_count):
                self.spawn_table()

        else:
            self.can_list = [obj for obj in self.sim_object.get_objects() if "can" in str(obj.get_name())]
            self.table_list = [obj for obj in self.sim_object.get_objects() if "surface" in str(obj.get_name())]
            self.table_list.remove(self.sim_object.get_obj("surface_0"))

        self.trace = []
        self.compute_mp = mp

        self.randomize_env()

    def sample_can_countertop(self,object_name,surface_name="surface_0",range=[[-0.43,-0.1],[0.1,0.43]]):
        obj = self.sim_object.get_obj(object_name)
        t_current = obj.get_transform()
        current_grabbed_flag = False
        if self.sim_object.robot.grabbed_flag_1:
            current_grabbed_flag = True
            grabbed_object = deepcopy(self.sim_object.robot.grabbed_object_1)
            self.sim_object.robot.release()
        
        attempt_num = 0
        while attempt_num < 25:
            x = np.random.uniform(low=range[0][0]+self.can_radius,high=range[0][1]-self.can_radius)
            y = np.random.uniform(low=range[1][0]+self.can_radius,high=range[1][1]-self.can_radius)
            z = self.clearance

            countertop = self.sim_object.get_obj(surface_name)
            countertop_t = countertop.get_transform()

            t = self.sim_object.matrixFromPose([1,0,0,0,x,y,z])            

            t = countertop_t.dot(t)

            obj.set_transform(t)
            if not (self.sim_object.collision_check([obj]) or self.obj_checker(obj)):
                break
            attempt_num += 1
        
        obj.set_transform(t_current)
        if current_grabbed_flag:
            self.sim_object.robot.grab(obj=grabbed_object)
            current_grabbed_flag = False

        if attempt_num == 24:
            return None

        return t
    
    def sample_can_table(self,object_name,surface_name,range=[[-0.43,-0.1],[0.1,0.43]]):
        obj = self.sim_object.get_obj(object_name)
        t_current = obj.get_transform()
        current_grabbed_flag = False
        if self.sim_object.robot.grabbed_flag_1:
            current_grabbed_flag = True
            grabbed_object = deepcopy(self.sim_object.robot.grabbed_object_1)
            self.sim_object.robot.release()

        obj_dims = self.sim_object.get_object_dims(object_name=object_name)
        
        table = self.sim_object.get_obj(surface_name)
        table_t = table.get_transform()
        table_x,table_y,table_z = table_t[:3,3]
        table_dims = self.sim_object.get_object_dims(str(table.get_name()))
        attempt_num = 0
        while attempt_num < 25:
            x = np.random.uniform(low=range[0][0]+obj_dims[0],high=range[0][1]-obj_dims[0])
            y = np.random.uniform(low=range[1][0]+obj_dims[1],high=range[1][1]-obj_dims[1])
            z = self.table_top_thickness/2.0 + self.clearance

            grasp_num = np.random.choice(Config.NUM_GRASPS)
            rot_angle = (grasp_num * (2*np.pi) / Config.NUM_GRASPS)
            rot_z = self.sim_object.matrixFromAxisAngle([0,0,rot_angle])

            t = self.sim_object.matrixFromPose([1,0,0,0,x,y,z])
            t = table_t.dot(t)
            t = t.dot(rot_z)

            obj.set_transform(t)
            if not (self.sim_object.collision_check([obj]) or self.obj_checker(obj)):
                break

            attempt_num += 1
        
        obj.set_transform(t_current)
        if current_grabbed_flag:
            self.sim_object.robot.grab(obj=grabbed_object)
            current_grabbed_flag = False

        if attempt_num == 25:
            return None

        return t

    def sample_robot_base_countertop(self,surface_name="surface_0",mp=False,range=None,**kwargs):
        self.sim_object.robot.activate_base_joints()
        current_dof_vals = self.sim_object.robot.get_active_dof_values()

        countertop = self.sim_object.get_obj(surface_name)
        countertop_t = countertop.get_transform()

        diff = 0.4
        countertop_x_dim = 0.45
        x_offset = -diff-countertop_x_dim
        y_offset = 0

        diff_translation_matrix = self.sim_object.matrixFromPose([1,0,0,0,x_offset,y_offset,0])

        valid_pose = None
        count = 0
            
        while valid_pose is None and count < 5:
            rot_angle = (2*np.pi) / Config.NUM_GRASPS
            
            rot_Z = self.sim_object.matrixFromAxisAngle([0, 0, -np.pi/2])
            rot_mat = self.sim_object.matrixFromAxisAngle([0,0,rot_angle])
            t = np.eye(4)
            t = countertop_t.dot(rot_mat).dot(rot_Z).dot(diff_translation_matrix)

            _x = t[0,3]
            _y = t[1,3]
            _yaw = self.sim_object.axisAngleFromRotationMatrix(t[:3,:3])[-1]
            pose = [_x,_y,_yaw]

            self.sim_object.robot.set_active_dof_values(pose)
            count += 1
            if not self.sim_object.collision_check([self.sim_object.robot]):
                valid_pose = pose
                break
        
        self.sim_object.robot.set_active_dof_values(current_dof_vals)
        return valid_pose, self.sim_object.robot.get_link_transform("base_link")
    
    def sample_robot_base_table(self,surface_name,mp=False,range=None,**kwargs):
        self.sim_object.robot.activate_base_joints()
        current_dof_vals = self.sim_object.robot.get_active_dof_values()

        diff = 0.4
        table = self.sim_object.get_obj(surface_name)
        table_t = table.get_transform()
        table_dims = self.sim_object.get_object_dims(str(table.get_name()))
        diff_translation_matrix = self.sim_object.matrixFromPose([1,0,0,0,-diff-table_dims[0],0,0])

        valid_pose = None
        count = 0
            
        while valid_pose is None and count < 5:
            if self.grasp_num is None:
                rot_angle = np.random.uniform(low=-np.pi,high=np.pi)
            else:
                rot_angle = (self.grasp_num * (2*np.pi) / Config.NUM_GRASPS)                
            
            rot_Z = self.sim_object.matrixFromAxisAngle([0, 0, -np.pi/2])
            rot_mat = self.sim_object.matrixFromAxisAngle([0,0,rot_angle])
            t = np.eye(4)
            t = table_t.dot(rot_mat).dot(rot_Z).dot(diff_translation_matrix)

            _x = t[0,3]
            _y = t[1,3]
            _yaw = self.sim_object.axisAngleFromRotationMatrix(t[:3,:3])[-1]
            pose = [_x,_y,_yaw]

            self.sim_object.robot.set_active_dof_values(pose)
            count += 1

            if not self.sim_object.collision_check([self.sim_object.robot]):
                valid_pose = pose
                break
        
        self.sim_object.robot.set_active_dof_values(current_dof_vals)

        return valid_pose, self.sim_object.robot.get_link_transform("base_link")

    def table_checker(self,table):
        t1 = table.get_transform()[:2,3]
        for pl in self.table_list:
            if table!=pl:
                t2 = pl.get_transform()[:2,3]
                if abs(min(t1-t2)) > 3 or np.linalg.norm(t1-t2) > 3.5:
                    continue
                return True

        return False

    def table_randomizer(self):
        objects_to_remove = set()
        for obj in self.sim_object.collision_set:
            if obj.get_name().startswith("surface"):
                objects_to_remove.add(obj)

        for obj in objects_to_remove:
            self.sim_object.collision_set.remove(obj)

        for table in self.table_list:
            table_dims = self.sim_object.get_object_dims(object_name=str(table.get_name()))
            while True:
                x1 = np.random.uniform(low=self.large_range_x[0]+table_dims[0],high=self.large_range_x[1]-table_dims[0])
                y1 = np.random.uniform(low=self.large_range_y[0]+table_dims[1],high=self.large_range_y[1]-table_dims[1])

                x2 = np.random.uniform(low=self.table_spawn_x_range[0]+table_dims[0],high=self.table_spawn_x_range[1]-table_dims[0])
                y2 = np.random.uniform(low=self.table_spawn_y_range[0]+table_dims[1],high=self.table_spawn_y_range[1]-table_dims[1])

                z = self.table_h

                t1 = self.sim_object.matrixFromPose([1,0,0,0,x1,y1,z])
                t2 = self.sim_object.matrixFromPose([1,0,0,0,x2,y2,z])

                for t in [t1,t2]:
                    table.set_transform(t)
                    if not (self.sim_object.collision_check([table]) or self.table_checker(table)):
                        self.sim_object.collision_set.add(table)
                        break
                
                if table in self.sim_object.collision_set:
                    break
    
    def sample_grasp_pose(self,object_name="",pose = [],grasp_num=None,surface_id=None):
        if pose != []:
            world_T_obj = pose
        else:
            world_T_obj = self.sim_object.get_obj(object_name).get_transform()
                        
        self.sim_object.robot.activate_manip_joints()

        rot_Z = self.sim_object.matrixFromAxisAngle([0, 0, -np.pi/2])
        valid_pose = None
        gripper_offset = self.sim_object.robot.grasping_offset[Config.OBJECT_NAME[0]]
        if grasp_num is None:
            for j in range(Config.NUM_GRASPS):
                rot_ang = np.random.uniform(low = -np.pi, high = np.pi)
                obj_T_gripper = self.sim_object.matrixFromPose([1, 0, 0, 0, gripper_offset, 0, self.can_h/2.0])
                rot_mat = self.sim_object.matrixFromAxisAngle([0, 0, rot_ang])

                wrist_roll_pose = self.sim_object.robot.get_link_transform("wrist_roll_link")
                gripper_pose = self.sim_object.robot.get_link_transform("gripper_link")
                wrist_pose_wrt_gripper = np.matmul(np.linalg.inv(gripper_pose), wrist_roll_pose)

                grasp_T = np.eye(4).dot(rot_mat).dot(obj_T_gripper)
                grasp_T = world_T_obj.dot(grasp_T)
                grasp_T = np.matmul(grasp_T,wrist_pose_wrt_gripper)
                grasp_pose = self.sim_object.poseFromMatrix(grasp_T)
                ik_sols = self.sim_object.robot.get_ik_solutions(grasp_T,collision_fn=self.sim_object.collision_check)
                if len(ik_sols) > 0:
                    valid_pose = ik_sols
                    break
        else:
            if surface_id != 0:
                j = grasp_num
            else:
                j = 1

            rot_ang = (j * (2*np.pi) / Config.NUM_GRASPS)
            # print(rot_ang)
            obj_T_gripper = self.sim_object.matrixFromPose([1, 0, 0, 0, gripper_offset, 0, self.can_h/2.0])
            rot_mat = self.sim_object.matrixFromAxisAngle([0, 0, rot_ang])

            wrist_roll_pose = self.sim_object.robot.get_link_transform("wrist_roll_link")
            gripper_pose = self.sim_object.robot.get_link_transform("gripper_link")
            wrist_pose_wrt_gripper = np.matmul(np.linalg.inv(gripper_pose), wrist_roll_pose)

            grasp_T = world_T_obj.dot(rot_mat).dot(rot_Z).dot(obj_T_gripper)
            grasp_T = np.matmul(grasp_T,wrist_pose_wrt_gripper)
            
            grasp_pose = self.sim_object.poseFromMatrix(grasp_T)
            ik_sols = self.sim_object.robot.get_ik_solutions(grasp_T,collision_fn=self.sim_object.collision_check)
            if len(ik_sols) > 0:
                valid_pose = ik_sols
        
        return valid_pose

    def sample_goal_pose(self,config_tuple):
        obj_name,surface_name = config_tuple
        surface_id = int(surface_name.split("_")[Config.OBJ_ID_IND])

        if surface_id == 0:
            surface_type = "countertop"
        else:
            surface_type = "table"
        
        wrist_roll_pose = self.sim_object.robot.get_link_transform("wrist_roll_link")
        gripper_pose = self.sim_object.robot.get_link_transform("gripper_link")
        wrist_pose_wrt_gripper = np.matmul(np.linalg.inv(gripper_pose), wrist_roll_pose)

        count = 0
        valid_ik = None
        
        if self.quadrant is not None:
            quadrant_range = getattr(self,"{}_quadrant_ranges".format(surface_type))[self.quadrant-1]
        elif self.grasp_num is not None and self.match_quadrants:
            if surface_id != 0:
                q = self.grasp_num
            else:
                q = 1
            quadrant_range = getattr(self,"{}_quadrant_ranges".format(surface_type))[q]
        else:
            quadrant_range = getattr(self,"default_{}_range".format(surface_type))
            
        while count < Config.MAX_IK_ATTEMPTS and valid_ik is None:
            sampler = getattr(self,"sample_can_{}".format(surface_type))
            sample_obj_pose = sampler(object_name=obj_name,surface_name=surface_name,range=quadrant_range)
            
            sample_obj_pose[2,3] += self.can_h/2.0
            grasp_pose = sample_obj_pose.dot(wrist_pose_wrt_gripper)
            ik_sols = self.sim_object.robot.get_ik_solutions(grasp_pose,collision_fn=self.sim_object.collision_check)
            if len(ik_sols) > 0:
                valid_ik = ik_sols
            
            count+=1

        return valid_ik

    def get_random_range(self):
        max_y = -100000000
        for obj in self.sim_object.get_objects():
            if str(obj.get_name()).split("_")[Config.OBJ_TYPE_IND] in Config.SURFACE_NAME and int(str(obj.get_name()).split("_")[Config.OBJ_ID_IND])>0:
                max_y = max(obj.get_transform()[1,3],max_y)
        
        range = [self.large_range_x,([max_y+1.5,-1.2]),([-np.pi,np.pi])]

        return range

    def get_possible_configs(self,num_objects=1,configs_to_exclude=set([])):
        possible_configs=[]

        if len(self.table_list) > 1:
            random_cans = []
            while len(random_cans) < 2 and np.random.randint(0,2) and num_objects > 1:
                can_num = np.random.randint(1,num_objects)
                if can_num not in random_cans:
                    random_cans.append(can_num)

            for can_num in random_cans:
                possible_configs.append(("can_{}".format(can_num),"surface_0"))

        else:
            possible_configs.extend([("can_{}".format(can_num),"surface_0") for can_num in range(1,num_objects+1)])

        for i in range(1,len(self.table_list)+1):
            possible_configs.extend([("can_{}".format(can_num),"surface_{}".format(i)) for can_num in range(1,num_objects+1)])
        
        possible_configs = list(set(possible_configs).difference(configs_to_exclude))
        
        np.random.shuffle(possible_configs)
        return possible_configs
    
    def get_traj_config(self,init_configs, possible_configs):
        traj_config = []
        cans_placed = []

        for init_can, init_surface in init_configs:
            if init_can not in cans_placed:
                for config_can,config_surface in possible_configs:
                    if config_can == init_can:
                        traj_config.append(((init_can,init_surface),(config_can,config_surface)))
                        cans_placed.append(init_can)
                        break

        return traj_config

    def randomize_env(self,given_surface_lists=None,req_relation=None,traj_count=0,traj_config=None,configs_to_exclude=set([])):
        self.sim_object.collision_set.add(self.sim_object.robot)
        for obj_name in self.bound_object_name:
            self.sim_object.collision_set.add(self.sim_object.get_obj(obj_name))

        t = np.eye(4)
        t[2,3] = 10
        for obj in self.added_objects:
            obj.set_transform(t.dot(obj.get_transform()))
        self.added_objects = []

        self.table_randomizer()
        random_range = self.get_random_range()
        random_limits = zip(*random_range)
        self.sim_object.robot.activate_base_joints()
        self.sim_object.robot.random_config_robot(limits=random_limits,collision_fn=self.sim_object.collision_check,use_collision_set=True)

        quadrant_ranges = {}
        for key in Config.SURFACES:
            quadrant_ranges[key] = deepcopy(getattr(self,"{}_quadrant_ranges".format(key)))

        if self.minimum_object_count != self.object_count:
            objects_to_set = np.random.randint(low=self.minimum_object_count,high=self.object_count+1)
        else:
            objects_to_set = self.object_count

        if traj_config is None:
            # traj_config = self.set_traj_config(given_surface_lists,traj_count)
            possible_configs = self.get_possible_configs(objects_to_set,configs_to_exclude)
        else:
            possible_configs = [a for a,_ in traj_config]

        init_configs = []
        while len(init_configs) < objects_to_set:
            init_config = possible_configs.pop(0)
            can_name = init_config[0]
            can = self.sim_object.get_obj(can_name)
            if can in self.added_objects or init_config in init_configs:
                possible_configs.append(init_config)
                continue
            init_surface_name = init_config[1]
            init_surface_id = int(init_surface_name.split("_")[Config.OBJ_ID_IND])
            init_surface_T = self.sim_object.get_obj(init_surface_name).get_transform()
            init_surface_p = useful_functions.pose_from_transform(init_surface_T)
            if init_surface_id == 0:
                init_surface_type = "countertop"
            else:
                init_surface_type = "table"

            if req_relation is None:
                if self.grasp_num is not None and self.match_quadrants:
                    if init_surface_id != 0:
                        q = self.grasp_num
                    else:
                        q = 1
                    sample_quadrant_range = quadrant_ranges[init_surface_type][q]

                elif self.quadrant is not None:
                    if init_surface_id != 0:
                        q = self.quadrant-1
                    else:
                        q = 1
                    sample_quadrant_range = quadrant_ranges[init_surface_type][q]
                else:
                    sample_quadrant_range = getattr(self,"default_{}_range".format(init_surface_type))

                t = self.object_randomizer(obj_name=can_name,surface_name=init_surface_name,range=sample_quadrant_range)
                if t is None:
                    possible_configs.append(init_config)
                    continue
                else:
                    init_configs.append(init_config)
            
            else:
                rcr = [r.region for r in req_relation]
                sampler = getattr(self,"sample_can_{}".format(init_surface_type))
                
                discretizer = req_relation[0].discretizer
                sample_flag = True
                self.sim_object.collision_set.add(self.sim_object.get_obj(self.bound_object_name[0]))
                while sample_flag:
                    if init_surface_id == 0:
                        surface_range = self.default_countertop_range
                    else:
                        surface_range = self.default_table_range
                    ll_T = sampler(object_name=can_name,surface_name=init_surface_name,range=surface_range)
                    if ll_T is not None:
                        ll_P = useful_functions.pose_from_transform(ll_T)
                        can.set_transform(ll_T)
                        relative_pose = useful_functions.get_relative_pose(pose1=ll_P,pose2=init_surface_p)
                        discretized_pose = discretizer.get_discretized_pose(input_pose=relative_pose,is_relative=True)
                        discretized_pose.append(int(self.sim_object.robot.grabbed_flag_1))

                        if not (self.sim_object.collision_check([can]) or self.obj_checker(can)):
                            for region in rcr:
                                if discretized_pose in region:
                                    sample_flag = False
                                    if str(can.get_name()).split("_")[Config.OBJ_TYPE_IND] == Config.OBJECT_NAME[0]:
                                        self.added_objects.append(can)
                                    break
                        if not sample_flag:
                            init_configs.append(init_config)
                    else:
                        possible_configs.append(init_config)
                        break

        if traj_config is None:
            traj_config = self.get_traj_config(init_configs,possible_configs)

        obj_dic = {}
        for obj in self.sim_object.get_objects():
            if str(obj.get_name()) not in self.bound_object_name:
                if obj != self.sim_object.robot:
                    name = str(obj.get_name())
                    obj_dic[name] = obj.get_transform()
                else:
                    self.sim_object.robot.activate_manip_joints()
                    obj_dic["gripper_1"] = self.sim_object.robot.get_active_dof_values()

                    self.sim_object.robot.activate_base_joints()
                    obj_dic["freight_1"] = self.sim_object.robot.get_active_dof_values()
        
        return obj_dic, traj_config[:objects_to_set]

    def set_traj_config(self,given_surface_lists=None,traj_count=0):
        traj_config = []
        for i in range(self.object_count):
            if given_surface_lists is None:
                can_traj_config = []
                can_name = str(self.can_list[i].get_name())

                possible_samplers = deepcopy(self.possible_samplers)
                
                possible_surfaces = ["surface_0"]
                for table in self.table_list:
                    possible_surfaces.append(str(table.get_name()))
                
                j = np.random.randint(low=0,high=len(possible_surfaces))
                init_surface_name = possible_surfaces[j]

                can_traj_config.append((can_name,init_surface_name))
                possible_surfaces.remove(init_surface_name)
                
                if len(possible_surfaces) == 1:
                    j = 0
                else:
                    j = np.random.randint(low=0,high=len(possible_surfaces))
                
                goal_surface_name = possible_surfaces[j]
                can_traj_config.append((can_name,goal_surface_name))

            else:
                can_traj_config = []
                can_name = given_surface_lists[i][0]
                possible_samplers = deepcopy(self.possible_samplers)

                goal_sampler_id = int(given_surface_lists[i][1].split("_")[Config.OBJ_ID_IND])
                if goal_sampler_id == 0:
                    goal_sampler_type = "countertop"
                else:
                    goal_sampler_type = "table"
                
                if self.order:
                    possible_samplers.remove(goal_sampler_type)

                if len(possible_samplers) == 1:
                    index = 0
                else:
                    index = np.random.randint(len(possible_samplers))

                init_sampler_type = possible_samplers[index]
                if init_sampler_type == "countertop":
                    init_surface_name = "surface_0"

                else:
                    l = getattr(self,"{}_list".format(init_sampler_type))
                    if len(l) == 0:
                        init_surface_name = str(l[0].get_name())
                    else:
                        index = np.random.randint(len(l))
                        init_surface_name = str(l[index].get_name())                    

                can_traj_config.append([can_name,init_surface_name])
                can_traj_config.append(given_surface_lists[i])

            traj_config.append(can_traj_config)

        return traj_config

    def setup_objects(self,init_object_dict):
        self.sim_object.robot.release()
        for obj_name in init_object_dict.keys():
            if obj_name not in self.sim_object.robot.robot_type_object_mappings.values():
                obj = self.sim_object.get_obj(obj_name)
                obj.set_transform(init_object_dict[obj_name])
            else:
                dof_val = init_object_dict[obj_name]
                if len(dof_val) == 3:
                    self.sim_object.robot.activate_base_joints()
                else:
                    self.sim_object.robot.activate_manip_joints()
                self.sim_object.robot.set_active_dof_values(dof_val)

    def motion_plan_robot_base(self,config_tuple=None,given_pose=None):
        self.sim_object.robot.activate_base_joints()
        
        if config_tuple is not None:
            object_name, surface_name = config_tuple
            surface_id = int(surface_name.split("_")[Config.OBJ_ID_IND])
            if surface_id == 0:
                surface_type = "countertop"
            else:
                surface_type = "table"
            sampler = getattr(self,"sample_robot_base_{}".format(surface_type))
        else:
            surface_name = None
            sampler = self.sim_object.robot.random_config_robot

        range=self.get_random_range()
        traj = None
        count=0
        while traj is None and count<5:
            target_pose = None
            if given_pose is None:
                target_pose,_ = sampler(surface_name=surface_name,mp=True,range=range,limits=zip(*range),collision_fn=self.sim_object.collision_check)
            else:
                target_pose = given_pose

            if target_pose is not None:
                if self.compute_mp:
                    traj = self.sim_object.compute_motion_plan(target_pose)
                else:
                    traj = [target_pose]
            
            count += 1
        
        return traj, target_pose
    
    def motion_plan_robot_arm(self,config_tuple=None,pose=[],grasp_num=None,given_pose=None,surface_id=None):
        self.sim_object.robot.activate_manip_joints()
        
        if config_tuple is not None:
            object_name, surface_name = config_tuple
            surface_id = int(surface_name.split("_")[Config.OBJ_ID_IND])
            sampler = self.sample_grasp_pose
        else:
            object_name = None
            sampler = self.sim_object.robot.get_arm_tuck_dofs

        counter = 0
        traj = None
        target_pose = None            

        while traj is None and counter<5:
            if given_pose is None:
                target_pose = sampler(object_name=object_name,pose=pose,grasp_num=grasp_num,surface_id=surface_id)
            else:
                target_pose=given_pose

            if target_pose is not None:
                if self.compute_mp:
                    traj = self.sim_object.compute_motion_plan(target_pose)
                else:
                    traj = [target_pose]

            counter+=1

        return traj,target_pose

    @SimClass.conditional_env_lock
    def start(self,complete_random=False):
        i = 0        
        flag = 0
        j = 0
        pbar = tqdm.tqdm(total=self.n)
        
        if self.sim_object.GetViewer() is not None:
            self.sim_object.set_camera_wrt_obj("world_final")
            
        while i < self.n:
            init_object_dic,traj_config = self.randomize_env(traj_count=i)
            random_range = self.get_random_range()
            random_limits = zip(*random_range)
            while j <= self.j:
                self.setup_objects(init_object_dic)
                state_list = []

                for init_config,goal_config in traj_config:
                    object_name, surface1_name = init_config
                    _, surface2_name = goal_config
                    surface1_id = int(surface1_name.split("_")[Config.OBJ_ID_IND])
                    if surface1_id == 0:
                        surface1_type = "countertop"
                    else:
                        surface1_type = "table"

                    surface2_id = int(surface2_name.split("_")[Config.OBJ_ID_IND])
                    if surface2_id == 0:
                        surface2_type = "countertop"
                    else:
                        surface2_type = "table"

                    #KP_1
                    state_1 = [self.sim_object.get_one_state()]

                    base_sampler = getattr(self,"sample_robot_base_{}".format(surface1_type))

                    # print("sampling surface 1")
                    base_sample_1,_ = base_sampler(surface_name=surface1_name)
                        
                    if base_sample_1 is None:
                        if j == 0:
                            flag=1
                        else:
                            flag=2
                        break

                    self.sim_object.robot.activate_base_joints()
                    init_base_pose = self.sim_object.robot.get_active_dof_values()
                    self.sim_object.robot.set_active_dof_values(base_sample_1)

                    # print("grasp sampling")
                    traj_2,_ = self.motion_plan_robot_arm(init_config,grasp_num=self.grasp_num,surface_id=surface1_id)

                    if traj_2 is None:
                        if j == 0:
                            flag=1
                        else:
                            flag=2
                        break
                    
                    self.sim_object.robot.activate_base_joints()
                    self.sim_object.robot.set_active_dof_values(init_base_pose)
                    # print("motion planning for base to surface 1")
                    traj_1, base_sample_1 = self.motion_plan_robot_base(init_config,given_pose=base_sample_1)
                    if traj_1 is None:
                        if j == 0:
                            flag=1
                        else:
                            flag=2
                        break
                    
                    #KP_2
                    state_2 = self.sim_object.get_state_block(traj=traj_1,config_tuple=init_config)
                    #KP_3
                    state_3 = []
                    if not self.compute_mp:
                        self.sim_object.robot.activate_manip_joints()
                        current_dof = self.sim_object.robot.get_active_dof_values()
                        random_pose,_ = self.sim_object.robot.random_config_robot(current_dof=current_dof,collision_fn=self.sim_object.collision_check)
                        state_3 = self.sim_object.get_state_block(traj=[random_pose],config_tuple=init_config)

                    #KP_4 and KP_5
                    state_4_5 = self.sim_object.get_state_block(traj=traj_2,config_tuple=init_config,grab=True)
                    #KP_6
                    state_6 = []
                    if not self.compute_mp:
                        self.sim_object.robot.activate_manip_joints()
                        current_dof = self.sim_object.robot.get_active_dof_values()
                        random_pose,_ = self.sim_object.robot.random_config_robot(current_dof=current_dof,collision_fn=self.sim_object.collision_check)
                        state_6 = self.sim_object.get_state_block(traj=[random_pose],config_tuple=init_config)

                    # print("tucking arm with object")
                    traj_3, _ = self.motion_plan_robot_arm()
                    if traj_3 is None:
                        if j == 0:
                            flag=1
                        else:
                            flag=2
                        break
                        
                    #KP_7
                    state_7 = self.sim_object.get_state_block(traj=traj_3,config_tuple=init_config)
                    
                    #KP_8
                    state_8 = []
                    if not self.compute_mp:
                        self.sim_object.robot.activate_base_joints()
                        current_dof = self.sim_object.robot.get_active_dof_values()
                        random_pose,_ = self.sim_object.robot.random_config_robot(current_dof=current_dof,limits=random_limits,collision_fn=self.sim_object.collision_check)
                        state_8 = self.sim_object.get_state_block(traj=[random_pose],config_tuple=init_config)

                    # print("sampling surface 2")
                    base_sampler = getattr(self,"sample_robot_base_{}".format(surface2_type))
                    base_sample_2,_ = base_sampler(surface_name=surface2_name)
                    if base_sample_2 is None:
                        if j == 0:
                            flag=1
                        else:
                            flag=2
                        break

                    self.sim_object.robot.activate_base_joints()
                    init_base_pose = self.sim_object.robot.get_active_dof_values()
                    self.sim_object.robot.set_active_dof_values(base_sample_2)
                    
                    # print("trying to put down object")
                    traj_5 = None
                    count = 0
                    while traj_5 is None and count < 5:
                        ik = self.sample_goal_pose(goal_config)
                        if ik is not None:
                            if self.compute_mp:
                                traj_5 = self.sim_object.compute_motion_plan(goal=ik)
                            else:
                                traj_5 = [ik]

                        count += 1
                    
                    if traj_5 is None:
                        if j == 0:
                            flag=1
                        else:
                            flag=2
                        break
                    
                    self.sim_object.robot.activate_base_joints()
                    self.sim_object.robot.set_active_dof_values(init_base_pose)

                    # print("motion planning to surface 2")
                    traj_4, base_sample_2 = self.motion_plan_robot_base(config_tuple=goal_config,given_pose=base_sample_2)
                    if traj_4 is None:
                        if j == 0:
                            flag=1
                        else:
                            flag=2
                        break
                        
                    #KP_9
                    state_9 = self.sim_object.get_state_block(traj=traj_4,config_tuple=goal_config)
                    #KP_10
                    state_10 = []
                    if not self.compute_mp:
                        self.sim_object.robot.activate_manip_joints()
                        current_dof = self.sim_object.robot.get_active_dof_values()
                        random_pose,_ = self.sim_object.robot.random_config_robot(current_dof=current_dof,collision_fn=self.sim_object.collision_check)
                        state_10 = self.sim_object.get_state_block(traj=[random_pose],config_tuple=init_config)
                    
                    #KP_11 and KP_12
                    state_11_12 = self.sim_object.get_state_block(traj=traj_5,config_tuple=goal_config,grab=False)

                    #KP_13
                    state_13 = []
                    if not self.compute_mp:
                        self.sim_object.robot.activate_manip_joints()
                        current_dof = self.sim_object.robot.get_active_dof_values()
                        random_pose,_ = self.sim_object.robot.random_config_robot(current_dof=current_dof,collision_fn=self.sim_object.collision_check)
                        state_13 = self.sim_object.get_state_block(traj=[random_pose],config_tuple=init_config)

                    #tucking arm
                    # print("tucking empty arm")
                    traj_6 = None
                    count = 0
                    while traj_6 is None and count<5:
                        traj_6,_ = self.motion_plan_robot_arm()
                        count+=1

                    if traj_6 is None:
                        if j == 0:
                            flag=1
                        else:
                            flag=2
                        break
                        
                    #KP_14
                    state_14 = self.sim_object.get_state_block(traj=traj_6,config_tuple=init_config)

                    #random_placing_robot
                    # print("random place robot in end")
                    count = 0
                    traj_7 = None
                    while traj_7 is None and count < 5:
                        traj_7,_ = self.motion_plan_robot_base()
                        count += 1
                    
                    if traj_7 is None:
                        if j == 0:
                            flag=1
                        else:
                            flag=2
                        break

                    #KP_15
                    state_15 = self.sim_object.get_state_block(traj=traj_7,config_tuple=goal_config)

                    states = [state_1,state_2,state_3,state_4_5,state_6,state_7,state_8,state_9,state_10,state_11_12,state_13,state_14,state_15]
                    if self.random != -1:
                        last_ind = np.random.randint(low=1,high=len(states))
                    else:
                        last_ind = len(states)
                    
                    for state_num in range(last_ind):
                        state_list.extend(states[state_num])

                if flag == 1:
                    break
                elif flag == 2:
                    # print("continuing\n")
                    flag = 0
                    continue
                
                for obj in self.sim_object.get_objects():
                    if obj != self.sim_object.robot:
                        self.object_names.add(str(obj.get_name()))
                    else:
                        for key in self.sim_object.robot.robot_type_object_mappings:
                            self.object_names.add(self.sim_object.robot.robot_type_object_mappings[key])

                self.data.append(state_list)

                j += 1
                if j == self.j:
                    break
            
            j=0
            if flag == 1:
                flag = 0
                # print("reset\n")
                continue
            
            pbar.update(1)
            i=i+1
        
        pbar.close()                    
        time.sleep(5)
        self.save_data()
    
    @SimClass.env_lock
    def save_data(self,):
        final_data = {"env_states": self.data}
        object_data = self.object_names

        path = Config.DATA_MISC_DIR+ self.env_name+"/"
        if not os.path.exists(path):
            os.makedirs(path)

        cPickle.dump(final_data,open(Config.DATA_MISC_DIR+ self.env_name+"/"+ self.file_name ,"wb"),protocol=cPickle.HIGHEST_PROTOCOL)
        cPickle.dump(object_data,open(Config.DATA_MISC_DIR+ self.env_name+"/"+ self.env_name + "_object_list.p" ,"wb"),protocol=cPickle.HIGHEST_PROTOCOL)
                
        print("{} data saved".format(self.env_name))

    def obj_checker(self,obj):
        t_y = obj.get_transform()[1,3]
        t_x = obj.get_transform()[0,3]
        for pl in self.added_objects:
            if obj!=pl:
                p_y = pl.get_transform()[1,3]
                p_x = pl.get_transform()[0,3]
                if math.sqrt((p_y-t_y)*(p_y-t_y) + (p_x-t_x)*(p_x-t_x)) < self.sim_object.robot.obj_offset:
                    return True
        
        return False

    def object_randomizer(self,obj_name,surface_name,range):
        surface_id = int(surface_name.split("_")[Config.OBJ_ID_IND])
        if surface_id == 0:
            surface_type = "countertop"
        else:
            surface_type = "table"
        sampler = getattr(self,"sample_can_{}".format(surface_type))

        obj = self.sim_object.get_obj(obj_name)
        t = sampler(object_name=obj_name,surface_name=surface_name,range=range)

        if t is not None:
            obj.set_transform(t)
            if str(obj.get_name()).split("_")[Config.OBJ_TYPE_IND] == Config.OBJECT_NAME[0]:
                self.added_objects.append(obj)

            self.sim_object.collision_set.add(obj)

            return obj.get_transform()
    
        else:
            return None
    
    def spawn_object(self):
        radius = self.can_radius
        height = 0.2
        color = [0,0.8,1]
        body_name = Config.OBJECT_NAME[0] + "_{}".format(len(self.can_list)+1)

        t = self.sim_object.matrixFromPose([1, 0, 0, 0, 0, 0, -0.5])
        cylinder = Object(model_gen_utils.create_cylinder(self.sim_object.env, body_name, t, [radius, height], color))

        self.sim_object.add_obj(cylinder)
        self.can_list.append(cylinder)

        return t
    
    def spawn_table(self):
        thickness = 0.1
        legheight = 0.55
        color = [1,0.8,0]
        pose = [3, -5, 0.63]

        name = "surface_{}".format(len(self.table_list)+1)        
        table = Object(model_gen_utils.create_cafe_table(self.sim_object.env,
                                                         name,
                                                         0.90, #dim1
                                                         0.90, #dim2
                                                         thickness,
                                                         0.1, #legdim1
                                                         0.1, #legdim2
                                                         legheight,
                                                         pose,
                                                         color))
         
        self.sim_object.add_obj(table)
        self.table_list.append(table)
    
    def setup_base_env(self):
        _,traj_config = self.randomize_env()

    def setup_exp(self,given_surface_lists=None,req_relation=None,experiment_flag=False):
        if not experiment_flag: 
            _,init_configs = self.randomize_env(req_relation=req_relation)
            init_state = self.sim_object.get_one_state()

            configs_to_exclude = set([a for a,_ in init_configs])
            _,goal_configs = self.randomize_env(req_relation=req_relation,configs_to_exclude=configs_to_exclude)

            traj_config = []
            for i in range(len(init_configs)):
                init_config,_ = init_configs[i]
                init_can, _ = init_config
                for j in range(len(goal_configs)):
                    goal_config,_ = goal_configs[j]
                    goal_can,_ = goal_config
                    if goal_can == init_can:
                        traj_config.append((init_config,goal_config))
                        break

            goal_state = self.sim_object.get_one_state()
            self.sim_object.set_env_state(init_state)

            return init_state,goal_state,traj_config
        else:
            for can in self.can_list:
                self.sim_object.collision_set.add(can)
            for table in self.table_list:
                self.sim_object.collision_set.add(table)