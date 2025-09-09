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

class DinnerTable(object):
    def __init__(self,env_name,number_of_configs=1,number_of_mp=1,axis_for_offset="x",file_name="_data.p",reference_structure_name=None,visualize=False,object_count=None,random=False,order=False,surface="",objects_in_init_state=0,quadrant=None,grasp_num=None,minimum_object_count=None,mp=True,num_robots=1,structure_dependance=False,object_list=[],experiment_flag=False,real_world_experiment=False,set_y=False,complete_random=False,data_gen=False,robot_name=Config.ROBOT_NAME):
        self.sim_object = SimClass(robot_name=robot_name,
                                   visualize=visualize)

        self.experiment_flag = experiment_flag
        
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

        self.clearance = 0.01

        self.collision = False
        self.grabbed_flag_1 = False
        self.object_list = []
        self.quadrant = quadrant
        self.grasp_num = grasp_num
        self.match_quadrants = False
        
        self.object_name_list = object_list

        self.spawned_object_count = 0

        self.object_count = object_count
        self.surface_list = []

        surface_count = 3
        for _ in range(surface_count):
            self.spawn_surface()

        self.sim_object.collision_set = set([self.sim_object.robot])
        self.sim_object.load_env("env_{}.dae".format(Config.BOUND_OBJECT_NAME))

        if real_world_experiment:
            self.populate_env(data["object_list"])
            # self.sim_object
            for obj in self.sim_object.get_objects():
                if str(obj.get_name()).split("_")[Config.OBJ_TYPE_IND] in Config.OBJECT_NAME:
                    self.object_list.append(obj)
                    
            for obj in self.sim_object.get_objects():
                self.sim_object.collision_set.add(obj)

        self.bound_object_name = [Config.BOUND_OBJECT_NAME]
        if not real_world_experiment:
            for obj_name in self.bound_object_name:
                self.sim_object.collision_set.add(self.sim_object.get_obj(obj_name))

        if minimum_object_count is None:
            self.minimum_object_count = self.object_count
        else:
            self.minimum_object_count = minimum_object_count

        self.file_name = file_name
        self.env_limits = self.sim_object.get_env_limits(self.bound_object_name)
        min_vec, max_vec = zip(*self.env_limits[:2])
        self.sim_object.robot.set_translation_limits([list(min_vec),list(max_vec)  ])
        self.sim_object.robot_dims = self.sim_object.get_object_dims(self.sim_object.robot.get_name())
        self.glass_h = 0.25

        self.large_range_x = [-0.5,7]
        self.large_range_y = [-12,-4]

        self.random = random
        self.added_objects = []
        self.surface = surface
        self.num_robots = num_robots
        self.robots = [self.sim_object.robot]

        self.surface_h = 0.73
        self.surface_top_thickness = 0.1
                                            
        self.default_surface_range = [[-0.175,0.175], [-0.175,0.175]]
        self.surface_quadrant_ranges = [[[0.1,0.175],[0.1,0.175]],
                                        [[-0.175,-0.1],[0.1,0.175]],
                                        [[-0.175,-0.1],[-0.175,-0.1]],
                                        [[0.1,0.175],[-0.175,-0.1]]]

        self.surface_spawn_x_range = self.large_range_x
        self.surface_spawn_y_range = self.large_range_y

        if not experiment_flag:
            self.setup_env(current_env_num=0,env_list=[0])
            self.randomize_env()
        elif not real_world_experiment:
            self.setup_env(current_env_num=0,env_list=[0])

        self.trace = []
        self.compute_mp = mp
        self.experiment_flag = experiment_flag

    def populate_env(self,obj_list=["glass_1","surface_1","surface_2"]):
        for obj in obj_list:
            obj_type = obj.split("_")[Config.OBJ_TYPE_IND]
            if obj_type not in Config.ROBOT_TYPES.keys():
                spawner = getattr(self,"spawn_{}".format(obj_type))
                if obj_type in Config.SURFACES:
                    spawner(id=obj.split("_")[-1])
                else:
                    spawner(obj)

    def setup_env(self,current_env_num,env_list=[]):
        for obj in self.object_name_list:
            for ob in self.sim_object.get_objects():
                if obj in ob.get_name():
                    self.sim_object.remove_obj(ob)
                    if ob in self.object_list:
                        self.object_list.remove(ob)
        
        self.spawned_object_count = 0

        if current_env_num in env_list:
            objects_to_spawn = self.object_count
        else:
            objects_to_spawn = 1

        for i in range(objects_to_spawn):
            obj = str(self.object_name_list[i%2])
            spawner = getattr(self,"spawn_{}".format(obj))
            spawner()

        for obj in self.object_list:
            self.spawn_loc(obj)

    def sample_obj_surface(self,object_name,surface_name,range=[[-0.43,-0.1],[0.1,0.43]]):
        current_grabbed_flag = False
        if self.grabbed_flag_1:
            current_grabbed_flag = True
            grabbed_object = deepcopy(self.grabbed_object_1)
            self.release()

        obj_dims = self.sim_object.get_object_dims(object_name=object_name)
        obj = self.sim_object.get_obj(object_name)

        table = self.sim_object.get_obj(surface_name)
        surface_t = table.get_transform()

        sample_count = 0
        found_flag = False
        while sample_count < 25:
            obj_t = np.eye(4)
            obj_t[2,3] += self.surface_top_thickness/2.0 + self.clearance
            obj_t[1,3] += np.random.uniform(low=range[1][0]+obj_dims[1],high=range[1][1]-obj_dims[1])

            grasp_num = self.grasp_num
            if self.grasp_num is None:
                grasp_num = np.random.choice(Config.NUM_GRASPS)
            rot_angle = (grasp_num * (2*np.pi) / Config.NUM_GRASPS)
            rot_z = self.sim_object.matrixFromAxisAngle([0,0,rot_angle])

            dt = np.eye(4)
            dt[0,3] = -(np.random.uniform(0.1,0.35))

            t = surface_t.dot(obj_t).dot(rot_z).dot(dt)
            obj.set_transform(t)
            
            found_flag = not (self.sim_object.collision_check([obj]) or self.obj_checker(obj))
            if found_flag :
                break

            sample_count += 1

        if found_flag:
            return t
        else:
            return None

    def sample_robot_base(self,object_name,mp=False):
        self.sim_object.robot.activate_base_joints()
        current_dof_vals = self.sim_object.robot.get_active_dof_values()

        diff = 0.8
        table = self.sim_object.get_obj(object_name)
        surface_t = table.get_transform()
        diff_translation_matrix = self.sim_object.matrixFromPose([1,0,0,0,-diff,0,0])

        valid_pose = None
        count = 0
            
        while valid_pose is None and count < 5:
            t = surface_t.dot(diff_translation_matrix)
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
        return valid_pose

    def surface_checker(self,table):
        t1 = table.get_transform()[:2,3]
        for pl in self.surface_list:
            if table!=pl:
                t2 = pl.get_transform()[:2,3]
                if abs(min(t1-t2)) > 3 or np.linalg.norm(t1-t2) > 3.5:
                    continue
                return True

        return False

    def surface_randomizer(self):
        # self.sim_object.collision_set.difference_update(set([obj for obj in self.sim_object.collision_set if str(obj.get_name()).startswith("surface")]))
        self.sim_object.collision_set.difference_update([obj for obj in self.sim_object.collision_set if str(obj.get_name()).startswith("surface")])

        for table in self.surface_list:
            surface_dims = self.sim_object.get_object_dims(object_name=str(table.get_name()))

            range_x = [self.large_range_x[0] + surface_dims[0] + self.clearance,
                    self.large_range_x[1] - surface_dims[0] - self.clearance]
            range_y = [self.large_range_y[0] + surface_dims[1] + self.clearance,
                    self.large_range_y[1] - surface_dims[1] - self.clearance]
            range_z = [self.surface_h, self.surface_h]

            current_t = table.get_transform()
            while True:
                t = self.sim_object.get_random_pose(table, range_x, range_y, range_z)
                table.set_transform(t)

                if not self.surface_checker(table):
                    break

            self.sim_object.collision_set.add(table)

    def sample_grasp_pose(self,object_name="",pose = [],grasp_num=None,surface_id=None):
        if pose != []:
            world_T_obj = pose
        else:
            world_T_obj = self.sim_object.get_obj(object_name).get_transform()
                        
        self.sim_object.robot.activate_manip_joints()
        wrist_roll_pose = self.sim_object.robot.get_link_transform('wrist_roll_link')
        gripper_pose = self.sim_object.robot.get_link_transform('gripper_link')
        wrist_pose_wrt_gripper = np.matmul(np.linalg.inv(gripper_pose), wrist_roll_pose)

        valid_pose = None
        if "bowl" not in object_name:
            rot_Z = self.sim_object.matrixFromAxisAngle([0, 0, -np.pi/2])
            gripper_offset = self.sim_object.robot.grasping_offset[object_name.split("_")[Config.OBJ_TYPE_IND]]
            rot_ang = ((2*np.pi)/Config.NUM_GRASPS)
            # print(rot_ang)
            obj_T_gripper = self.sim_object.matrixFromPose([1, 0, 0, 0, gripper_offset, 0, self.glass_h/2.0-0.01])
            rot_mat = self.sim_object.matrixFromAxisAngle([0, 0, rot_ang])

            grasp_T = world_T_obj.dot(rot_mat).dot(rot_Z).dot(obj_T_gripper)
            grasp_T = np.matmul(grasp_T,wrist_pose_wrt_gripper)
            
            ik_sols = self.sim_object.robot.get_ik_solutions(grasp_T,collision_fn=self.sim_object.collision_check)
            if len(ik_sols) > 0:
                valid_pose = ik_sols

        else:
            t = np.eye(4)
            t1 = self.sim_object.matrixFromPose([1,0,0,0,-0.085,0,0.24])
            r1 = self.sim_object.matrixFromAxisAngle([0,math.pi/2.0,0])
            r2 = self.sim_object.matrixFromAxisAngle([math.pi/2.0,0,0])

            grasp_T = world_T_obj.dot(t1).dot(r1).dot(r2)
            ik_sols = self.sim_object.robot.get_ik_solutions(grasp_T,collision_fn=self.sim_object.collision_check)
            if len(ik_sols) > 0:
                valid_pose = ik_sols
        
        return valid_pose

    def sample_goal_pose(self,object_name):            
        count = 0
        valid_ik = None
        
        while count < Config.MAX_IK_ATTEMPTS and valid_ik is None:
            sample_obj_pose = self.sim_object.get_obj("{}targetLoc_{}".format(object_name.split("_")[Config.OBJ_TYPE_IND],object_name.split("_")[Config.OBJ_ID_IND])).get_transform()     
            grasp_pose = self.sample_grasp_pose(object_name=object_name,pose=sample_obj_pose)
            if grasp_pose is not None:
                valid_ik = grasp_pose
                break
            count+=1

        return valid_ik

    def get_init_pose(self):
        return self.init_pose    

    def get_possible_configs(self,num_objects=1,configs_to_exclude=set([])):
        possible_configs=[]

        # for i in range(1,len(self.surface_list)+1):
        possible_configs = product([str(obj.get_name()) for obj in self.object_list],[str(surface.get_name()) for surface in self.surface_list])            
        possible_configs = list(set(possible_configs).difference(configs_to_exclude))
        
        np.random.shuffle(possible_configs)
        return possible_configs

    def randomize_env(self,given_surface_lists=None,req_relation=None,traj_count=0,traj_config=None,configs_to_exclude=set([])):
        self.sim_object.collision_set.add(self.sim_object.robot)
        # print(self.bound_object_name)
        for obj_name in self.bound_object_name:
            self.sim_object.collision_set.add(self.sim_object.get_obj(obj_name))

        t = np.eye(4)
        t[2,3] = 10
        for obj in self.added_objects:
            obj.set_transform(t.dot(obj.get_transform()))
        self.added_objects = []

        self.surface_randomizer()
        self.sim_object.robot.tuck_arm()
        self.sim_object.robot.activate_base_joints()
        random_place, _ = self.sim_object.robot.random_config_robot(collision_fn=self.sim_object.collision_check,use_collision_set=True)
        self.sim_object.robot.set_active_dof_values(random_place)

        objects_to_set = len(self.object_list)

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
            init_surface_type = "surface"
            
            sample_quadrant_range = getattr(self,"default_{}_range".format(init_surface_type))
            t = self.object_randomizer(obj_name=can_name,surface_name=init_surface_name,range=sample_quadrant_range)
            if t is None:
                possible_configs.append(init_config)
                continue
            else:
                self.sim_object.get_obj(can_name).set_transform(t)
                init_configs.append(init_config)

        traj_config = []
        for num, init_config in enumerate(init_configs):
            init_obj, init_surface = init_config
            obj_configs = [(obj_name, surface) for obj_name, surface in possible_configs if obj_name == init_obj]                    
            while len(obj_configs) > 0:
                _, goal_surface_name = goal_config = obj_configs.pop(0)
                goal_surface_id = int(goal_surface_name.split("_")[Config.OBJ_ID_IND])
                goal_surface_type = "surface"

                sample_quadrant_range = getattr(self,"default_{}_range".format(goal_surface_type))
                t = self.object_randomizer(obj_name=init_obj,surface_name=goal_surface_name,range=sample_quadrant_range,init_config=False)
                
                if t is not None:
                    traj_config.append((init_config, goal_config))
                    break
        
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

    def motion_plan_robot_base(self,object_name=None,given_pose=None):
        self.sim_object.robot.activate_base_joints()
        
        traj = None
        count=0
        while traj is None and count<5:
            target_pose = given_pose
            if given_pose is None:
                if object_name is not None:
                    target_pose = self.sample_robot_base(object_name=object_name,mp=True)
                else:
                    target_pose,_ = self.sim_object.robot.random_config_robot(collision_fn=self.sim_object.collision_check)

            if target_pose is not None:
                traj = [target_pose]
                if self.compute_mp:
                    traj = self.sim_object.compute_motion_plan(target_pose)
            
            count += 1
        
        return traj, target_pose
    
    def motion_plan_robot_arm(self,object_name=None,pose=[],grasp_num=None,given_pose=None,surface_id=None):
        self.sim_object.robot.activate_manip_joints()
        
        if object_name is not None:
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
        envlist = np.random.choice(list(range(1,self.n+1)),int(0.3*self.n))

        if self.sim_object.GetViewer() is not None:
            self.sim_object.set_camera_wrt_obj(Config.BOUND_OBJECT_NAME)
            
        while i < self.n:
            init_object_dic,traj_config = self.randomize_env(traj_count=i)
            while j <= self.j:
                state_list = []
                for obj in self.object_list:
                    #going to init_surface
                    # print(init_config[0])
                    object_name = str(obj.get_name())
                    targetLoc = self.sim_object.get_obj("{}targetLoc_{}".format(object_name.split("_")[Config.OBJ_TYPE_IND],object_name.split("_")[Config.OBJ_ID_IND]))
                    _target_loc_name = str(targetLoc.get_name())
                    initLoc = self.sim_object.get_obj("{}initLoc_{}".format(object_name.split("_")[Config.OBJ_TYPE_IND],object_name.split("_")[Config.OBJ_ID_IND]))
                    initLoc_name = str(initLoc.get_name())

                    init_config = (object_name,initLoc_name)
                    #KP_1
                    state_1 = [self.sim_object.get_one_state()]

                    # print("sampling surface 1")
                    base_sample_1 = self.sample_robot_base(object_name=initLoc_name)
                        
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
                    traj_2,_ = self.motion_plan_robot_arm(object_name=object_name)

                    if traj_2 is None:
                        if j == 0:
                            flag=1
                        else:
                            flag=2
                        break
                    
                    self.sim_object.robot.activate_base_joints()
                    self.sim_object.robot.set_active_dof_values(init_base_pose)
                    # print("motion planning for base to surface 1")
                    traj_1, base_sample_1 = self.motion_plan_robot_base(object_name=initLoc_name,given_pose=base_sample_1)
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
                        random_pose,_ = self.sim_object.robot.random_config_robot(current_dof=current_dof,collision_fn=self.sim_object.collision_check)
                        state_8 = self.sim_object.get_state_block(traj=[random_pose],config_tuple=init_config)

                    # print("sampling surface 2")
                    base_sampler = self.sample_robot_base
                    base_sample_2 = base_sampler(object_name=_target_loc_name)
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
                        ik = self.sample_goal_pose(object_name)
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
                    traj_4, base_sample_2 = self.motion_plan_robot_base(object_name=_target_loc_name,given_pose=base_sample_2)
                    if traj_4 is None:
                        if j == 0:
                            flag=1
                        else:
                            flag=2
                        break
                        
                    #KP_9
                    state_9 = self.sim_object.get_state_block(traj=traj_4,config_tuple=init_config)
                    #KP_10
                    state_10 = []
                    if not self.compute_mp:
                        self.sim_object.robot.activate_manip_joints()
                        current_dof = self.sim_object.robot.get_active_dof_values()
                        random_pose,_ = self.sim_object.robot.random_config_robot(current_dof=current_dof,collision_fn=self.sim_object.collision_check)
                        state_10 = self.sim_object.get_state_block(traj=[random_pose],config_tuple=init_config)
                    
                    #KP_11 and KP_12
                    state_11_12 = self.sim_object.get_state_block(traj=traj_5,config_tuple=init_config,grab=False)

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
                    state_15 = self.sim_object.get_state_block(traj=traj_7,config_tuple=init_config)
                    
                    states = [state_1,state_2,state_3,state_4_5,state_6,state_7,state_8,state_9,state_10,state_11_12,state_13,state_14,state_15]
                    if self.random != -1:
                        last_ind = np.random.randint(low=1,high=len(states))
                    else:
                        last_ind = len(states)
                    
                    for state_num in range(last_ind):
                        state_list.extend(states[state_num])
                    
                    # state_list.extend(state_1)
                    # state_list.extend(state_2)
                    # state_list.extend(state_3)
                    # state_list.extend(state_4_5)
                    # state_list.extend(state_6)
                    # state_list.extend(state_7)
                    # state_list.extend(state_8)
                    # state_list.extend(state_9)
                    # state_list.extend(state_10)
                    # state_list.extend(state_11_12)
                    # state_list.extend(state_13)
                    # state_list.extend(state_14)
                    # state_list.extend(state_15)

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

                # self.execute_traj(state_list)
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

    def object_randomizer(self,obj_name,surface_name,range,init_config=True):
        obj = self.sim_object.get_obj(obj_name)
        obj_type = obj_name.split("_")[Config.OBJ_TYPE_IND]
        current_t = obj.get_transform()

        surface_id = int(surface_name.split("_")[Config.OBJ_ID_IND])
        surface_type = "surface"
        
        sampler = getattr(self,"sample_obj_{}".format(surface_type))

        t = sampler(object_name=obj_name,surface_name=surface_name,range=range)
        if t is not None:
            if str(obj.get_name()).split("_")[Config.OBJ_TYPE_IND] in Config.OBJECT_NAME:
                self.added_objects.append(obj)
                if init_config:
                    Loc_obj = self.sim_object.get_obj("{}initLoc_{}".format(obj_name.split("_")[Config.OBJ_TYPE_IND],obj_name.split("_")[Config.OBJ_ID_IND]))
                    obj.set_transform(t)
                else:
                    Loc_obj = self.sim_object.get_obj("{}targetLoc_{}".format(obj_name.split("_")[Config.OBJ_TYPE_IND],obj_name.split("_")[Config.OBJ_ID_IND]))

                Loc_obj.set_transform(t)

            self.sim_object.collision_set.add(obj)
            
            obj.set_transform(current_t)
            return t
    
        else:
            obj.set_transform(current_t)
            return None
    
    def spawn_glass(self,name=""):
        radius = 0.0425
        height = 0.24
        color = [0,0.8,1]
        body_name = "glass"+ "_{}".format(self.spawned_object_count+1)

        t = self.sim_object.matrixFromPose([1, 0, 0, 0, 0, 0, -0.5])
        cylinder = Object(model_gen_utils.create_cylinder(self.sim_object.env, body_name, t, [radius, height], color))
  
        self.sim_object.add_obj(cylinder)
        self.spawned_object_count += 1

        self.object_list.append(cylinder)
        return t

    def spawn_surface(self,name="",id=None):
        thickness = 0.1
        legheight = 0.55
        color = [1,0.8,0]
        pose = [3, -5, 0.63]

        name = "surface_{}".format(len(self.surface_list)+1)        
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
        self.surface_list.append(table)

    def spawn_bowl(self,name=""):
        body_name = "bowl" + "_{}".format(self.spawned_object_count+1)

        t = self.sim_object.matrixFromPose([1, 0, 0, 0, 0, 0, -0.5])
        bowl = self.sim_object.load_stl_object("bowl", t)

        bowl.set_name(body_name)##

        self.sim_object.add_obj(bowl)
        self.sim_object.collision_set.add(bowl)

        self.spawned_object_count += 1
        self.object_list.append(bowl)
        return t
    
    def spawn_loc(self,obj):
        object_name = str(obj.get_name())
        _target_loc_obj = Object(model_gen_utils.create_cylinder(self.sim_object.env,"{}targetLoc_{}".format(object_name.split("_")[Config.OBJ_TYPE_IND],object_name.split("_")[Config.OBJ_ID_IND]),np.eye(4),[0,0]))
        initLoc_obj = Object(model_gen_utils.create_cylinder(self.sim_object.env,"{}initLoc_{}".format(object_name.split("_")[Config.OBJ_TYPE_IND],object_name.split("_")[Config.OBJ_ID_IND]),np.eye(4),[0,0]))
        self.sim_object.add_obj(_target_loc_obj)
        self.sim_object.add_obj(initLoc_obj)
        return 1

    def setup_exp(self,given_surface_lists=None,req_relation=None,experiment_flag=False):
        if not self.experiment_flag:
            self.randomize_env(given_surface_lists)
        init_state = self.sim_object.get_one_state()

        for obj in self.object_list:
            obj.set_transform(self.sim_object.get_obj("{}targetLoc_{}".format(obj.get_name().split("_")[Config.OBJ_TYPE_IND],obj.get_name().split("_")[Config.OBJ_ID_IND])).get_transform())

        goal_state = self.sim_object.get_one_state()

        return init_state,goal_state,None