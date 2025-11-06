import os
import sys
import pickle
from copy import deepcopy
import argparse
from itertools import product
import numpy as np
import tqdm
import time
import importlib
from Config import Config
import useful_functions

SimClass = Config.get_simulator_module("SimClass").SimClass
Object = Config.get_simulator_module("Object").Object
model_gen_utils = Config.get_simulator_module("model_gen_utils")

class Packing(object):
    def __init__(self,env_name,number_of_configs=1,number_of_mp=1,axis_for_offset="x",file_name="_data",reference_structure_name=None,visualize=False,object_count=None,random=False,order=False,surface="",objects_in_init_state=0,quadrant=None,grasp_num=None,minimum_object_count=None,mp=True,num_robots=1,structure_dependance=False,object_list=[],experiment_flag=False,real_world_experiment=False,set_y=False,complete_random=False,data_gen=False,robot_name="MagneticGripper"):
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

        self.sim_object.load_env(env_name + ".dae")

        self.bound_object_name = ['table6']
        drop = self.sim_object.get_obj("drop_area")
        if drop is not None:
            self.drop_name = "surface_1"
            dropbox = Object(model_gen_utils.create_dropbox(self.sim_object.env,init_trans=drop.get_transform()))
            self.sim_object.remove_obj(drop)
            dropbox.set_name(self.drop_name)
            self.sim_object.add_obj(dropbox)
            self.sim_object.collision_set = set([self.sim_object.robot,dropbox])
        else:
            self.sim_object.collision_set = set([self.sim_object.robot])

        spawn = self.sim_object.get_obj("spawn_area")
        if spawn is not None:
            self.sim_object.remove_obj(spawn)
            
        for obj_name in self.bound_object_name:
            self.sim_object.collision_set.add(self.sim_object.get_obj(obj_name))

        self.clearance = 0.001
        self.obj_h = 0.05
        self.droparea_dims = 0.075
        self.sim_object.robot_dims = self.sim_object.get_object_dims(self.sim_object.robot.name)
        self.can_radius = 0.0325
        self.can_h = 0.15
        self.sim_object.robot.obj_offset = 2*self.can_radius
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

        self.object_count = object_count
        if minimum_object_count is None:
            self.minimum_object_count = self.object_count
        else:
            self.minimum_object_count = minimum_object_count

        self.objects_in_init_state = objects_in_init_state
        self.file_name = file_name

        self.random = random
        self.added_objects = []
        self.surface = surface
        self.num_robots = num_robots

        if "problem" not in env_name:
            for i in range(self.object_count):
                # self.spawn_goalLoc()
                self.spawn_object()
            
        self.randomize_env()
        self.trace = []
        self.compute_mp = mp

        if not self.experiment_flag:
            self.setup_base_env()

    def sample_grasp_pose(self,object_name="",pose = []):
        if pose != []:
            world_T_obj = pose
        else:
            world_T_obj = self.sim_object.get_obj(object_name).get_transform()
            
        world_T_robot = useful_functions.transform_from_sixd_pose(self.sim_object.robot.get_active_dof_values())
        robot_T_world = np.linalg.inv(world_T_robot)

        obj_T_robot = np.eye(4)
        obj_T_robot[2,3]= self.sim_object.robot.grasping_offset[Config.OBJECT_NAME[0]]
        
        t1 = useful_functions.rotmat_from_rotvec([0, 0, -np.pi/4.0])
        # t2 = useful_functions.rotmat_from_rotvec([-np.pi, 0, 0])

        # obj_T_robot = obj_T_robot.dot(t1).dot(t2)
        obj_T_robot = obj_T_robot.dot(t1)
        t = np.matmul(world_T_obj,obj_T_robot)
        pose = useful_functions.sixd_pose_from_transform(t)
        
        return pose
    
    def sample_goal_pose(self,object_name):
        grabbed_obj = None
        if self.sim_object.robot.grabbed_flag_1:
            if self.sim_object.robot.grabbed_object_1 == str(object_name):
                grabbed_obj = object_name
                self.sim_object.robot.release()

        obj = self.sim_object.get_obj(object_name)
        t_current = obj.get_transform()
        obj_dims = self.sim_object.get_object_dims(object_name=object_name)
        
        drop = self.sim_object.get_obj(self.drop_name)
        drop_t = drop.get_transform()

        x_edge_offset = abs(self.droparea_dims-obj_dims[0])
        y_edge_offset = abs(self.droparea_dims-obj_dims[1])

        while True:
            x = np.random.uniform(low=-x_edge_offset,high=x_edge_offset)
            y = np.random.uniform(low=-y_edge_offset,high=y_edge_offset)
            z = 0.001

            grasp_num = np.random.choice(Config.NUM_GRASPS)
            rot_angle = (grasp_num * (2*np.pi) / Config.NUM_GRASPS)
            rot_z = useful_functions.rotmat_from_rotvec([0,0,rot_angle])

            t = useful_functions.transform_from_sevend_pose([1,0,0,0,x,y,z])#.dot(rot_z)
            t = drop_t.dot(t)
            obj.set_transform(t)
            if not(self.sim_object.collision_check([obj]) and self.obj_checker(obj)):
                break
            
            obj.set_transform(t_current)
            if grabbed_obj is not None:
                self.sim_object.robot.grab(obj=object_name)
            # t = self.sample_goal_pose(object_name=object_name)
        
        obj.set_transform(t_current)
        if grabbed_obj is not None:
            self.sim_object.robot.grab(obj=object_name)
        return t   

    def object_randomizer(self,plank,x_offsets=[0.1,0.1],y_offsets=[0.1,0.1]):
        drop_t = self.sim_object.get_obj(self.drop_name).get_transform()
        drop_h = deepcopy(drop_t[2,3])
        drop_t[2,3] = 0.05
        self.sim_object.get_obj(self.drop_name).set_transform(drop_t)
        
        while True:
            t = self.sim_object.get_random_pose(obj = plank,
                                                range_x = [self.env_x_limits[0]+x_offsets[0],self.env_x_limits[1]-x_offsets[1]],
                                                range_y = [self.env_y_limits[0]+y_offsets[0],self.env_y_limits[1]-y_offsets[1]],
                                                range_z = [drop_h+0.001,drop_h+0.001]
                                                )
            plank.set_transform(t)
            if not self.obj_checker(plank):
                break
            else:
                plank.set_transform(current_t)
        
        if plank.get_name().split("_")[Config.OBJ_TYPE_IND] in Config.OBJECT_NAME and plank not in self.can_list:
            self.can_list.append(plank)

        drop_t[2,3] = 0.0
        self.sim_object.get_obj(self.drop_name).set_transform(drop_t)

        return plank.get_transform()
    
    def drop_randomizer(self,x_offsets=[0.1,0.1],y_offsets=[0.1,0.1]):
        drop = self.sim_object.get_obj(self.drop_name)
        if drop in self.sim_object.collision_set:
            self.sim_object.collision_set.remove(drop)
        
        t = drop.get_transform()
        while True:
            t[0,3] = np.random.uniform(low = self.env_x_limits[0]+self.droparea_dims+x_offsets[0], high = self.env_x_limits[1]-self.droparea_dims-x_offsets[1])
            t[1,3] = np.random.uniform(low = self.env_y_limits[0]+self.droparea_dims+y_offsets[0], high = self.env_y_limits[1]-self.droparea_dims-y_offsets[1])
            t[2,3] = 0.05

            drop.set_transform(t)
            if not (self.sim_object.collision_check([drop])):
                break

        t = self.sim_object.get_random_pose(obj = drop,
                                 range_x = [self.env_x_limits[0]+self.droparea_dims+x_offsets[0],self.env_x_limits[1]-self.droparea_dims-x_offsets[1]],
                                 range_y = [self.env_y_limits[0]+self.droparea_dims+y_offsets[0],self.env_y_limits[1]-self.droparea_dims-y_offsets[1]],
                                 range_z = [0.05,0.05],
                                )
        drop.set_transform(t)
        
        self.sim_object.collision_set.add(drop)

        t[2,3] = 0.0
        drop.set_transform(t)

        return 1  
    
    def setup_base_env(self):
        if self.object_count > 6:
            if self.sim_object.robot.name != "yumi":
                self.setup_bigger_base()
            x_offsets = [1.0,0.2]
            y_offsets = [1.4,0.2]
        else:
            x_offsets = [0.1,0.1]
            y_offsets = [0.1,0.1]

        self.randomize_env(x_offsets,y_offsets)

        return True
        
    def setup_bigger_base(self):
        for obj_name in self.bound_object_name:
            obj = self.sim_object.get_obj(obj_name)
            self.bound_object_name.remove(obj_name)
            self.sim_object.collision_set.remove(obj)
            self.sim_object.remove_obj(obj)

        dims = [1,1,0.25]
        new_bound_object = Object(model_gen_utils.create_box(self.sim_object.env,'base',np.eye(4),dims,[0.75,0.5,0]))
        self.sim_object.collision_set.add(new_bound_object)
        self.bound_object_name.append("base")
        self.sim_object.add_obj(new_bound_object)
        t = np.eye(4)
        t[2,3] = -0.38
        new_bound_object.set_transform(t)

        self.env_limits = self.sim_object.get_env_limits(self.bound_object_name)
        self.env_y_limits = self.env_limits[1] #[y_min,y_max]
        self.env_x_limits = self.env_limits[0] #[x_min,x_max]
        self.env_z_limits = [self.env_limits[-1],1.0]

        return True
    
    # @SimClass.env_lock
    def setup_exp(self,arg=None,req_relation=None,experiment_flag=False):
        if not experiment_flag:
            droparea = self.sim_object.get_obj(self.drop_name)
            self.drop_randomizer(x_offsets=[0.1,0.1],y_offsets=[0.1,0.1])
            drop_t = droparea.get_transform()
            drop_pose = useful_functions.sixd_pose_from_transform(drop_t)
            rcr = [r.region for r in req_relation]
            discretizer = req_relation[0].discretizer
            for obj in self.can_list:
                sample_flag = True
                while sample_flag:
                    t = self.sample_goal_pose(object_name=obj.get_name())
                    object_pose = useful_functions.sixd_pose_from_transform(t)
                    relative_1 = useful_functions.get_relative_pose(pose1=drop_pose,pose2=object_pose)
                    relative_2 = useful_functions.get_relative_pose(pose1=object_pose,pose2=drop_pose)

                    discretized_1 = discretizer.get_discretized_pose(input_pose=relative_1,is_relative=True)
                    discretized_2 = discretizer.get_discretized_pose(input_pose=relative_2,is_relative=True)
                    discretized_1.extend([0]*Config.SENSOR_COUNT)
                    discretized_2.extend([0]*Config.SENSOR_COUNT)

                    for samples in rcr:
                        region = [r for r in samples]
                        if discretized_1 in region or discretized_2 in region:
                            sample_flag = False
                            obj.set_transform(t)
                            break
            
            goal_state = self.sim_object.get_one_state()
            
            planks_to_place = len(self.can_list) - self.objects_in_init_state
            for plank in self.can_list[::-1][:planks_to_place]:
                x_offsets=[0.01,0.01]
                y_offsets=[0.01,0.01]

                self.object_randomizer(plank,x_offsets,y_offsets)
            
            init_state = self.sim_object.get_one_state()

        return init_state, goal_state, None   
    
    def obj_checker(self,obj):
        t_y = obj.get_transform()[1,3]
        t_x = obj.get_transform()[0,3]
        for pl in self.can_list:
            if obj!=pl:
                p_y = pl.get_transform()[1,3]
                p_x = pl.get_transform()[0,3]
                if abs(p_y-t_y) < (self.sim_object.robot.obj_offset) and abs(p_x-t_x) < (self.sim_object.robot.obj_offset):
                    return True
        
        return False

    def spawn_object(self):
        radius = self.can_radius
        height = self.can_h
        color = [0,0.8,1]
        body_name = Config.OBJECT_NAME[0] + "_{}".format(len(self.can_list)+1)

        t = useful_functions.transform_from_sevend_pose([1, 0, 0, 0, 0, 0, -0.5])
        cylinder = Object(model_gen_utils.create_cylinder(self.sim_object.env, body_name, t, [radius, height], color))

        self.sim_object.add_obj(cylinder)
        self.can_list.append(cylinder)
        
        return t
    
    def randomize_cans(self,x_offsets=[0.1,0.1],y_offsets=[0.1,0.1]):
        self.sim_object.robot.release()
        can_transform_link = {}
        to_remove_list = []
        for obj in self.sim_object.collision_set:
            if obj.get_name().split("_")[Config.OBJ_TYPE_IND] in Config.OBJECT_NAME:
                to_remove_list.append(obj)

        for obj in to_remove_list:
            self.sim_object.collision_set.remove(obj)

        for can in self.can_list:            
            t = self.object_randomizer(can,x_offsets,y_offsets)
            can_transform_link[can] = t
            self.sim_object.collision_set.add(can)

        return can_transform_link
    
    def randomize_env(self,x_offsets=[0.1,0.1],y_offsets=[0.1,0.1],traj_config=None):
        transform_dict = self.randomize_cans(x_offsets,y_offsets)
        random_place, _ = self.sim_object.robot.random_config_robot(collision_fn=self.sim_object.collision_check)
        self.sim_object.robot.set_active_dof_values(random_place)
        self.sim_object.robot.get_init_pose()
        self.drop_randomizer(x_offsets,y_offsets)

        if not(self.collision):
            t = self.sim_object.get_obj(self.drop_name).get_transform()
            t[2,3] = 0.0
            self.sim_object.get_obj(self.drop_name).set_transform(t)
        
        for obj in self.bound_object_name:
            self.sim_object.collision_set.add(self.sim_object.get_obj(obj))   

        return transform_dict
    
    def reset_planks(self,tranform_dict):
        self.sim_object.robot.release()
        for p in tranform_dict.keys():
            p.set_transform(tranform_dict[p])
    
        return True
    
    @SimClass.conditional_env_lock
    def start(self,complete_random=False):
        i = 0        
        flag = 0
        j = 0
        pbar = tqdm.tqdm(total=self.n)

        if self.sim_object.GetViewer() is not None:
            self.sim_object.set_camera_wrt_obj("table6",2)
        while i < self.n:
            transform_dict = self.randomize_env()
            planks_to_set=self.object_count
            if self.minimum_object_count != self.object_count:
                planks_to_set = np.random.randint(low=self.minimum_object_count,high=self.object_count+1)

            goalLoc_list = self.can_list[:planks_to_set]
            while j <= self.j:
                state_list = []
                self.sim_object.robot.set_init_pose()
                self.reset_planks(transform_dict)

                for goalLoc in goalLoc_list:
                    traj_1 = None
                    traj_2 = None
                    traj_3 = None

                    grabbed_flag = getattr(self.sim_object.robot,"grabbed_flag_{}".format(self.sim_object.robot.id))
                    init_state =  [self.sim_object.get_one_state()]

                    object_count = int(str(goalLoc.get_name()).split("_")[-1])-1
                    #reaching plank
                    counter = 0
                    while traj_1 is None:
                        gp = self.sample_grasp_pose(self.can_list[object_count].get_name())
                        if self.compute_mp:
                            traj_1 = self.sim_object.compute_motion_plan(gp)
                        else:
                            traj_1 = [gp]

                        counter+=1
                        if (counter == 10 and j < 1):
                            flag = 1
                            break
                        elif counter >= 30 and traj_1 is None:
                            flag=2
                            break
                        
                    if flag>0:
                        break
                    
                    time.sleep(0.001)
                    # self.execute_traj(traj_1)
                    self.sim_object.set_to_last_waypoint(traj_1)
                    grabbed_flag = getattr(self.sim_object.robot,"grabbed_flag_{}".format(self.sim_object.robot.id))
                    state1 = self.sim_object.get_state_list(traj_1)
                    self.sim_object.robot.grab(self.can_list[object_count].get_name())
                    grabbed_flag = getattr(self.sim_object.robot,"grabbed_flag_{}".format(self.sim_object.robot.id))
                    one_state = self.sim_object.get_one_state()
                    state1.append(one_state)

                    #taking plank to droparea
                    counter = 0
                    while traj_2 is None:
                        goal_t = self.sample_goal_pose(object_name=str(goalLoc.get_name()))
                        ep = self.sample_grasp_pose(pose=goal_t)
                        if self.compute_mp:
                            traj_2 = self.sim_object.compute_motion_plan(ep)
                        else:
                            traj_2 = [ep]

                        counter+=1
                        if (counter == 10 and j < 1) or (counter == 15 and traj_2 is None):
                            flag = 1
                            break
                        elif counter >= 30 and traj_2 is None:
                            flag=2
                            break
                        
                    if flag>0:
                        break

                    time.sleep(0.001)
                    # self.execute_traj(traj_2)
                    self.sim_object.set_to_last_waypoint(traj_2)
                    self.sim_object.collision_set.add(self.can_list[object_count])
                    grabbed_flag = getattr(self.sim_object.robot,"grabbed_flag_{}".format(self.sim_object.robot.id))
                    state2 = self.sim_object.get_state_list(traj_2)
                    self.sim_object.robot.release()
                    grabbed_flag = getattr(self.sim_object.robot,"grabbed_flag_{}".format(self.sim_object.robot.id))
                    one_state = self.sim_object.get_one_state()
                    state2.append(one_state)

                    #random_placing_robot
                    while traj_3 is None:
                        ep,_ = self.sim_object.robot.random_config_robot(current_dof=self.sim_object.robot.get_active_dof_values(),collision_fn=self.sim_object.collision_check)
                        if self.compute_mp:
                            traj_3 = self.sim_object.compute_motion_plan(ep)
                        else:
                            traj_3 = [ep]
                        counter+=1
                        if (counter == 10 and j < 1) or (counter == 15 and traj_3 is None):
                            flag = 1
                            break
                        elif counter >= 30 and traj_3 is None:
                            flag=2
                            break
                        
                    if flag>0:
                        break

                    time.sleep(0.001)
                    # self.execute_traj(traj_3)
                    self.sim_object.set_to_last_waypoint(traj_3)
                    grabbed_flag = getattr(self.sim_object.robot,"grabbed_flag_{}".format(self.sim_object.robot.id))
                    state3 = self.sim_object.get_state_list(traj_3)

                    states = [init_state,state1,state2,state3]
                    if self.random != -1:
                        last_ind = np.random.randint(low=1,high=len(states))
                    else:
                        last_ind = len(states)
                    
                    for state_num in range(last_ind):
                        state_list.extend(states[state_num])
                    
                    # state_list.extend(init_state)
                    # state_list.extend(state1)
                    # state_list.extend(state2)
                    # state_list.extend(state3)

                if flag == 1:
                    break
                elif flag == 2:
                    flag = 0
                    continue
                
                for obj in self.sim_object.get_objects():
                    self.object_names.add(str(obj.get_name()))

                self.data.append(state_list)

                j += 1
                if j == self.j:
                    break
            
            j=0
            if flag == 1:
                flag = 0
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

        pickle.dump(final_data,open(Config.DATA_MISC_DIR+ self.env_name+"/"+ self.file_name+Config.PICKLE_SUFFIX ,"wb"),protocol=Config.PICKLE_PROTOCOL )
        pickle.dump(object_data,open(Config.DATA_MISC_DIR+ self.env_name+"/"+ self.env_name + "_object_list"+Config.PICKLE_SUFFIX ,"wb"),protocol=Config.PICKLE_PROTOCOL )
                
        print("{} data saved".format(self.env_name))