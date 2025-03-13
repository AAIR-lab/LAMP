from openravepy.misc import DrawAxes
import openravepy as orpy
import Config
import numpy as np
import cPickle
import tqdm
import time
import utils
from openravepy import *
from src.data_structures.EnvState import EnvState
from prpy.planning.base import PlanningError
from prpy.planning import ompl, BiRRTPlanner
import random
from copy import deepcopy
from Environments.Keva import object_models
from src.robot_files.Robots import Fetch, MagicGripper
import math
import importlib
from src.useful_functions import get_relative_transform
import os

def GetDomain(robot_name="MagneticGripper"):
    mod = importlib.import_module("src.robot_files.Robots")
    RobotClass = getattr(mod,robot_name)

    class Packing(RobotClass):
        def __init__(self,env_name,number_of_configs,number_of_mp,axis_for_offset,file_name="_data.p",reference_structure_name=None,visualize = False,plank_count=None, random=False,order=False,surface="",planks_in_init_state=0,quadrant=None,grasp_num=None,minimum_plank_count=None,mp=True,num_robots=1,structure_dependance=False,object_list=[],experiment_flag=False,real_world_experiment=False,set_y=False,complete_random=False,data_gen=False):
            self.env = orpy.Environment()
            self.env.SetDebugLevel(0)
            super(Packing,self).__init__(env=self.env,id=1)

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
            self.env.Load(Config.ENV_DIR + env_name + ".dae")

            self.bound_object_name = ['table6']
            drop = self.env.GetKinBody("drop_area")
            if drop is not None:
                self.drop_name = Config.SURFACE_NAME
                if "box" not in Config.SURFACE_NAME:
                    self.drop_name = Config.SURFACE_NAME + "_1"
                dropbox = object_models.create_dropbox(self.env,init_trans=drop.GetTransform())
                self.env.Add(dropbox)
                self.env.Remove(drop)
                dropbox.SetName(self.drop_name)
                self.collision_set = set([self.robot,drop])
            else:
                self.collision_set = set([self.robot])

            spawn = self.env.GetKinBody("spawn_area")
            if spawn is not None:
                spawn.SetName("spawnarea")
                self.env.Remove(spawn)
            for obj_name in self.bound_object_name:
                self.collision_set.add(self.env.GetKinBody(obj_name))

            self.clearance = 0.001
            self.obj_h = 0.05
            self.droparea_dims = 0.075
            self.robot_dims = self.get_object_dims(self.robot_name)
            self.can_radius = 0.0325
            self.can_h = 0.15
            self.robot_offset = 2*self.can_radius
            self.env_limits = self.get_env_limits()
            self.env_y_limits = self.env_limits[1] #[y_min,y_max]
            self.env_x_limits = self.env_limits[0] #[x_min,x_max]
            self.env_z_limits = [self.env_limits[-1],1.0]

            self.collision = False
            self.grabbed_flag_1 = False
            self.can_list = []
            self.table_list = []
            self.goalLoc_list = []
            self.quadrant = quadrant
            self.grasp_num = grasp_num

            self.object_count = plank_count
            if minimum_plank_count is None:
                self.minimum_object_count = self.object_count
            else:
                self.minimum_object_count = minimum_plank_count

            self.objects_in_init_state = planks_in_init_state
            cc = orpy.RaveCreateCollisionChecker(self.env,'pqp')
            cc.SetCollisionOptions(orpy.CollisionOptions.AllGeometryContacts)
            self.env.SetCollisionChecker(cc)
            self.file_name = file_name
            self.motion_planner = BiRRTPlanner()

            self.random = random
            self.added_objects = []
            self.surface = surface
            self.num_robots = num_robots
            self.robot_1 = self
            self.robots = [self.robot_1]

            if visualize:
                self.env.SetViewer('qtcoin')

            if "problem" not in env_name:
                for i in range(self.object_count):
                    # self.spawn_goalLoc()
                    self.spawn_object()
                
            self.randomize_env()
            self.trace = []
            self.compute_mp = mp

        def get_relative_pose(self, pose1, pose2):
            #obj2 w.r.t. obj1
            transform1 = self.transform_from_wp(pose1)
            transform2 = self.transform_from_wp(pose2)
            return self.get_pose_from_transform((np.linalg.pinv(transform1).dot(transform2)))
        
        def sample_grasp_pose(self,object_name="",pose = []):
            if pose != []:
                world_T_obj = pose
            else:
                world_T_obj = self.env.GetKinBody(object_name).GetTransform()
                
            world_T_robot = self.transform_from_wp(self.robot.GetActiveDOFValues())
            robot_T_world = np.linalg.inv(world_T_robot)

            obj_T_robot = np.eye(4)
            obj_T_robot[2,3]= self.grasping_offset[Config.OBJECT_NAME[0]]
            
            t1 = orpy.matrixFromAxisAngle([0, 0, -np.pi/4.0])
            # t2 = orpy.matrixFromAxisAngle([-np.pi, 0, 0])

            # obj_T_robot = obj_T_robot.dot(t1).dot(t2)
            obj_T_robot = obj_T_robot.dot(t1)
            t = np.matmul(world_T_obj,obj_T_robot)
            pose = self.get_pose_from_transform(t)
            
            return pose
        
        def sample_goal_pose(self,object_name):
            grabbed_obj = None
            if self.grabbed_flag_1:
                if self.grabbed_object_1 == str(object_name):
                    grabbed_obj = object_name
                    self.release()

            obj = self.env.GetKinBody(object_name)
            t_current = obj.GetTransform()
            obj_dims = self.get_object_dims(object_name=object_name)
            
            drop = self.env.GetKinBody(self.drop_name)
            drop_t = drop.GetTransform()

            x_edge_offset = abs(self.droparea_dims-obj_dims[0])
            y_edge_offset = abs(self.droparea_dims-obj_dims[1])

            while True:
                x = np.random.uniform(low=-x_edge_offset,high=x_edge_offset)
                y = np.random.uniform(low=-y_edge_offset,high=y_edge_offset)
                z = 0.001

                grasp_num = np.random.choice(Config.NUM_GRASPS)
                rot_angle = (grasp_num * (2*np.pi) / Config.NUM_GRASPS)
                rot_z = matrixFromAxisAngle([0,0,rot_angle])

                t = matrixFromPose([1,0,0,0,x,y,z])#.dot(rot_z)
                t = drop_t.dot(t)
                obj.SetTransform(t)
                if not(self.collision_check([obj]) and self.obj_checker(obj)):
                    break
                
                obj.SetTransform(t_current)
                if grabbed_obj is not None:
                    self.grab(obj=object_name)
                # t = self.sample_goal_pose(object_name=object_name)
            
            obj.SetTransform(t_current)
            if grabbed_obj is not None:
                self.grab(obj=object_name)
            return t
        
        def store_init_config(self):
            bodies = self.env.GetBodies()
            for body in bodies:
                if body.GetName() != self.robot_name:
                    self.init_config[body.GetName()] = body.GetTransform()
                else:
                    for link in self.robot_type_object_mappings:
                        self.init_config[self.robot_type_object_mappings[link]] = self.robot.GetLink(link).GetTransform()

        def execute_traj(self, traj):
            num = traj.GetNumWaypoints()
            for waypoint in range(num):
                self.robot.SetActiveDOFValues(traj.GetWaypoint(waypoint))
                time.sleep(0.01)

        def set_to_last_waypoint(self,trajectory):
            if type(trajectory) != list:
                num = trajectory.GetNumWaypoints()
                last_wp = trajectory.GetWaypoint(num-1)
            else:
                last_wp = trajectory[0]

            if len(last_wp) == 3:
                self.activate_base_joints()
            else:
                self.activate_manip_joints()

            self.robot.SetActiveDOFValues(last_wp)

        def object_randomizer(self,plank,x_offsets=[0.1,0.1],y_offsets=[0.1,0.1]):
            drop_t = self.env.GetKinBody(self.drop_name).GetTransform()
            drop_h = deepcopy(drop_t[2,3])
            drop_t[2,3] = 0.05
            self.env.GetKinBody(self.drop_name).SetTransform(drop_t)

            while True:
                t = np.eye(4)
                t[0,3] = np.random.uniform(low = self.env_x_limits[0]+x_offsets[0], high = self.env_x_limits[1]-x_offsets[1])
                t[1,3] = np.random.uniform(low = self.env_y_limits[0]+y_offsets[0], high = self.env_y_limits[1]-y_offsets[1])

                if str(plank.GetName()).split("_")[0] in Config.OBJECT_NAME:
                    t[2,3] = drop_h+0.001

                plank.SetTransform(t)
                if not (self.collision_check([plank]) or self.obj_checker(plank)):
                    break

            if str(plank.GetName()).split("_")[0] in Config.OBJECT_NAME and plank not in self.can_list:
                self.can_list.append(plank)

            drop_t[2,3] = 0.0
            self.env.GetKinBody(self.drop_name).SetTransform(drop_t)

            return plank.GetTransform()
        
        def drop_randomizer(self,x_offsets=[0.1,0.1],y_offsets=[0.1,0.1]):
            drop = self.env.GetKinBody(self.drop_name)
            if drop in self.collision_set:
                self.collision_set.remove(drop)
            
            t = drop.GetTransform()
            while True:
                t[0,3] = np.random.uniform(low = self.env_x_limits[0]+self.droparea_dims+x_offsets[0], high = self.env_x_limits[1]-self.droparea_dims-x_offsets[1])
                t[1,3] = np.random.uniform(low = self.env_y_limits[0]+self.droparea_dims+y_offsets[0], high = self.env_y_limits[1]-self.droparea_dims-y_offsets[1])
                t[2,3] = 0.05

                drop.SetTransform(t)
                if not (self.collision_check([drop])):
                    break
            
            self.collision_set.add(drop)

            t[2,3] = 0.0
            drop.SetTransform(t)
 
            return 1
        
        def get_object_dims(self,object_name,use_ref=False):
            if use_ref:
                obj = self.reference_env.GetKinBody(object_name)
            else:            
                obj = self.env.GetKinBody(object_name)
            limits = utils.get_object_limits(obj)
            obj_dim = [abs(limits[1]-limits[0])/2.0,abs(limits[3]-limits[2])/2.0,limits[-1]]
            return obj_dim

        def get_env_limits(self):
            table_1 = self.env.GetKinBody(self.bound_object_name[0])
            table_1_limits = utils.get_object_limits(table_1)

            env_x = list(table_1_limits[:2])
            env_y = list(table_1_limits[2:-1])

            return [[min(env_x),max(env_x)],[min(env_y),max(env_y)],table_1_limits[-1]]
        
        def setup_base_env(self):
            if self.object_count > 6:
                if self.robot_name != "yumi":
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
                obj = self.env.GetKinBody(obj_name)
                self.bound_object_name.remove(obj_name)
                self.collision_set.remove(obj)
                self.env.Remove(obj)

            dims = [1,1,0.25]
            new_bound_object = object_models.create_box(self.env,'base',np.eye(4),dims,[0.75,0.5,0])
            self.collision_set.add(new_bound_object)
            self.bound_object_name.append("base")
            self.env.Add(new_bound_object)
            t = np.eye(4)
            t[2,3] = -0.38
            new_bound_object.SetTransform(t)

            self.env_limits = self.get_env_limits()
            self.env_y_limits = self.env_limits[1] #[y_min,y_max]
            self.env_x_limits = self.env_limits[0] #[x_min,x_max]
            self.env_z_limits = [self.env_limits[-1],1.0]

            return True
        
        def setup_exp(self,arg=None,req_relation=None,experiment_flag=False):
            if not experiment_flag:
                with self.env:
                # if True:
                    droparea = self.env.GetKinBody(self.drop_name)
                    self.drop_randomizer(x_offsets=[0.1,0.1],y_offsets=[0.1,0.1])
                    drop_t = droparea.GetTransform()
                    drop_pose = self.get_pose_from_transform(drop_t)
                    rcr = [r.region for r in req_relation]
                    discretizer = req_relation[0].discretizer
                    for obj in self.can_list:
                        sample_flag = True
                        while sample_flag:
                            t = self.sample_goal_pose(object_name=str(obj.GetName()))
                            object_pose = self.get_pose_from_transform(t)
                            relative_1 = self.get_relative_pose(pose1=drop_pose,pose2=object_pose)
                            relative_2 = self.get_relative_pose(pose1=object_pose,pose2=drop_pose)

                            discretized_1 = discretizer.get_discretized_pose(input_pose=relative_1,is_relative=True)
                            discretized_2 = discretizer.get_discretized_pose(input_pose=relative_2,is_relative=True)
                            discretized_1.append(0)
                            discretized_2.append(0)

                            for region in rcr:
                                if discretized_1 in region or discretized_2 in region:
                                    sample_flag = False
                                    obj.SetTransform(t)
                                    break
                    
                    goal_state = self.get_one_state()
                    
                    planks_to_place = len(self.can_list) - self.objects_in_init_state
                    for plank in self.can_list[::-1][:planks_to_place]:
                        x_offsets=[0.01,0.01]
                        y_offsets=[0.01,0.01]

                        self.object_randomizer(plank,x_offsets,y_offsets)
                    
                    init_state = self.get_one_state()

                return init_state, goal_state, None
        
        def collision_check(self,obj_list):
            with self.env:
            # while True:
                collision = False
                for obj in obj_list:
                    if type(obj) == str:
                        if obj in self.robot_type_object_mappings.values():
                            obj = self.robot_1.robot
                        else:
                            obj = self.env.GetKinBody(obj)
                            
                    for c_obj in self.collision_set:
                        grabbed_objects = []
                        for i in range(1,self.num_robots+1):
                            for rob in self.robots:
                                if rob.id == i:
                                    break
                            grabbed_objects.append(getattr(rob,"grabbed_object_{}".format(i)))
                        if c_obj == self.robot and str(obj.GetName()) in grabbed_objects:
                            collision_flag = False
                        else:
                            collision_flag = self.env.CheckCollision(obj,c_obj)
                        collision = collision_flag and obj.GetName() != c_obj.GetName() and str(c_obj.GetName()) not in grabbed_objects
                        if collision:
                            return collision

            self.collision = collision
            return collision
        
        def obj_checker(self,obj):
            t_y = obj.GetTransform()[1,3]
            t_x = obj.GetTransform()[0,3]
            for pl in self.can_list:
                if obj!=pl:
                    p_y = pl.GetTransform()[1,3]
                    p_x = pl.GetTransform()[0,3]
                    if abs(p_y-t_y) < (self.robot_offset) and abs(p_x-t_x) < (self.robot_offset):
                        return True
            
            return False

        def spawn_object(self):
            radius = self.can_radius
            height = self.can_h
            color = [0,0.8,1]
            body_name = Config.OBJECT_NAME[0] + "_{}".format(len(self.can_list)+1)

            t = matrixFromPose([1, 0, 0, 0, 0, 0, -0.5])
            cylinder = object_models.create_cylinder(self.env, body_name, t, [radius, height], color)

            self.env.Add(cylinder)
            self.can_list.append(cylinder)
            return t
        
        def random_place_robot(self):
            random_dof_values = []
            if self.robot_name != "yumi":
                t = self.robot.GetTransform()
                wp = self.get_pose_from_transform(t)

                random_x = np.random.uniform(self.env_x_limits[0],self.env_x_limits[1])
                random_y = np.random.uniform(self.env_y_limits[0],self.env_y_limits[1])
                random_z = np.random.uniform(self.env_z_limits[0],self.env_z_limits[1])
                random_roll  = np.random.uniform(-3.14, 3.14)
                random_pitch = np.random.uniform(-3.14, 3.14)
                random_yaw   = np.random.uniform(-3.14, 3.14)
                
                self.init_pose = [random_x,random_y,random_z,random_roll,random_pitch,random_yaw]
                random_dof_values = [random_x,random_y,random_z,random_roll,random_pitch,random_yaw]
                self.robot.SetActiveDOFValues([random_x,random_y,random_z,random_roll,random_pitch,random_yaw])

                if self.collision_check([self.robot]):
                    self.random_place_robot()
            else:
                for arm in self.manipulator_groups:
                    self.tuck_arm(arm)

            return random_dof_values
        
        def randomize_cans(self,x_offsets=[0.1,0.1],y_offsets=[0.1,0.1]):
            self.release()
            can_transform_link = {}
            to_remove_list = []
            for obj in self.collision_set:
                if obj.GetName().split("_")[0] in Config.OBJECT_NAME:
                    to_remove_list.append(obj)

            for obj in to_remove_list:
                self.collision_set.remove(obj)

            for can in self.can_list:            
                t = self.object_randomizer(can,x_offsets,y_offsets)
                can_transform_link[can] = t
                self.collision_set.add(can)

            return can_transform_link
        
        def randomize_env(self,x_offsets=[0.1,0.1],y_offsets=[0.1,0.1],traj_config=None):
            transform_dict = self.randomize_cans(x_offsets,y_offsets)
            self.random_place_robot()
            self.drop_randomizer(x_offsets,y_offsets)

            if not(self.collision):
                t = self.env.GetKinBody(self.drop_name).GetTransform()
                t[2,3] = 0.0
                self.env.GetKinBody(self.drop_name).SetTransform(t)
            
            for obj in self.bound_object_name:
                self.collision_set.add(self.env.GetKinBody(obj))   

            return transform_dict
        
        def reset_planks(self,tranform_dict):
            self.release()
            for p in tranform_dict.keys():
                p.SetTransform(tranform_dict[p])
        
            return True
        
        def draw_axes(self,objects):
            axes_object = []
            for obj in objects:
                obj_t = obj.GetTransform()
                axes_object.append(DrawAxes(self.env,obj_t))
            
            return axes_object
        
        def get_discretized_poses(self,obj1,obj2):
            obj1_type = str(obj1.GetName()).split("_")[0]
            obj2_type = str(obj2.GetName()).split("_")[0]
            obj1_pose = self.get_pose_from_transform(obj1.GetTransform())
            obj2_pose = self.get_pose_from_transform(obj2.GetTransform())

            discretizer = Config.get_discretizer(obj1=obj1_type.split("_")[0],obj2=obj2_type.split("_")[0])
            relative_pose = self.get_relative_pose(pose1=obj1_pose,pose2=obj2_pose)
            discretized_pose = discretizer.get_discretized_pose(input_pose=relative_pose,is_relative=True)
            
            return discretized_pose
        
        def set_camera_wrt_obj(self,object_name,transform_num=1):
            obj = self.env.GetKinBody(object_name)
            relative_t = np.load(Config.ROOT_DIR+"camera_wrt_{}_{}.npy".format(object_name,transform_num))
            self.env.GetViewer().SetCamera(obj.GetTransform().dot(relative_t))

        def save_camera_angle_wrt_obj(self,object_name,transform_num=0):
            obj = self.env.GetKinBody(object_name)
            camera_t = self.env.GetViewer().GetCameraTransform()

            relative_t = get_relative_transform(obj.GetTransform(),camera_t)
            np.save(Config.ROOT_DIR+"camera_wrt_{}_{}.npy".format(object_name,transform_num),relative_t)

        def start(self,complete_random=False):
            i = 0        
            flag = 0
            j = 0
            pbar = tqdm.tqdm(total=self.n)
            with self.env:
            # if True:
                if self.env.GetViewer() is not None:
                    self.set_camera_wrt_obj("table6",2)
                while i < self.n:
                    transform_dict = self.randomize_env()
                    planks_to_set=self.object_count
                    if self.minimum_object_count != self.object_count:
                        planks_to_set = np.random.randint(low=self.minimum_object_count,high=self.object_count+1)

                    goalLoc_list = self.can_list[:planks_to_set]
                    while j <= self.j:
                        state_list = []
                        self.robot.SetActiveDOFValues(self.init_pose)
                        self.reset_planks(transform_dict)

                        for goalLoc in goalLoc_list:
                            traj_1 = None
                            traj_2 = None
                            traj_3 = None

                            grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
                            init_state =  [self.get_one_state()]

                            plank_count = int(str(goalLoc.GetName()).split("_")[-1])-1
                            #reaching plank
                            counter = 0
                            while traj_1 is None:
                                gp = self.sample_grasp_pose(self.can_list[plank_count].GetName())
                                if self.compute_mp:
                                    traj_1 = self.compute_motion_plan(gp)
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
                            self.set_to_last_waypoint(traj_1)
                            grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
                            # discretized_pose = self.get_discretized_poses(obj1=self.robot_1.robot,obj2=self.can_list[plank_count])
                            state1 = self.get_state_list(traj_1,grab_flag=grabbed_flag)
                            self.grab(self.can_list[plank_count].GetName())
                            grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
                            one_state = self.get_one_state()
                            state1.append(one_state)

                            #taking plank to droparea
                            counter = 0
                            while traj_2 is None:
                                goal_t = self.sample_goal_pose(object_name=str(goalLoc.GetName()))
                                ep = self.sample_grasp_pose(pose=goal_t)
                                if self.compute_mp:
                                    traj_2 = self.compute_motion_plan(ep)
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
                            self.set_to_last_waypoint(traj_2)
                            self.collision_set.add(self.can_list[plank_count])
                            grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
                            state2 = self.get_state_list(traj_2,grab_flag=grabbed_flag)
                            self.release()
                            grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
                            one_state = self.get_one_state()
                            state2.append(one_state)

                            #random_placing_robot
                            while traj_3 is None:
                                ep,_ = self.random_config_robot(current_dof=self.robot.GetActiveDOFValues())
                                if self.compute_mp:
                                    traj_3 = self.compute_motion_plan(ep)
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
                            self.set_to_last_waypoint(traj_3)
                            grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
                            state3 = self.get_state_list(traj_3,grab_flag=grabbed_flag)

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
                        
                        for obj in self.env.GetBodies():
                            self.object_names.add(str(obj.GetName()))

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
            with self.env:
                final_data = {"env_states": self.data,"object_list": list(self.object_names)}
                path = Config.DATA_MISC_DIR+ self.env_name+"/"
                if not os.path.exists(path):
                    os.makedirs(path)
                cPickle.dump(final_data,open(Config.DATA_MISC_DIR+ self.env_name+"/"+ self.file_name ,"wb"),protocol=cPickle.HIGHEST_PROTOCOL)
                print("{} data saved".format(self.env_name))

        def random_config_robot(self,current_dof,robot=None,exp=None):
            random_dof_values = []
            if self.robot_name!="yumi":
                random_x = np.random.uniform(self.env_x_limits[0],self.env_x_limits[1])
                random_y = np.random.uniform(self.env_y_limits[0],self.env_y_limits[1])
                random_z = np.random.uniform(self.env_z_limits[0]+0.3,self.env_z_limits[1])
                random_roll  = np.random.uniform(-3.14, 3.14)
                random_pitch = np.random.uniform(-3.14, 3.14)
                random_yaw   = np.random.uniform(-3.14, 3.14)

                random_dof_values = [random_x,random_y,random_z,random_roll,random_pitch,random_yaw]

            else:
                self.activate_manip_joints()
                lower_limits,upper_limits = self.robot.GetActiveDOFLimits()
                for i in range(len(current_dof)):
                    r = np.random.uniform(lower_limits[i],upper_limits[i])
                    random_dof_values.append(r)

            self.robot.SetActiveDOFValues(random_dof_values)
            t = self.gripper_link.GetTransform()

            if self.collision_check([self.robot]):
                self.random_config_robot(current_dof)

            self.robot.SetActiveDOFValues(current_dof)   
            return random_dof_values, t

        def compute_motion_plan(self,goal,robot=None):
            try:
                traj = self.motion_planner.PlanToConfiguration(self.robot,goal)
                return traj

            # except PlanningError as e:
            except:
                # print(e)
                # print("could not find motion plan")
                return None

        def get_state_list(self,traj,grab_flag):
            state_list = []
            if type(traj) != list:
                len_traj = traj.GetNumWaypoints()

                wp = traj.GetWaypoint(0)
                if len(wp) == 3:
                    self.activate_base_joints()
                else:
                    self.activate_manip_joints()

                for i in range(len_traj):
                    wp = traj.GetWaypoint(i)
                    obj_dic = {}
                    self.robot.SetActiveDOFValues(wp)
                    for obj in self.env.GetBodies():
                        if str(obj.GetName()) not in self.bound_object_name:
                            if obj != self.robot:
                                name = str(obj.GetName())
                                obj_dic[name] = self.get_pose_from_transform(obj.GetTransform())
                            else:
                                for link in self.robot_type_object_mappings.keys():
                                    name = self.robot_type_object_mappings[link]
                                    obj_dic[name] = [self.robot.GetActiveDOFValues(),self.get_pose_from_transform(self.robot.GetLink(link).GetTransform())]

                    kwargs = {}

                    for n in range(1,self.num_robots+1):
                        grabbed_flag_key = "grabbed_flag_{}".format(n)
                        grabbed_object_key = "grabbed_object_{}".format(n)
                        grabbed_flag = getattr(self,grabbed_flag_key)
                        grabbed_object = getattr(self,grabbed_object_key)
                        kwargs[grabbed_flag_key] = grabbed_flag
                        kwargs[grabbed_object_key] = grabbed_object
                    
                    state = EnvState(obj_dic=obj_dic,keyword_arguments=kwargs,num_robots=self.num_robots)
                    state_list.append(state)
            
            else:
                wp = traj[0]
                if len(wp) == 3:
                    self.activate_base_joints()
                else:
                    self.activate_manip_joints()

                self.robot.SetActiveDOFValues(wp)
                state_list.append(self.get_one_state())

            return state_list
        
        def get_one_state(self):
            obj_dic = {}
            for obj in self.env.GetBodies():
                if str(obj.GetName()) not in self.bound_object_name:
                    if obj != self.robot:
                        name = str(obj.GetName())
                        obj_dic[name] = self.get_pose_from_transform(obj.GetTransform())
                    else:
                        for link in self.robot_type_object_mappings.keys():
                            name_type = self.robot_type_object_mappings[link].split("_")[0]
                            joints_activation_function = getattr(self,"activate_{}".format(Config.ROBOT_TYPES[name_type]))
                            joints_activation_function()
                            name = self.robot_type_object_mappings[link]
                            obj_dic[name] = [self.robot.GetActiveDOFValues(),self.get_pose_from_transform(self.robot.GetLink(link).GetTransform())]
            
            kwargs = {}

            for n in range(1,self.num_robots+1):
                grabbed_flag_key = "grabbed_flag_{}".format(n)
                grabbed_object_key = "grabbed_object_{}".format(n)
                grabbed_flag = getattr(self,grabbed_flag_key)
                grabbed_object = getattr(self,grabbed_object_key)
                kwargs[grabbed_flag_key] = grabbed_flag
                kwargs[grabbed_object_key] = grabbed_object
            
            state = EnvState(obj_dic=obj_dic,keyword_arguments=kwargs,num_robots=self.num_robots)

            return state

        def transform_from_wp(self,dof_vals):
            # print('last@transform_from_wp')
            quat = quatFromAxisAngle(dof_vals[3:])
            pose = []
            pose.extend(quat)
            pose.extend(dof_vals[:3])
            transform = matrixFromPose(pose)
            return transform

        def get_pose_from_transform(self,transform):
            # print('last@get_pose_from_transform')
            pose = poseFromMatrix(transform)
            quat = pose[:4]
            eul = axisAngleFromQuat(quat)
            dofs = []
            dofs.extend(pose[4:])
            dofs.extend(eul)

            return dofs

    return Packing