from src.useful_functions import blockPrinting
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
from Environments.CafeWorld import object_models
from src.robot_files.Robots import Fetch, MagicGripper
import math
import importlib
from time import sleep
from src.useful_functions import get_relative_transform
import os

def GetDomain(robot_name="Fetch"):
    mod = importlib.import_module("src.robot_files.Robots")
    RobotClass = getattr(mod,robot_name)

    class CafeWorld(RobotClass):
        def __init__(self,env_name,number_of_configs,number_of_mp,axis_for_offset,file_name="_data.p",reference_structure_name=None,visualize = False,plank_count=None, random=False,order=False,planks_in_init_state=0,surface="",quadrant=None,grasp_num=None,minimum_plank_count=None,mp=True,num_robots=1,structure_dependance=False,object_list=[],experiment_flag=False,real_world_experiment=False,set_y=False,complete_random=False,data_gen=False):
            self.env = orpy.Environment()
            self.env.SetDebugLevel(0)
            super(CafeWorld,self).__init__(env=self.env,id=1)
            
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
            
            self.bound_object_name = ['world_final']
            self.collision_set = set([self.robot])
            for obj_name in self.bound_object_name:
                self.collision_set.add(self.env.GetKinBody(obj_name))

            self.clearance = 0.005
            self.obj_h = 0.05
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
            self.match_quadrants = False

            self.object_count = plank_count
            if minimum_plank_count is None:
                self.minimum_object_count = self.object_count
            else:
                self.minimum_object_count = minimum_plank_count

            cc = orpy.RaveCreateCollisionChecker(self.env,'pqp')
            cc.SetCollisionOptions(orpy.CollisionOptions.AllGeometryContacts)
            self.env.SetCollisionChecker(cc)
            self.file_name = file_name
            self.motion_planner = BiRRTPlanner()

            self.robot_dims = self.get_object_dims(self.robot_name)
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
            self.robot_1 = self
            self.robots = [self.robot_1]

            if visualize:
                self.env.SetViewer('qtcoin')

            if "problem" not in env_name:
                self.env.GetKinBody("countertop").SetName("surface_0")
                
                for i in range(self.object_count):
                    # self.spawn_goalLoc()
                    self.spawn_object()
                # table_count = min(2,max(2,self.object_count))
                table_count = 5
                for _ in range(table_count):
                    self.spawn_table()

                # self.tt = self.env.GetKinBody("surface_1").GetTransform()
                # self.randomize_env()
            else:
                self.can_list = [obj for obj in self.env.GetBodies() if "can" in str(obj.GetName())]
                self.table_list = [obj for obj in self.env.GetBodies() if "surface" in str(obj.GetName())]
                self.table_list.remove(self.env.GetKinBody("surface_0"))

            # t1 = self.load_traj_data("env4")["env_states"]
            # traj = t1[2]
            # self.set_env_state(traj[0])
            # self.set_camera_wrt_obj("surface_2",2)
            # self.execute_traj(traj[:5])
            self.trace = []
            self.compute_mp = mp

            # rcr_data = self.load_rcr_data()
            # self.debug_plotting(rcr_data,obj1="surface_2", obj2="can_1")

            # while True:
            #     self.randomize_env()

            # self.discretizer = Config.get_discretizer("surface","can")
            # env_data = self.load_traj_data("env2")
            # for traj in env_data["env_states"]:
            #     self.execute_traj(traj)

            # with open(Config.DATA_MISC_DIR+"200_0_rcr_indices.p") as f:
                # indices_data = cPickle.load(f)
            # self.randomize_env()
            # self.debug_plotting(indices_data["rcr_dict"],obj1="can_1", obj2="surface_1")

            # # req_state = data["env_states"][0][2484]
            # trajs = data["env_states"]
            # req_states = []
            # with open(Config.DATA_MISC_DIR+"plot.p","rb") as f:
            #     indices = cPickle.load(f)
            #     f.close()
            # for a,b in indices:
            #     req_states.append(trajs[a][b])

            # self.set_env_state(traj)
            # for s in req_states:
            #     self.set_env_state(s)
            #     print("")

        def load_rcr_data(self):
            with open(Config.DATA_MISC_DIR+"200_67_rcr_indices.p","rb") as f:
                data_dict = cPickle.load(f)
                f.close()

            return data_dict["rcr_dict"]

        def remove_traces(self):
            for t in self.trace:
                t.Close()               

        def execute_traj(self,traj):
            for i,state in enumerate(traj):
                self.set_env_state(state)
                sleep(0.2)

        def load_traj_data(self,env_name):
            with open("Data/CafeWorld/misc/{}/{}_data.p".format(env_name,env_name),"rb") as f:
                data = cPickle.load(f)
                print("loaded")
                f.close()
            return data

        def set_env_state(self,env_state):
            objects_not_found = []
            with self.env:
            # if True:
                id_dict = {}
                for rob in self.robots:
                    if getattr(env_state,"grabbed_flag_{}".format(rob.id)):
                        id_dict[rob.id] = {
                                    "grabbed_object_{}".format(rob.id) : getattr(env_state,"grabbed_object_{}".format(rob.id))
                        }
                    rob.release()
                        
                for obj in env_state.object_dict.keys():
                    if obj.split("_")[0] not in Config.ROBOT_TYPES.keys():
                        object = self.env.GetKinBody(obj)
                        if object is not None:
                            object.SetTransform(env_state.transform_from_pose(env_state.object_dict[obj]))
                        else:
                            objects_not_found.append(obj)
                    else:
                        for rob in self.robots:
                            if rob.id == int(obj.split("_")[1]):
                                joints_activation_function = getattr(rob,"activate_{}".format(Config.ROBOT_TYPES[obj.split("_")[0]]))
                                joints_activation_function()
                                rob.robot.SetActiveDOFValues(env_state.object_dict[obj][0])

                for rob in self.robots:
                    if rob.id in id_dict.keys():
                        rob.grab(id_dict[rob.id]["grabbed_object_{}".format(rob.id)])
                
            return objects_not_found

        def get_relative_pose(self, pose1, pose2):
            #obj2 w.r.t. obj1
            transform1 = self.transform_from_wp(pose1)
            transform2 = self.transform_from_wp(pose2)
            return self.get_pose_from_transform((np.linalg.pinv(transform1).dot(transform2)))

        def debug_trajs(self,traj):
            for state in traj:
                gripper_pose = state.object_dict["gripper_1"][1]
                base_pose = state.object_dict["freight_1"][1]
                object_pose = state.object_dict["can_1"]
                table_pose = state.object_dict["table_1"]
                counter_pose = state.object_dict["countertop"]
                rp1 = state.get_relative_pose(object_pose,gripper_pose)
                rp2 = state.get_relative_pose(table_pose,base_pose)
                rp3 = state.get_relative_pose(counter_pose,base_pose)
                current_object_pose = self.env.GetKinBody("can_1").GetTransform()
                current_table_pose = self.env.GetKinBody("table_1").GetTransform()
                current_counter_pose = self.env.GetKinBody("countertop").GetTransform()
                p1 = current_object_pose.dot(state.transform_from_pose(rp1))
                p2 = current_table_pose.dot(state.transform_from_pose(rp2))
                p3 = current_counter_pose.dot(state.transform_from_pose(rp3))
                self.trace.append(self.env.plot3(points = [p1[0,3],p1[1,3],p1[2,3]], pointsize = 0.05, colors = np.array([255,0,0]), drawstyle = 1 ))
                self.trace.append(self.env.plot3(points = [p2[0,3],p2[1,3],p2[2,3]], pointsize = 0.05, colors = np.array([0,255,0]), drawstyle = 1 ))
                self.trace.append(self.env.plot3(points = [p3[0,3],p3[1,3],p3[2,3]], pointsize = 0.05, colors = np.array([0,0,255]), drawstyle = 1 ))

        def debug_plotting(self,rcrs,obj1,obj2,color=[0,255,0]):
            discretizer = Config.get_discretizer(obj1.split("_")[0],obj2.split("_")[0])
            env_state = self.get_one_state()
            # for obj in env_state.object_dict.keys():
            obj1_pose = env_state.object_dict[obj1]
            if obj1 in Config.ROBOT_TYPES:
                obj1_pose = obj1_pose[1]
            obj1_T = self.transform_from_wp(obj1_pose)

            obj2_pose = env_state.object_dict[obj2]
            if obj2 in Config.ROBOT_TYPES:
                obj2_pose = obj2_pose[1]
            obj2_T = self.transform_from_wp(obj2_pose)
            
            xyz_set = set()
            if obj1.split("_")[0] in rcrs.keys() and obj2.split("_")[0] in rcrs[obj1.split("_")[0]].keys():
                region = rcrs[obj1.split("_")[0]][obj2.split("_")[0]]
                switch = False
            else:
                region = rcrs[obj2.split("_")[0]][obj1.split("_")[0]]
                switch = True

            for rgn in region:
                for ind in rgn:
                    for _ in range(20):
                        try:
                            converted_sample = discretizer.convert_sample(ind[:6],is_relative=True)
                        except:
                            converted_sample = discretizer.convert_sample(ind[:3],is_relative=True)
                            converted_sample.extend(self.get_pose_from_transform(np.eye(4))[3:])
                        converted_T = self.transform_from_wp(converted_sample)

                        grounded_T = obj1_T.dot(converted_T)
                        if switch:
                            grounded_T = obj1_T.dot(np.linalg.pinv(converted_T))

                        x,y,z = grounded_T[:3,3]
                        # x = round(round(x/0.05)*0.05,2)
                        # y = round(round(y/0.05)*0.05,2)
                        # z = round(round(z/0.05)*0.05,2)
                        xyz_set.add(tuple([x,y,z]))
                
            for i in xyz_set:
                i = list(i)
                self.trace.append(self.env.plot3(points = [i[0],i[1],i[2]], pointsize = 0.07, colors = np.array(color), drawstyle = 1 ))

            return None

        def sample_can_countertop(self,object_name,surface_name="surface_0",range=[[-0.43,-0.1],[0.1,0.43]]):
            obj = self.env.GetKinBody(object_name)
            t_current = obj.GetTransform()
            current_grabbed_flag = False
            if self.grabbed_flag_1:
                current_grabbed_flag = True
                grabbed_object = deepcopy(self.grabbed_object_1)
                self.release()
            
            attempt_num = 0
            while attempt_num < 25:
                x = np.random.uniform(low=range[0][0]+self.can_radius,high=range[0][1]-self.can_radius)
                y = np.random.uniform(low=range[1][0]+self.can_radius,high=range[1][1]-self.can_radius)
                # z = self.countertop_h
                z = self.clearance

                countertop = self.env.GetKinBody(surface_name)
                countertop_t = countertop.GetTransform()

                t = matrixFromPose([1,0,0,0,x,y,z])            

                t = countertop_t.dot(t)

                obj.SetTransform(t)
                if not (self.collision_check([obj]) or self.obj_checker(obj)):
                    break
                attempt_num += 1
            
            obj.SetTransform(t_current)
            if current_grabbed_flag:
                self.grab(obj=grabbed_object)
                current_grabbed_flag = False

            if attempt_num == 24:
                return None

            return t
        
        def sample_can_table(self,object_name,surface_name,range=[[-0.43,-0.1],[0.1,0.43]]):
            obj = self.env.GetKinBody(object_name)
            t_current = obj.GetTransform()
            current_grabbed_flag = False
            if self.grabbed_flag_1:
                current_grabbed_flag = True
                grabbed_object = deepcopy(self.grabbed_object_1)
                self.release()

            obj_dims = self.get_object_dims(object_name=object_name)
            
            table = self.env.GetKinBody(surface_name)
            table_t = table.GetTransform()
            table_x,table_y,table_z = table_t[:3,3]
            table_dims = self.get_object_dims(str(table.GetName()))
            attempt_num = 0
            while attempt_num < 25:
                x = np.random.uniform(low=range[0][0]+obj_dims[0],high=range[0][1]-obj_dims[0])
                y = np.random.uniform(low=range[1][0]+obj_dims[1],high=range[1][1]-obj_dims[1])
                z = self.table_top_thickness/2.0 + self.clearance

                grasp_num = np.random.choice(Config.NUM_GRASPS)
                rot_angle = (grasp_num * (2*np.pi) / Config.NUM_GRASPS)
                rot_z = matrixFromAxisAngle([0,0,rot_angle])

                # t = rot_z.dot(matrixFromPose([1,0,0,0,x,y,z]))
                t = matrixFromPose([1,0,0,0,x,y,z])
                t = table_t.dot(t)
                t = t.dot(rot_z)

                # reorient_pitch_T = matrixFromAxisAngle([0, 0, -np.pi/4])
                # t = t.dot(reorient_pitch_T)

                obj.SetTransform(t)
                if not (self.collision_check([obj]) or self.obj_checker(obj)):
                    break

                attempt_num += 1
            
            obj.SetTransform(t_current)
            if current_grabbed_flag:
                self.grab(obj=grabbed_object)
                current_grabbed_flag = False

            if attempt_num == 25:
                return None

            return t

        def sample_robot_base_countertop(self,surface_name="surface_0",mp=False,range=None):
            self.activate_base_joints()
            current_dof_vals = self.robot.GetActiveDOFValues()

            countertop = self.env.GetKinBody(surface_name)
            countertop_t = countertop.GetTransform()
            diff = 0.4
            countertop_x_dim = 0.45
            x_offset = -diff-countertop_x_dim
            y_offset = 0
            # x_offset = np.random.uniform(low=self.near_countertop_x[0]+diff, high=self.near_countertop_x[1]-diff)
            # y_offset = np.random.uniform(low=self.near_countertop_y[0]+diff, high=self.near_countertop_y[1]-diff)
            diff_translation_matrix = matrixFromPose([1,0,0,0,x_offset,y_offset,0])

            valid_pose = None
            count = 0
                
            while valid_pose is None and count < 5:
                rot_angle = (2*np.pi) / Config.NUM_GRASPS
                
                rot_Z = matrixFromAxisAngle([0, 0, -np.pi/2])
                rot_mat = matrixFromAxisAngle([0,0,rot_angle])
                t = np.eye(4)
                t = countertop_t.dot(rot_mat).dot(rot_Z).dot(diff_translation_matrix)

                _x = t[0,3]
                _y = t[1,3]
                _yaw = axisAngleFromRotationMatrix(t[:3,:3])[-1]
                pose = [_x,_y,_yaw]
                self.robot.SetActiveDOFValues(pose)
                count += 1
                if not self.collision_check([self.robot]):
                    valid_pose = pose
                    break
            
            self.robot.SetActiveDOFValues(current_dof_vals)
            return valid_pose
        
        def sample_robot_base_table(self,surface_name,mp=False,range=None):
            self.activate_base_joints()
            current_dof_vals = self.robot.GetActiveDOFValues()

            diff = 0.4
            table = self.env.GetKinBody(surface_name)
            table_t = table.GetTransform()
            table_dims = self.get_object_dims(str(table.GetName()))
            diff_translation_matrix = matrixFromPose([1,0,0,0,-diff-table_dims[0],0,0])

            valid_pose = None
            count = 0
                
            while valid_pose is None and count < 5:
                if self.grasp_num is None:
                    rot_angle = np.random.uniform(low=-np.pi,high=np.pi)
                else:
                    rot_angle = (self.grasp_num * (2*np.pi) / Config.NUM_GRASPS)                
                
                rot_Z = matrixFromAxisAngle([0, 0, -np.pi/2])
                rot_mat = matrixFromAxisAngle([0,0,rot_angle])
                t = np.eye(4)
                t = table_t.dot(rot_mat).dot(rot_Z).dot(diff_translation_matrix)

                _x = t[0,3]
                _y = t[1,3]
                _yaw = axisAngleFromRotationMatrix(t[:3,:3])[-1]
                pose = [_x,_y,_yaw]
                self.robot.SetActiveDOFValues(pose)
                count += 1
                if not self.collision_check([self.robot]):
                    valid_pose = pose
                    break
            
            self.robot.SetActiveDOFValues(current_dof_vals)
            return valid_pose

        def store_init_config(self):
            bodies = self.env.GetBodies()
            for body in bodies:
                if body.GetName() != self.robot_name:
                    self.init_config[body.GetName()] = body.GetTransform()
                else:
                    for link in self.robot_type_object_mappings:
                        self.init_config[self.robot_type_object_mappings[link]] = self.robot.GetLink(link).GetTransform()

        def table_checker(self,table):
            t1 = table.GetTransform()[:2,3]
            for pl in self.table_list:
                if table!=pl:
                    t2 = pl.GetTransform()[:2,3]
                    if abs(min(t1-t2)) > 3 or np.linalg.norm(t1-t2) > 3.5:
                        continue
                    return True

            return False

        def table_randomizer(self):
            objects_to_remove = set()
            for obj in self.collision_set:
                if obj.GetName().startswith("surface"):
                    objects_to_remove.add(obj)

            for obj in objects_to_remove:
                self.collision_set.remove(obj)

            for table in self.table_list:
                table_dims = self.get_object_dims(object_name=str(table.GetName()))
                while True:
                    x1 = np.random.uniform(low=self.large_range_x[0]+table_dims[0],high=self.large_range_x[1]-table_dims[0])
                    y1 = np.random.uniform(low=self.large_range_y[0]+table_dims[1],high=self.large_range_y[1]-table_dims[1])

                    x2 = np.random.uniform(low=self.table_spawn_x_range[0]+table_dims[0],high=self.table_spawn_x_range[1]-table_dims[0])
                    y2 = np.random.uniform(low=self.table_spawn_y_range[0]+table_dims[1],high=self.table_spawn_y_range[1]-table_dims[1])

                    z = self.table_h

                    t1 = matrixFromPose([1,0,0,0,x1,y1,z])
                    t2 = matrixFromPose([1,0,0,0,x2,y2,z])

                    for t in [t1,t2]:
                        table.SetTransform(t)
                        if not (self.collision_check([table]) or self.table_checker(table)):
                            self.collision_set.add(table)
                            break
                    
                    if table in self.collision_set:
                        break
        
        def sample_grasp_pose(self,object_name="",pose = [],grasp_num=None,surface_id=None):
            if pose != []:
                world_T_obj = pose
            else:
                world_T_obj = self.env.GetKinBody(object_name).GetTransform()
                            
            self.activate_manip_joints()

            rot_Z = matrixFromAxisAngle([0, 0, -np.pi/2])
            valid_pose = None
            gripper_offset = self.grasping_offset[Config.OBJECT_NAME[0]]
            if grasp_num is None:
                for j in range(Config.NUM_GRASPS):
                    rot_ang = np.random.uniform(low = -np.pi, high = np.pi)
                    obj_T_gripper = matrixFromPose([1, 0, 0, 0, gripper_offset, 0, self.can_h/2.0])
                    rot_mat = matrixFromAxisAngle([0, 0, rot_ang])

                    wrist_roll_pose = self.robot.GetLink('wrist_roll_link').GetTransform()
                    gripper_pose = self.robot.GetLink('gripper_link').GetTransform()
                    wrist_pose_wrt_gripper = np.matmul(np.linalg.inv(gripper_pose), wrist_roll_pose)

                    grasp_T = np.eye(4).dot(rot_mat).dot(obj_T_gripper)
                    grasp_T = world_T_obj.dot(grasp_T)
                    grasp_T = np.matmul(grasp_T,wrist_pose_wrt_gripper)
                    grasp_pose = poseFromMatrix(grasp_T)
                    ik_sols = self.get_ik_solutions(grasp_T,collision_fn=self.collision_check)
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
                obj_T_gripper = matrixFromPose([1, 0, 0, 0, gripper_offset, 0, self.can_h/2.0])
                rot_mat = matrixFromAxisAngle([0, 0, rot_ang])

                wrist_roll_pose = self.robot.GetLink('wrist_roll_link').GetTransform()
                gripper_pose = self.robot.GetLink('gripper_link').GetTransform()
                wrist_pose_wrt_gripper = np.matmul(np.linalg.inv(gripper_pose), wrist_roll_pose)

                grasp_T = world_T_obj.dot(rot_mat).dot(rot_Z).dot(obj_T_gripper)
                grasp_T = np.matmul(grasp_T,wrist_pose_wrt_gripper)
                
                grasp_pose = poseFromMatrix(grasp_T)
                ik_sols = self.get_ik_solutions(grasp_T,collision_fn=self.collision_check)
                if len(ik_sols) > 0:
                    valid_pose = ik_sols
            
            return valid_pose

        def sample_goal_pose(self,config_tuple):
            obj_name,surface_name = config_tuple
            surface_id = int(surface_name.split("_")[1])

            if surface_id == 0:
                surface_type = "countertop"
            else:
                surface_type = "table"
            
            wrist_roll_pose = self.robot.GetLink('wrist_roll_link').GetTransform()
            gripper_pose = self.robot.GetLink('gripper_link').GetTransform()
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
                ik_sols = self.get_ik_solutions(grasp_pose,collision_fn=self.collision_check)
                if len(ik_sols) > 0:
                    valid_ik = ik_sols
                
                count+=1

            return valid_ik
        
        def set_goalLoc(self,traj_config):
            goalLoc_list = []

            for obj_config in traj_config:
                i = int(str(obj_config[1][0]).split("_")[-1])
                goalLoc = self.env.GetKinBody("goalLoc"+"_{}".format(i))
                goalLoc_list.append(goalLoc)

                surface_name = str(obj_config[1][1])
                surface_id = int(surface_name.split("_")[1])
                if surface_id == 0:
                    surface_type = "countertop"
                else:
                    surface_type = "table"

                surface = self.env.GetKinBody(surface_name)
                surface_T = surface.GetTransform() if surface is not None else np.eye(4)
                relative_T = matrixFromPose(self.goalLoc_relative_pose[surface_type])
                
                goalLoc_T = surface_T.dot(relative_T)
                goalLoc.SetTransform(goalLoc_T)

            return goalLoc_list

        def get_init_pose(self):
            return self.init_pose    

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

        def get_random_range(self):
            max_y = -100000000
            for obj in self.env.GetBodies():
                if Config.SURFACE_NAME in str(obj.GetName()) and int(str(obj.GetName()).split("_")[1])>0:
                    max_y = max(obj.GetTransform()[1,3],max_y)
            
            range = [self.large_range_x,([max_y+1.5,-1.2])]

            return range

        def random_place_robot(self,surface_name=None,mp=False,range=[[-0.5,6],[-12,-3]]):
            if mp is False:
                self.tuck_arm()
                
            self.activate_base_joints()
            current_dof_val = self.robot.GetActiveDOFValues()
            
            while True:
                random_x = np.random.uniform(range[0][0],range[0][1])
                random_y = np.random.uniform(range[1][0],range[1][1])
                random_yaw = np.random.uniform(-3.14, 3.14)
                sample_config = [random_x,random_y,random_yaw]
                
                self.init_base_pose = sample_config
                self.robot.SetActiveDOFValues(sample_config)

                if not self.collision_check([self.robot]):
                    break
            
            if mp:
                self.robot.SetActiveDOFValues(current_dof_val)

            return sample_config

        def get_possible_configs(self,num_objects=1,configs_to_exclude=set([])):
            possible_configs=[]

            random_cans = []
            while len(random_cans) < 2 and np.random.randint(0,2) and num_objects > 1:
                can_num = np.random.randint(1,num_objects)
                if can_num not in random_cans:
                    random_cans.append(can_num)

            for can_num in random_cans:
                possible_configs.append(("can_{}".format(can_num),"surface_0"))

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
            self.collision_set.add(self.robot)
            for obj_name in self.bound_object_name:
                self.collision_set.add(self.env.GetKinBody(obj_name))

            t = np.eye(4)
            t[2,3] = 10
            for obj in self.added_objects:
                obj.SetTransform(t.dot(obj.GetTransform()))
            self.added_objects = []

            self.table_randomizer()
            random_range = self.get_random_range()
            self.random_place_robot(range=random_range)

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
                can = self.env.GetKinBody(can_name)
                if can in self.added_objects or init_config in init_configs:
                    possible_configs.append(init_config)
                    continue
                init_surface_name = init_config[1]
                init_surface_id = int(init_surface_name.split("_")[1])
                init_surface_T = self.env.GetKinBody(init_surface_name).GetTransform()
                init_surface_p = self.get_pose_from_transform(init_surface_T)
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
                    self.collision_set.add(self.env.GetKinBody(self.bound_object_name[0]))
                    while sample_flag:
                        if init_surface_id == 0:
                            surface_range = self.default_countertop_range
                        else:
                            surface_range = self.default_table_range
                        ll_T = sampler(object_name=can_name,surface_name=init_surface_name,range=surface_range)
                        if ll_T is not None:
                            ll_P = self.get_pose_from_transform(ll_T)
                            can.SetTransform(ll_T)
                            relative_pose = self.get_relative_pose(pose1=ll_P,pose2=init_surface_p)
                            discretized_pose = discretizer.get_discretized_pose(input_pose=relative_pose,is_relative=True)
                            discretized_pose.append(int(self.grabbed_flag_1))

                            if not (self.collision_check([can]) or self.obj_checker(can)):
                                for region in rcr:
                                    if discretized_pose in region:
                                        sample_flag = False
                                        if str(can.GetName()).split("_")[0] == Config.OBJECT_NAME[0]:
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
            for obj in self.env.GetBodies():
                if str(obj.GetName()) not in self.bound_object_name:
                    if obj != self.robot:
                        name = str(obj.GetName())
                        obj_dic[name] = obj.GetTransform()
                    else:
                        self.activate_manip_joints()
                        obj_dic["gripper_1"] = self.robot.GetActiveDOFValues()

                        self.activate_base_joints()
                        obj_dic["freight_1"] = self.robot.GetActiveDOFValues()
            
            return obj_dic, traj_config[:objects_to_set]

        def set_traj_config(self,given_surface_lists=None,traj_count=0):
            traj_config = []
            for i in range(self.object_count):
                if given_surface_lists is None:
                    can_traj_config = []
                    can_name = str(self.can_list[i].GetName())

                    possible_samplers = deepcopy(self.possible_samplers)
                    
                    possible_surfaces = ["surface_0"]
                    for table in self.table_list:
                        possible_surfaces.append(str(table.GetName()))
                    
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

                    goal_sampler_id = int(given_surface_lists[i][1].split("_")[1])
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
                            init_surface_name = str(l[0].GetName())
                        else:
                            index = np.random.randint(len(l))
                            init_surface_name = str(l[index].GetName())                    

                    can_traj_config.append([can_name,init_surface_name])
                    can_traj_config.append(given_surface_lists[i])

                traj_config.append(can_traj_config)

            return traj_config

        def setup_objects(self,init_object_dict):
            self.release()
            for obj_name in init_object_dict.keys():
                if obj_name not in self.robot_type_object_mappings.values():
                    obj = self.env.GetKinBody(obj_name)
                    obj.SetTransform(init_object_dict[obj_name])
                else:
                    dof_val = init_object_dict[obj_name]
                    if len(dof_val) == 3:
                        self.activate_base_joints()
                    else:
                        self.activate_manip_joints()
                    self.robot.SetActiveDOFValues(dof_val)

        def motion_plan_robot_base(self,config_tuple=None,given_pose=None):
            self.activate_base_joints()
            
            if config_tuple is not None:
                object_name, surface_name = config_tuple
                surface_id = int(surface_name.split("_")[1])
                if surface_id == 0:
                    surface_type = "countertop"
                else:
                    surface_type = "table"
                sampler = getattr(self,"sample_robot_base_{}".format(surface_type))
            else:
                surface_name = None
                sampler = self.random_place_robot

            range=self.get_random_range()
            traj = None
            count=0
            while traj is None and count<5:
                target_pose = None
                if given_pose is None:
                    target_pose = sampler(surface_name=surface_name,mp=True,range=range)
                else:
                    target_pose = given_pose

                if target_pose is not None:
                    if self.compute_mp:
                        traj = self.compute_motion_plan(target_pose)
                    else:
                        traj = [target_pose]
                
                count += 1
            
            return traj, target_pose
        
        def motion_plan_robot_arm(self,config_tuple=None,pose=[],grasp_num=None,given_pose=None,surface_id=None):
            self.activate_manip_joints()
            
            if config_tuple is not None:
                object_name, surface_name = config_tuple
                surface_id = int(surface_name.split("_")[1])
                sampler = self.sample_grasp_pose
            else:
                object_name = None
                sampler = self.get_arm_tuck_dofs

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
                        traj = self.compute_motion_plan(target_pose)
                    else:
                        traj = [target_pose]

                counter+=1

            return traj,target_pose

        def get_state_block(self,traj,config_tuple,grab=None):
            time.sleep(0.02)
            object_name,_ = config_tuple
            state = []
            self.set_to_last_waypoint(traj)
            if type(traj) != list:
                state = self.get_state_list(traj)
            else:
                state.append(self.get_one_state())
            
            if grab is not None:
                if grab:
                    self.grab(object_name)
                else:
                    self.release()

                one_state = self.get_one_state()
                state.append(one_state)

            return state

        def set_camera_wrt_obj(self,object_name,transform_num=1):
            obj = self.env.GetKinBody(object_name)
            relative_t = np.load(Config.ROOT_DIR+"camera_wrt_{}_{}.npy".format(object_name.split("_")[0],transform_num))
            self.env.GetViewer().SetCamera(obj.GetTransform().dot(relative_t))

        def save_camera_angle_wrt_obj(self,object_name,transform_num=0):
            obj = self.env.GetKinBody(object_name)
            camera_t = self.env.GetViewer().GetCameraTransform()

            relative_t = get_relative_transform(obj.GetTransform(),camera_t)
            np.save(Config.ROOT_DIR+"camera_wrt_{}_{}.npy".format(object_name,transform_num),relative_t)
        
        @blockPrinting
        def start(self,complete_random=False):
            i = 0        
            flag = 0
            j = 0
            pbar = tqdm.tqdm(total=self.n)
            with self.env:
            # if True:
                if self.env.GetViewer() is not None:
                    self.set_camera_wrt_obj("world_final")
                while i < self.n:
                    # print("starting of i loop i={}".format(i))
                    init_object_dic,traj_config = self.randomize_env(traj_count=i)
                    random_range = self.get_random_range()
                    while j <= self.j:
                        # print("start of j loop j={}".format(j))
                        self.setup_objects(init_object_dic)
                        state_list = []
                        # for config in traj_config:
                        #     print(config[0],config[1])

                        for init_config,goal_config in traj_config:
                            #going to init_surface
                            # print(init_config[0])
                            object_name, surface1_name = init_config
                            _, surface2_name = goal_config
                            surface1_id = int(surface1_name.split("_")[1])
                            if surface1_id == 0:
                                surface1_type = "countertop"
                            else:
                                surface1_type = "table"

                            surface2_id = int(surface2_name.split("_")[1])
                            if surface2_id == 0:
                                surface2_type = "countertop"
                            else:
                                surface2_type = "table"

                            #KP_1
                            state_1 = [self.get_one_state()]

                            base_sampler = getattr(self,"sample_robot_base_{}".format(surface1_type))

                            # print("sampling surface 1")
                            base_sample_1 = base_sampler(surface_name=surface1_name)
                                
                            if base_sample_1 is None:
                                if j == 0:
                                    flag=1
                                else:
                                    flag=2
                                break

                            self.activate_base_joints()
                            init_base_pose = self.robot.GetActiveDOFValues()
                            self.robot.SetActiveDOFValues(base_sample_1)

                            # print("grasp sampling")
                            traj_2,_ = self.motion_plan_robot_arm(init_config,grasp_num=self.grasp_num,surface_id=surface1_id)

                            if traj_2 is None:
                                if j == 0:
                                    flag=1
                                else:
                                    flag=2
                                break
                            
                            self.activate_base_joints()
                            self.robot.SetActiveDOFValues(init_base_pose)
                            # print("motion planning for base to surface 1")
                            traj_1, base_sample_1 = self.motion_plan_robot_base(init_config,given_pose=base_sample_1)
                            if traj_1 is None:
                                if j == 0:
                                    flag=1
                                else:
                                    flag=2
                                break
                            
                            #KP_2
                            state_2 = self.get_state_block(traj=traj_1,config_tuple=init_config)
                            #KP_3
                            state_3 = []
                            if not self.compute_mp:
                                self.activate_manip_joints()
                                current_dof = self.robot.GetActiveDOFValues()
                                random_pose,_ = self.random_config_robot(current_dof=current_dof,allowed_range_x=random_range[0],allowed_range_y=random_range[1])
                                state_3 = self.get_state_block(traj=[random_pose],config_tuple=init_config)

                            #KP_4 and KP_5
                            state_4_5 = self.get_state_block(traj=traj_2,config_tuple=init_config,grab=True)
                            #KP_6
                            state_6 = []
                            if not self.compute_mp:
                                self.activate_manip_joints()
                                current_dof = self.robot.GetActiveDOFValues()
                                random_pose,_ = self.random_config_robot(current_dof=current_dof,allowed_range_x=random_range[0],allowed_range_y=random_range[1])
                                state_6 = self.get_state_block(traj=[random_pose],config_tuple=init_config)

                            # print("tucking arm with object")
                            traj_3, _ = self.motion_plan_robot_arm()
                            if traj_3 is None:
                                if j == 0:
                                    flag=1
                                else:
                                    flag=2
                                break
                                
                            #KP_7
                            state_7 = self.get_state_block(traj=traj_3,config_tuple=init_config)
                            
                            #KP_8
                            state_8 = []
                            if not self.compute_mp:
                                self.activate_base_joints()
                                current_dof = self.robot.GetActiveDOFValues()
                                random_pose,_ = self.random_config_robot(current_dof=current_dof,allowed_range_x=random_range[0],allowed_range_y=random_range[1])
                                state_8 = self.get_state_block(traj=[random_pose],config_tuple=init_config)

                            # print("sampling surface 2")
                            base_sampler = getattr(self,"sample_robot_base_{}".format(surface2_type))
                            base_sample_2 = base_sampler(surface_name=surface2_name)
                            if base_sample_2 is None:
                                if j == 0:
                                    flag=1
                                else:
                                    flag=2
                                break

                            self.activate_base_joints()
                            init_base_pose = self.robot.GetActiveDOFValues()
                            self.robot.SetActiveDOFValues(base_sample_2)
                            
                            # print("trying to put down object")
                            traj_5 = None
                            count = 0
                            while traj_5 is None and count < 5:
                                ik = self.sample_goal_pose(goal_config)
                                if ik is not None:
                                    if self.compute_mp:
                                        traj_5 = self.compute_motion_plan(goal=ik)
                                    else:
                                        traj_5 = [ik]

                                count += 1
                            
                            if traj_5 is None:
                                if j == 0:
                                    flag=1
                                else:
                                    flag=2
                                break
                            
                            self.activate_base_joints()
                            self.robot.SetActiveDOFValues(init_base_pose)

                            # print("motion planning to surface 2")
                            traj_4, base_sample_2 = self.motion_plan_robot_base(config_tuple=goal_config,given_pose=base_sample_2)
                            if traj_4 is None:
                                if j == 0:
                                    flag=1
                                else:
                                    flag=2
                                break
                                
                            #KP_9
                            state_9 = self.get_state_block(traj=traj_4,config_tuple=goal_config)
                            #KP_10
                            state_10 = []
                            if not self.compute_mp:
                                self.activate_manip_joints()
                                current_dof = self.robot.GetActiveDOFValues()
                                random_pose,_ = self.random_config_robot(current_dof=current_dof,allowed_range_x=random_range[0],allowed_range_y=random_range[1])
                                state_10 = self.get_state_block(traj=[random_pose],config_tuple=init_config)
                            
                            #KP_11 and KP_12
                            state_11_12 = self.get_state_block(traj=traj_5,config_tuple=goal_config,grab=False)

                            #KP_13
                            state_13 = []
                            if not self.compute_mp:
                                self.activate_manip_joints()
                                current_dof = self.robot.GetActiveDOFValues()
                                random_pose,_ = self.random_config_robot(current_dof=current_dof,allowed_range_x=random_range[0],allowed_range_y=random_range[1])
                                state_13 = self.get_state_block(traj=[random_pose],config_tuple=init_config)

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
                            state_14 = self.get_state_block(traj=traj_6,config_tuple=init_config)

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
                            state_15 = self.get_state_block(traj=traj_7,config_tuple=goal_config)

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
                        
                        for obj in self.env.GetBodies():
                            if obj != self.robot:
                                self.object_names.add(str(obj.GetName()))
                            else:
                                for key in self.robot_type_object_mappings:
                                    self.object_names.add(self.robot_type_object_mappings[key])

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
            with self.env:
                final_data = {"env_states": self.data,"object_list": list(self.object_names)}
                path = Config.DATA_MISC_DIR+ self.env_name+"/"
                if not os.path.exists(path):
                    os.makedirs(path)
                cPickle.dump(final_data,open(Config.DATA_MISC_DIR+ self.env_name+"/"+ self.file_name ,"wb"),protocol=cPickle.HIGHEST_PROTOCOL)
                print("{} data saved".format(self.env_name))

        def random_config_robot(self,current_dof,allowed_range_x=[-0.5,6],allowed_range_y=[-12,-3],robot=None,exp=None):
            random_dof_values = []
            
            if len(current_dof) == 3:
                self.activate_base_joints()

                random_x = np.random.uniform(allowed_range_x[0],allowed_range_x[1])
                random_y = np.random.uniform(allowed_range_y[0],allowed_range_y[1])
                random_yaw   = np.random.uniform(-3.14, 3.14)

                random_dof_values =  [random_x,random_y,random_yaw]
                
            else:
                self.activate_manip_joints()
                lower_limits,upper_limits = self.robot.GetActiveDOFLimits()
                for i in range(len(current_dof)):
                    range_offset = abs(upper_limits[i]-lower_limits[i])/6.0
                    r = np.random.uniform(lower_limits[i]+range_offset,upper_limits[i]-range_offset)
                    random_dof_values.append(r)
                

            self.robot.SetActiveDOFValues(random_dof_values)
            if len(current_dof) == 3:
                t = self.robot.GetLink("base_link").GetTransform()
            else:
                t = self.gripper_link.GetTransform()
                
            if self.grabbed_object_1 is None:
                if self.collision_check([self.robot]):
                    random_dof_values,_ = self.random_config_robot(current_dof,allowed_range_x=allowed_range_x,allowed_range_y=allowed_range_y)
            else:
                grabbed_object = self.env.GetKinBody(self.grabbed_object_1)
                if self.collision_check([self.robot,grabbed_object]):
                    random_dof_values,_ = self.random_config_robot(current_dof,allowed_range_x=allowed_range_x,allowed_range_y=allowed_range_y)

            self.robot.SetActiveDOFValues(current_dof)

            return random_dof_values,t
            
        def compute_motion_plan(self,goal,robot=None):
            if len(goal) == 3:
                self.activate_base_joints()
            else:
                self.activate_manip_joints()

            try:
                traj = self.motion_planner.PlanToConfiguration(self.robot,goal)
                return traj

            # except PlanningError as e:
            except:
                # print(e)
                # print("could not find motion plan")
                return None

        def get_state_list(self,traj):
            state_list = []
            if type(traj) != list:
                len_traj = traj.GetNumWaypoints()
                for i in range(len_traj):
                    wp = traj.GetWaypoint(i)
                    if len(wp) == 3:
                        self.activate_base_joints()
                    else:
                        self.activate_manip_joints()
                    obj_dic = {}
                    self.robot.SetActiveDOFValues(wp)
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
            if len(dof_vals) == 3:
                quat = quatFromAxisAngle([0,0,dof_vals[-1]])
                pos = [dof_vals[0],dof_vals[1],0]
            else:
                quat = quatFromAxisAngle(dof_vals[3:])
                pos = dof_vals[:3]
            pose = []
            pose.extend(quat)
            pose.extend(pos)
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
            for pl in self.added_objects:
                if obj!=pl:
                    p_y = pl.GetTransform()[1,3]
                    p_x = pl.GetTransform()[0,3]
                    if math.sqrt((p_y-t_y)*(p_y-t_y) + (p_x-t_x)*(p_x-t_x)) < self.robot_offset:
                        return True
            
            return False

        def object_randomizer(self,obj_name,surface_name,range):
            surface_id = int(surface_name.split("_")[1])
            if surface_id == 0:
                surface_type = "countertop"
            else:
                surface_type = "table"
            sampler = getattr(self,"sample_can_{}".format(surface_type))

            obj = self.env.GetKinBody(obj_name)
            t = sampler(object_name=obj_name,surface_name=surface_name,range=range)

            if t is not None:
                obj.SetTransform(t)
                if str(obj.GetName()).split("_")[0] == Config.OBJECT_NAME[0]:
                    self.added_objects.append(obj)

                self.collision_set.add(obj)

                return obj.GetTransform()
        
            else:
                return None
        
        def spawn_object(self):
            radius = self.can_radius
            height = 0.2
            color = [0,0.8,1]
            body_name = Config.OBJECT_NAME[0] + "_{}".format(len(self.can_list)+1)

            t = matrixFromPose([1, 0, 0, 0, 0, 0, -0.5])
            cylinder = object_models.create_cylinder(self.env, body_name, t, [radius, height], color)

            self.env.Add(cylinder)
            self.can_list.append(cylinder)
            return t
        
        def spawn_goalLoc(self):
            goalLoc_obj = object_models.create_cylinder(self.env,'goalLoc_{}'.format(len(self.goalLoc_list)+1),np.eye(4),[0,0])
            self.env.Add(goalLoc_obj)
            self.goalLoc_list.append(goalLoc_obj)
            return 1
        
        def spawn_table(self):
            thickness = 0.1
            legheight = 0.55
            color = [1,0.8,0]
            pose = [3, -5, 0.63]

            name = "surface_{}".format(len(self.table_list)+1)        
            table = object_models.create_table(self.env,
                                            name,
                                            0.90, #dim1
                                            0.90, #dim2
                                            thickness,
                                            0.1, #legdim1
                                            0.1, #legdim2
                                            legheight,
                                            pose,
                                            color)
            
            self.env.Add(table)
            self.table_list.append(table)

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
            _,traj_config = self.randomize_env()

        def setup_exp(self,given_surface_lists=None,req_relation=None,experiment_flag=False):
            if not experiment_flag: 
                _,init_configs = self.randomize_env(req_relation=req_relation)
                init_state = self.get_one_state()

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

                goal_state = self.get_one_state()
                self.set_env_state(init_state)

                return init_state,goal_state,traj_config
            else:
                for can in self.can_list:
                    self.collision_set.add(can)
                for table in self.table_list:
                    self.collision_set.add(table)
        
    return CafeWorld