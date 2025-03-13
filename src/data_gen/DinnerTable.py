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
from Environments.DinnerTable import object_models
from src.robot_files.Robots import Fetch, MagicGripper
import math
import importlib
from time import sleep
import os

def GetDomain(robot_name="Fetch"):
    mod = importlib.import_module("src.robot_files.Robots")
    RobotClass = getattr(mod,robot_name)

    class DinnerTable(RobotClass):
        def __init__(self,env_name,number_of_configs,number_of_mp,axis_for_offset,file_name="_data.p",reference_structure_name=None,visualize = False,plank_count=None, random=False,order=False,planks_in_init_state=0,surface="",quadrant=None,grasp_num=None,minimum_plank_count=None,mp=True,num_robots=1,structure_dependance=False,object_list=["bowl","glass"],experiment_flag=False,real_world_experiment=False,set_y=False,complete_random=False,data_gen=False):
            self.env = orpy.Environment()
            self.env.SetDebugLevel(0)
            super(DinnerTable,self).__init__(env=self.env,id=1)
            
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

            self.clearance = 0.005
            # self.env_limits = self.get_env_limits()
            # self.env_y_limits = self.env_limits[1] #[y_min,y_max]
            # self.env_x_limits = self.env_limits[0] #[x_min,x_max]
            # self.env_z_limits = [self.env_limits[-1],1.0]

            self.collision = False
            self.grabbed_flag_1 = False
            self.object_list = []
            self.targetLoc_list = []
            self.quadrant = quadrant
            self.grasp_num = grasp_num
            self.match_quadrants = False
            
            self.object_name_list = object_list

            self.object_count = plank_count
            self.bowl_list = []
            self.glass_list = []
            self.table_list = []

            table_count = 2
            for _ in range(table_count):
                self.spawn_table()

            self.collision_set = set([self.robot])
            if real_world_experiment:
                self.env.Load(Config.REAL_EXPERIMENT_FILE)
                self.env.SetViewer('qtcoin')
                for obj in self.env.GetBodies():
                    if str(obj.GetName()).split("_")[0] in Config.OBJECT_NAME:
                        self.object_list.append(obj)
                        if "glass" in str(obj.GetName()):
                            self.glass_list.append(obj)
                        if "bowl" in str(obj.GetName()):
                            self.bowl_list.append(obj)

                for obj in self.env.GetBodies():
                    self.collision_set.add(obj)
            else:
                self.env.Load(Config.ENV_DIR + "env.dae")
                self.env.Remove(self.env.GetKinBody("countertop"))
            
            self.bound_object_name = ['world_final']
            if not real_world_experiment:
                for obj_name in self.bound_object_name:
                    self.collision_set.add(self.env.GetKinBody(obj_name))

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
            self.glass_radius = 0.035
            self.glass_h = 0.25
            self.bowl_radius = 0.085
            self.bowl_height = 0.07
            self.object_placement_h = 0.94

            self.large_range_x = [-0.5,6]
            self.large_range_y = [-12,-3]

            self.near_countertop_x = [-1.3,-0.3]
            self.near_countertop_y = [-0.4,0.8]
            
            self.random = random
            self.added_objects = []
            self.surface = surface
            self.num_robots = num_robots
            self.robot_1 = self
            self.robots = [self.robot_1]

            self.table_h = 0.63
            self.table_top_thickness = 0.1
            
            self.default_countertop_range = [[-0.175,0.175], [-0.175,0.175]]
            self.countertop_quadrant_ranges = [[[0.1,0.175],[0.1,0.175]],
                                               [[-0.175,-0.1],[0.1,0.175]],
                                               [[-0.175,-0.1],[-0.175,-0.1]],
                                               [[0.1,0.175],[-0.175,-0.1]]]
                                               
            self.default_table_range = [[-0.175,0.175], [-0.175,0.175]]
            self.table_quadrant_ranges = [[[0.1,0.175],[0.1,0.175]],
                                          [[-0.175,-0.1],[0.1,0.175]],
                                          [[-0.175,-0.1],[-0.175,-0.1]],
                                          [[0.1,0.175],[-0.175,-0.1]]]

            self.table_spawn_x_range = [3,7]
            self.table_spawn_y_range = [-4,3]

            self.near_countertop_x = [-1.3,-0.3]
            self.near_countertop_y = [-0.4,0.8]
            
            if visualize:
                self.env.SetViewer('qtcoin')

            if not experiment_flag:
                self.setup_env(current_env_num=0,env_list=[0])
                self.randomize_env()
            elif not real_world_experiment:
                self.setup_env(current_env_num=0,env_list=[0])

            self.trace = []
            self.compute_mp = mp
            self.experiment_flag = experiment_flag

            # while True:
            #     self.randomize_env()

            # self.discretizer = Config.get_discretizer("surface","glass")
            # env_data = self.load_traj_data("env2")
            # for traj in env_data["env_states"]:
            #     self.execute_traj(traj)

            # with open(Config.DATA_MISC_DIR+"200_0_rcr_indices.p") as f:
            #     indices_data = cPickle.load(f)
            # rgn = indices_data["rcr_dict"]["freight"]["glassinitLoc"]
            self.randomize_env()
            # self.debug_plotting(region=rgn,obj1="glassinitLoc", obj2="freight")
            # self.debug_plotting(region=rgn,obj1="freight", obj2="glassinitLoc")

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

        def setup_env(self,current_env_num,env_list=[]):
            for obj in self.object_name_list:
                for ob in self.env.GetBodies():
                    if obj in str(ob.GetName()):
                        self.env.Remove(ob)
                        if ob in self.object_list:
                            self.object_list.remove(ob)
                            if "glass" in str(ob.GetName()):
                                self.glass_list.remove(ob)
                            if "bowl" in str(ob.GetName()):
                                self.bowl_list.remove(ob)

            if current_env_num in env_list:
                objects_to_spawn = self.object_count
            else:
                objects_to_spawn = 1

            for i in range(objects_to_spawn):
                for obj in self.object_name_list:
                    if obj == "bowl":
                        self.spawn_bowl()
                    if obj == "glass":
                        self.spawn_glass()

            for obj in self.object_list:
                self.spawn_targetLoc(obj)

        def load_rcr_data(self):
            with open(Config.DATA_MISC_DIR+"4000_Keva_1.p","rb") as f:
                data_dict = cPickle.load(f)
                f.close()

            return data_dict["rcrs"]

        def show_region(self,region,pT,sample_count=100):
            discretizer = Config.get_discretizer("plank","plank")
            for i in range(sample_count):
                rel_pose = discretizer.convert_sample(region[0][:6],is_relative=True)
                rel_t = self.transform_from_wp(rel_pose)
                p = pT.dot(np.linalg.pinv(rel_t))
                self.trace.append(self.env.plot3(points = [p[0,3],p[1,3],p[2,3]], pointsize = 0.002, colors = np.array([255,255,0]), drawstyle = 1 ))

        def remove_traces(self):
            for t in self.trace:
                t.Close()               

        def execute_traj(self,traj):
            for i,state in enumerate(traj):
                self.set_env_state(state)
                sleep(0.01)

        def load_traj_data(self,env_name):
            with open("Data/DinnerTable/misc/{}/{}_data.p".format(env_name,env_name),"rb") as f:
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
                table_pose = state.object_dict["surface_1"]
                counter_pose = state.object_dict["countertop"]
                rp1 = state.get_relative_pose(object_pose,gripper_pose)
                rp2 = state.get_relative_pose(table_pose,base_pose)
                rp3 = state.get_relative_pose(counter_pose,base_pose)
                current_object_pose = self.env.GetKinBody("can_1").GetTransform()
                current_table_pose = self.env.GetKinBody("surface_1").GetTransform()
                current_counter_pose = self.env.GetKinBody("countertop").GetTransform()
                p1 = current_object_pose.dot(state.transform_from_pose(rp1))
                p2 = current_table_pose.dot(state.transform_from_pose(rp2))
                p3 = current_counter_pose.dot(state.transform_from_pose(rp3))
                self.trace.append(self.env.plot3(points = [p1[0,3],p1[1,3],p1[2,3]], pointsize = 0.05, colors = np.array([255,0,0]), drawstyle = 1 ))
                self.trace.append(self.env.plot3(points = [p2[0,3],p2[1,3],p2[2,3]], pointsize = 0.05, colors = np.array([0,255,0]), drawstyle = 1 ))
                self.trace.append(self.env.plot3(points = [p3[0,3],p3[1,3],p3[2,3]], pointsize = 0.05, colors = np.array([0,0,255]), drawstyle = 1 ))

        def debug_plotting(self,region,obj1,obj2):
            trace = []
            discretizer = Config.get_discretizer(obj1,obj2)
            env_state = self.get_one_state()
            for obj in env_state.object_dict.keys():
                if obj.split("_")[0] == obj1:
                    obj1_pose = env_state.object_dict[obj]
                    if obj1 in Config.ROBOT_TYPES:
                        obj1_pose = obj1_pose[1]
                    obj1_T = self.transform_from_wp(obj1_pose)

                if obj.split("_")[0] == obj2:
                    obj2_pose = env_state.object_dict[obj]
                    if obj2 in Config.ROBOT_TYPES:
                        obj2_pose = obj2_pose[1]
                    obj2_T = self.transform_from_wp(obj2_pose)
            
            xyz_set = set()
            for rgn in region:
                for ind in rgn:
                    try:
                        converted_sample = discretizer.convert_sample(ind[:6],is_relative=True)
                    except:
                        converted_sample = discretizer.convert_sample(ind[:3],is_relative=True)
                        converted_sample.extend(self.get_pose_from_transform(np.eye(4))[3:])
                    converted_T = self.transform_from_wp(converted_sample)

                    grounded_T = obj1_T.dot(np.linalg.pinv(converted_T))
                    # grounded_T = obj1_T.dot(converted_T)
                    x,y,z = grounded_T[:3,3]
                    x = round(round(x/0.05)*0.05,2)
                    y = round(round(y/0.05)*0.05,2)
                    z = round(round(z/0.05)*0.05,2)
                    xyz_set.add(tuple([x,y,z]))
            
            for i in xyz_set:
                i = list(i)
                trace.append(self.env.plot3(points = [i[0],i[1],i[2]], pointsize = 0.05, colors = np.array([0,255,0]), drawstyle = 1 ))

            return None

        def sample_obj_table(self,object_name,surface_name,range=[[-0.43,-0.1],[0.1,0.43]]):
            current_grabbed_flag = False
            if self.grabbed_flag_1:
                current_grabbed_flag = True
                grabbed_object = deepcopy(self.grabbed_object_1)
                self.release()

            obj_dims = self.get_object_dims(object_name=object_name)
            obj = self.env.GetKinBody(object_name)

            table = self.env.GetKinBody(surface_name)
            table_t = table.GetTransform()

            obj_t = np.eye(4)
            obj_t[2,3] += self.table_top_thickness/2.0 + self.clearance
            obj_t[1,3] += np.random.uniform(low=range[1][0]+obj_dims[1],high=range[1][1]-obj_dims[1])

            grasp_num = self.grasp_num
            if self.grasp_num is None:
                grasp_num = np.random.choice(Config.NUM_GRASPS)
            rot_angle = (grasp_num * (2*np.pi) / Config.NUM_GRASPS)
            rot_z = matrixFromAxisAngle([0,0,rot_angle])

            dt = np.eye(4)
            dt[0,3] = -0.35

            t = table_t.dot(obj_t).dot(rot_z).dot(dt)
            # obj.SetTransform(t)
            return t

        def sample_obj_countertop(self,object_name,surface_name,range=[[-0.43,-0.1],[0.1,0.43]]):
            range=[[-0.43,-0.1],[0.1,0.43]]
            current_grabbed_flag = False
            if self.grabbed_flag_1:
                current_grabbed_flag = True
                grabbed_object = deepcopy(self.grabbed_object_1)
                self.release()

            obj_dims = self.get_object_dims(object_name=object_name)
            obj = self.env.GetKinBody(object_name)

            table = self.env.GetKinBody(surface_name)
            table_t = table.GetTransform()

            obj_t = np.eye(4)
            obj_t[2,3] += self.clearance 
            obj_t[1,3] += np.random.uniform(low=range[1][0],high=range[1][1])

            # grasp_num = np.random.choice(Config.NUM_GRASPS)
            rot_angle = ((2*np.pi) / Config.NUM_GRASPS)
            rot_z = matrixFromAxisAngle([0,0,rot_angle])

            dt = np.eye(4)
            dt[0,3] = -0.35

            t = table_t.dot(obj_t).dot(rot_z).dot(dt)
            # obj.SetTransform(t)
            return t

        def sample_robot_base(self,object_name,mp=False):
            self.activate_base_joints()
            current_dof_vals = self.robot.GetActiveDOFValues()

            diff = 0.8
            table = self.env.GetKinBody(object_name)
            table_t = table.GetTransform()
            diff_translation_matrix = matrixFromPose([1,0,0,0,-diff,0,0])

            valid_pose = None
            count = 0
                
            while valid_pose is None and count < 5:
                t = table_t.dot(diff_translation_matrix)
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
            wrist_roll_pose = self.robot.GetLink('wrist_roll_link').GetTransform()
            gripper_pose = self.robot.GetLink('gripper_link').GetTransform()
            wrist_pose_wrt_gripper = np.matmul(np.linalg.inv(gripper_pose), wrist_roll_pose)

            valid_pose = None
            if "bowl" not in object_name:
                rot_Z = matrixFromAxisAngle([0, 0, -np.pi/2])
                gripper_offset = self.grasping_offset[object_name.split("_")[0]]
                rot_ang = ((2*np.pi) / Config.NUM_GRASPS)
                # print(rot_ang)
                obj_T_gripper = matrixFromPose([1, 0, 0, 0, gripper_offset, 0, self.glass_h/2.0-0.01])
                rot_mat = matrixFromAxisAngle([0, 0, rot_ang])

                grasp_T = world_T_obj.dot(rot_mat).dot(rot_Z).dot(obj_T_gripper)
                grasp_T = np.matmul(grasp_T,wrist_pose_wrt_gripper)
                
                ik_sols = self.get_ik_solutions(grasp_T,collision_fn=self.collision_check)
                if len(ik_sols) > 0:
                    valid_pose = ik_sols

            else:
                t = np.eye(4)
                t1 = matrixFromPose([1,0,0,0,-0.085,0,0.24])
                r1 = matrixFromAxisAngle([0,math.pi/2.0,0])
                r2 = matrixFromAxisAngle([math.pi/2.0,0,0])

                grasp_T = world_T_obj.dot(t1).dot(r1).dot(r2)
                ik_sols = self.get_ik_solutions(grasp_T,collision_fn=self.collision_check)
                if len(ik_sols) > 0:
                    valid_pose = ik_sols
            
            return valid_pose

        def sample_goal_pose(self,object_name):            
            count = 0
            valid_ik = None
            
            while count < Config.MAX_IK_ATTEMPTS and valid_ik is None:
                sample_obj_pose = self.env.GetKinBody("{}targetLoc_{}".format(object_name.split("_")[0],object_name.split("_")[1])).GetTransform()                
                grasp_pose = self.sample_grasp_pose(object_name=object_name,pose=sample_obj_pose)
                if grasp_pose is not None:
                    valid_ik = grasp_pose
                    break
                count+=1

            return valid_ik
        
        def set_targetLoc(self):
            remove_object_list = []
            for obj in self.object_list:
                obj_name = str(obj.GetName())
                obj_id = obj_name.split("_")[1]

                targetLoc = self.env.GetKinBody("{}targetLoc_{}".format(obj_name.split("_")[0],obj_id))
                current_obj_t = obj.GetTransform()
                self.added_objects.append(targetLoc)
                remove_object_list.append(targetLoc)
                goal_obj_t = self.object_randomizer(obj_name)
                obj.SetTransform(current_obj_t)

                targetLoc.SetTransform(goal_obj_t)
            
            for obj in remove_object_list:
                self.added_objects.remove(obj)

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

        def random_place_robot(self,object_name=None,mp=False,range=[[-0.5,6],[-12,-3]]):
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
        
        def get_random_range(self):
            max_y = -100000000
            for obj in self.env.GetBodies():
                if Config.SURFACE_NAME in str(obj.GetName()) and int(str(obj.GetName()).split("_")[1])>0:
                    max_y = max(obj.GetTransform()[1,3],max_y)
            
            range = [self.large_range_x,([max_y+1.5,-1.2])]

            return range

        def get_possible_configs(self,num_objects=1,configs_to_exclude=set([])):
            possible_configs=[]

            random_cans = []
            while len(random_cans) < 2 and np.random.randint(0,2) and num_objects > 1:
                can_num = np.random.randint(1,num_objects)
                if can_num not in random_cans:
                    random_cans.append(can_num)

            for can_num in random_cans:
                possible_configs.append(("{}_{}".format(self.object_name_list[0],can_num),"surface_0"))

            for i in range(1,len(self.table_list)+1):
                possible_configs.extend([("{}_{}".format(self.object_name_list[0],can_num),"surface_{}".format(i)) for can_num in range(1,num_objects+1)])
            
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
                if init_surface_id == 0:
                    init_surface_type = "countertop"
                else:
                    init_surface_type = "table"
                
                sample_quadrant_range = getattr(self,"default_{}_range".format(init_surface_type))
                t = self.object_randomizer(obj_name=can_name,surface_name=init_surface_name,range=sample_quadrant_range)
                if t is None:
                    possible_configs.append(init_config)
                    continue
                else:
                    init_configs.append(init_config)
                
            if traj_config is None:
                traj_config = self.get_traj_config(init_configs,possible_configs)
            
            for _,goal_config in traj_config:
                can_name = goal_config[0]
                can = self.env.GetKinBody(can_name)
                goal_surface_name = goal_config[1]
                goal_surface_id = int(goal_surface_name.split("_")[1])
                if goal_surface_id == 0:
                    goal_surface_type = "countertop"
                else:
                    goal_surface_type = "table"

                sample_quadrant_range = getattr(self,"default_{}_range".format(goal_surface_type))
                t = self.object_randomizer(obj_name=can_name,surface_name=goal_surface_name,range=sample_quadrant_range,init_config=False)

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

        def motion_plan_robot_base(self,object_name=None,given_pose=None):
            self.activate_base_joints()
            
            if object_name is not None:
                sampler = self.sample_robot_base
            else:
                surface_name = None
                sampler = self.random_place_robot

            traj = None
            count=0
            while traj is None and count<5:
                target_pose = None
                if given_pose is None:
                    target_pose = sampler(object_name=object_name,mp=True)
                else:
                    target_pose = given_pose

                if target_pose is not None:
                    if self.compute_mp:
                        traj = self.compute_motion_plan(target_pose)
                    else:
                        traj = [target_pose]
                
                count += 1
            
            return traj, target_pose
        
        def motion_plan_robot_arm(self,object_name=None,pose=[],grasp_num=None,given_pose=None,surface_id=None):
            self.activate_manip_joints()
            
            if object_name is not None:
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

        def get_state_block(self,traj,object_name,grab=None):
            time.sleep(0.02)
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
            envlist = np.random.choice(list(range(1,self.n+1)),int(0.3*self.n))
            with self.env:
            # if True:
                if self.env.GetViewer() is not None:
                    self.set_camera_wrt_obj("world_final")
                while i < self.n:
                    # print("starting of i loop i={}".format(i))
                    # self.setup_env(current_env_num=i+1, env_list=envlist)
                    init_object_dic,traj_config = self.randomize_env(traj_count=i)
                    while j <= self.j:
                        # print("start of j loop j={}".format(j))
                        state_list = []
                        # self.setup_objects(init_object_dic)
                        # for config in traj_config:
                        #     print(config[0],config[1])

                        for obj in self.object_list:
                            #going to init_surface
                            # print(init_config[0])
                            object_name = str(obj.GetName())
                            targetLoc = self.env.GetKinBody("{}targetLoc_{}".format(object_name.split("_")[0],object_name.split("_")[1]))
                            targetLoc_name = str(targetLoc.GetName())
                            initLoc = self.env.GetKinBody("{}initLoc_{}".format(object_name.split("_")[0],object_name.split("_")[1]))
                            initLoc_name = str(initLoc.GetName())
                            #KP_1
                            state_1 = [self.get_one_state()]

                            # print("sampling surface 1")
                            base_sample_1 = self.sample_robot_base(object_name=initLoc_name)
                                
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
                            traj_2,_ = self.motion_plan_robot_arm(object_name=object_name)

                            if traj_2 is None:
                                if j == 0:
                                    flag=1
                                else:
                                    flag=2
                                break
                            
                            self.activate_base_joints()
                            self.robot.SetActiveDOFValues(init_base_pose)
                            # print("motion planning for base to surface 1")
                            traj_1, base_sample_1 = self.motion_plan_robot_base(object_name=initLoc_name,given_pose=base_sample_1)
                            if traj_1 is None:
                                if j == 0:
                                    flag=1
                                else:
                                    flag=2
                                break
                            
                            #KP_2
                            state_2 = self.get_state_block(traj=traj_1,object_name=object_name)
                            #KP_3
                            state_3 = []
                            if not self.compute_mp:
                                self.activate_manip_joints()
                                current_dof = self.robot.GetActiveDOFValues()
                                random_pose,_ = self.random_config_robot(current_dof=current_dof)
                                state_3 = self.get_state_block(traj=[random_pose],object_name=object_name)

                            #KP_4 and KP_5
                            state_4_5 = self.get_state_block(traj=traj_2,object_name=object_name,grab=True)
                            #KP_6
                            state_6 = []
                            if not self.compute_mp:
                                self.activate_manip_joints()
                                current_dof = self.robot.GetActiveDOFValues()
                                random_pose,_ = self.random_config_robot(current_dof=current_dof)
                                state_6 = self.get_state_block(traj=[random_pose],object_name=object_name)

                            # print("tucking arm with object")
                            traj_3, _ = self.motion_plan_robot_arm()
                            if traj_3 is None:
                                if j == 0:
                                    flag=1
                                else:
                                    flag=2
                                break
                                
                            #KP_7
                            state_7 = self.get_state_block(traj=traj_3,object_name=object_name)
                            
                            #KP_8
                            state_8 = []
                            if not self.compute_mp:
                                self.activate_base_joints()
                                current_dof = self.robot.GetActiveDOFValues()
                                random_pose,_ = self.random_config_robot(current_dof=current_dof)
                                state_8 = self.get_state_block(traj=[random_pose],object_name=object_name)

                            # print("sampling surface 2")
                            base_sampler = self.sample_robot_base
                            base_sample_2 = base_sampler(object_name=targetLoc_name)
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
                                ik = self.sample_goal_pose(object_name)
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
                            traj_4, base_sample_2 = self.motion_plan_robot_base(object_name=targetLoc_name,given_pose=base_sample_2)
                            if traj_4 is None:
                                if j == 0:
                                    flag=1
                                else:
                                    flag=2
                                break
                                
                            #KP_9
                            state_9 = self.get_state_block(traj=traj_4,object_name=object_name)
                            #KP_10
                            state_10 = []
                            if not self.compute_mp:
                                self.activate_manip_joints()
                                current_dof = self.robot.GetActiveDOFValues()
                                random_pose,_ = self.random_config_robot(current_dof=current_dof)
                                state_10 = self.get_state_block(traj=[random_pose],object_name=object_name)
                            
                            #KP_11 and KP_12
                            state_11_12 = self.get_state_block(traj=traj_5,object_name=object_name,grab=False)

                            #KP_13
                            state_13 = []
                            if not self.compute_mp:
                                self.activate_manip_joints()
                                current_dof = self.robot.GetActiveDOFValues()
                                random_pose,_ = self.random_config_robot(current_dof=current_dof)
                                state_13 = self.get_state_block(traj=[random_pose],object_name=object_name)

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
                            state_14 = self.get_state_block(traj=traj_6,object_name=object_name)

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
                            state_15 = self.get_state_block(traj=traj_7,object_name=object_name)
                            
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

        def random_config_robot(self,current_dof,allowed_range_x=[0.5,6],allowed_range_y=[-12,-3],robot=None):
            random_dof_values = []
            
            if len(current_dof) == 3:
                self.activate_base_joints()

                if not self.experiment_flag:
                    random_x = np.random.uniform(allowed_range_x[0],allowed_range_x[1])
                    random_y = np.random.uniform(allowed_range_y[0],allowed_range_y[1])
                    random_yaw = np.random.uniform(-3.14, 3.14)

                else:
                    current_T = self.env.GetRobots()[0].GetTransform()
                    t = np.eye(4)
                    t[0,3] -= 0.5
                    t = current_T.dot(t)
                    
                    random_x = t[0,3]
                    random_y = t[1,3]
                    random_yaw = axisAngleFromRotationMatrix(t[:3,:3])[-1]

                random_dof_values =  [random_x,random_y,random_yaw]
                
            else:
                self.activate_manip_joints()
                lower_limits,upper_limits = self.robot.GetActiveDOFLimits()
                if not self.experiment_flag:
                    for i in range(len(current_dof)):
                        range_offset = abs(upper_limits[i]-lower_limits[i])/6.0
                        r = np.random.uniform(lower_limits[i]+range_offset,upper_limits[i]-range_offset)
                        random_dof_values.append(r)
                else:
                    random_dof_values = [0.15, 0.72466344, -0.05064385, -1.73952133, 2.25099986, -1.50486781, -0.02543545, 2.76926565]

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

            return random_dof_values, t
            
        def compute_motion_plan(self,goal,robot=None):
            if len(goal) == 3:
                self.activate_base_joints()
            else:
                self.activate_manip_joints()

            try:
                traj = self.motion_planner.PlanToConfiguration(self.robot,goal)
                return traj

            except PlanningError as e:
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
                    if math.sqrt((p_y-t_y)*(p_y-t_y) + (p_x-t_x)*(p_x-t_x)) < 1.5:
                        return True
            
            return False

        def object_randomizer(self,obj_name,surface_name,range,init_config=True):
            obj = self.env.GetKinBody(obj_name)
            obj_type = obj_name.split("_")[0]
            surface_id = int(surface_name.split("_")[1])
            if surface_id == 0:
                surface_type = "countertop"
            else:
                surface_type = "table"
            sampler = getattr(self,"sample_obj_{}".format(surface_type))

            t = sampler(object_name=obj_name,surface_name=surface_name,range=range)
            current_t = obj.GetTransform()
            if t is not None:
                if str(obj.GetName()).split("_")[0] in Config.OBJECT_NAME:
                    self.added_objects.append(obj)
                    if init_config:
                        Loc_obj = self.env.GetKinBody("{}initLoc_{}".format(obj.GetName().split("_")[0],obj.GetName().split("_")[1]))
                        obj.SetTransform(t)
                    else:
                        Loc_obj = self.env.GetKinBody("{}targetLoc_{}".format(obj.GetName().split("_")[0],obj.GetName().split("_")[1]))

                    Loc_obj.SetTransform(t)

                self.collision_set.add(obj)

                return obj.GetTransform()
        
            else:
                return None
        
        def spawn_glass(self):
            radius = 0.0425
            height = 0.24
            color = [0,0.8,1]
            body_name = "glass"+ "_{}".format(len(self.glass_list)+1)

            t = matrixFromPose([1, 0, 0, 0, 0, 0, -0.5])
            cylinder = object_models.create_cylinder(self.env, body_name, t, [radius, height], color)

            self.env.Add(cylinder)
            self.glass_list.append(cylinder)
            self.object_list.append(cylinder)
            return t

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

        def spawn_bowl(self):
            body_name = "bowl" + "_{}".format(len(self.bowl_list)+1)

            t = matrixFromPose([1, 0, 0, 0, 0, 0, -0.5])
            bowl = self.env.ReadKinBodyXMLFile(Config.OBJECTS_DIR+"MCan.stl")

            bowl.SetName(body_name)
            bowl.SetTransform(t)
            self.env.Add(bowl)
            self.bowl_list.append(bowl)
            self.object_list.append(bowl)
            return t
        
        def spawn_targetLoc(self,obj):
            object_name = str(obj.GetName())
            targetLoc_obj = object_models.create_cylinder(self.env,'{}targetLoc_{}'.format(object_name.split("_")[0],object_name.split("_")[1]),np.eye(4),[0,0])
            initLoc_obj = object_models.create_cylinder(self.env,'{}initLoc_{}'.format(object_name.split("_")[0],object_name.split("_")[1]),np.eye(4),[0,0])
            self.env.Add(targetLoc_obj)
            self.env.Add(initLoc_obj)
            self.targetLoc_list.append((targetLoc_obj,initLoc_obj))
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
           self.randomize_env()

        def setup_exp(self,given_surface_lists=None,req_relation=None,experiment_flag=False):
            if not self.experiment_flag:
                self.randomize_env(given_surface_lists)
            init_state = self.get_one_state()

            for obj in self.object_list:
                obj.SetTransform(self.env.GetKinBody("{}targetLoc_{}".format(str(obj.GetName()).split("_")[0],str(obj.GetName()).split("_")[1])).GetTransform())

            goal_state = self.get_one_state()

            return init_state,goal_state,None
        
    return DinnerTable