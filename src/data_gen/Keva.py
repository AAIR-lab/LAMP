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
from prpy.planning import ompl, BiRRTPlanner, CBiRRTPlanner
from copy import deepcopy
from Environments.Keva import object_models
from src.robot_files.Robots import Fetch, MagicGripper
import math
import importlib
from src.useful_functions import *
import os

def GetDomain(robot_name="yumi"):
    mod = importlib.import_module("src.robot_files.Robots")
    RobotClass = getattr(mod,robot_name)

    class Keva(RobotClass):
        def __init__(self,env_name,number_of_configs,number_of_mp,axis_for_offset,file_name="_data.p",reference_structure_name=None,visualize = False,plank_count=None, random=False,order=False,surface="",planks_in_init_state=0,quadrant=None,grasp_num=None,minimum_plank_count=None,mp=True,num_robots=1,structure_dependance=False,object_list=[],experiment_flag=False,real_world_experiment=False,set_y=False,complete_random=False,data_gen=True):
            self.data_gen_flag = data_gen
            self.env = orpy.Environment()
            self.env.SetDebugLevel(0)
            super(Keva,self).__init__(env=self.env,id=1,collection=True)

            self.order = order
            self.env_name = env_name
            print(self.env_name)
            self.reference_env = orpy.Environment()
            default_reference_flag = False
            successful_load = False
            if reference_structure_name is None:
                successful_load = self.reference_env.Load(Config.PLANK_PAIRS_DIR + "{}.dae".format(env_name))
            else:
                if "env" in reference_structure_name:
                    successful_load = self.reference_env.Load(Config.PLANK_PAIRS_DIR + reference_structure_name + ".dae")
                else:    
                    successful_load = self.reference_env.Load(Config.REFERENCE_DIR + reference_structure_name + ".dae")
            if not successful_load:
                default_reference_flag = True
                if "problem" in env_name:
                    self.reference_env.Load(Config.ENV_DIR + "{}.dae".format(env_name))
                else:
                    self.reference_env.Load(Config.PLANK_PAIRS_DIR + "env.dae")
            self.n = number_of_configs
            self.j = number_of_mp
            self.data = []
            self.envs = []
            self.seeds = []
            self.object_names = set()
            self.init_config = {}
            self.env.Load(Config.ENV_DIR +"env.dae")
            drop = self.env.GetKinBody("drop_area")
            drop.SetName("droparea")
            drop_t = drop.GetTransform()
            self.env.Remove(drop)
            drop = object_models.create_flat_area(self.env,"droparea",[0.99,0.99,0.99],t=drop_t)
            spawn = self.env.GetKinBody("spawn_area")
            spawn.SetName("spawnarea")
            self.env.Remove(spawn)
            self.env_obstacle_num = 10
            self.bound_object_name = ['table6']
            self.collision_set = set([drop,spawn])
            for obj_name in self.bound_object_name:
                self.collision_set.add(self.env.GetKinBody(obj_name))
            self.clearance = 0.0
            self.obj_h = 0.05
            self.env_limits = self.get_env_limits()
            self.env_y_limits = self.env_limits[1] #[y_min,y_max]
            self.env_x_limits = self.env_limits[0] #[x_min,x_max]
            self.env_z_limits = [self.env_limits[-1],1.0]
            self.region_h = utils.get_object_limits(self.env.GetKinBody('droparea'))[-1] + self.clearance
            self.region_dim = 0.075
            self.collision = False
            self.plank_list = []
            self.goalLoc_list = []
            cc = orpy.RaveCreateCollisionChecker(self.env,'pqp')
            cc.SetCollisionOptions(orpy.CollisionOptions.AllGeometryContacts)
            self.env.SetCollisionChecker(cc)
            self.file_name = file_name
            self.motion_planner = BiRRTPlanner()
            self.delta_planner = CBiRRTPlanner()
            if robot_name != "yumi":
                self.random_place_robot()
                # self.set_init_pose()
            self.store_init_config()
            self.axis_for_offset = axis_for_offset
            self.base_offset = Config.AXIS_MAP[Config.DOMAIN_NAME][axis_for_offset]
            if type(random) != bool:
                self.random = random>=0
            elif random is not None:
                self.random = random
            
            self.complete_random = complete_random

            self.planks_in_init_state = planks_in_init_state
            self.added_planks = []
            self.compute_mp = mp
            self.num_robots = num_robots
            self.robot_1 = self
            self.robots = [self.robot_1]
            self.set_y = set_y

            if visualize:
                self.env.SetViewer('qtcoin')
            
            if plank_count is None:
                self.plank_count = len(self.reference_env.GetBodies())
                self.no_of_planks_in_loop = self.plank_count

            elif plank_count > len(self.reference_env.GetBodies()) > 0:
                self.no_of_planks_in_loop = len(self.reference_env.GetBodies())
                self.plank_count=plank_count

            else:
                self.plank_count=plank_count
                self.no_of_planks_in_loop = self.plank_count

            self.minimum_plank_count = minimum_plank_count
            if self.minimum_plank_count is None:
                self.minimum_plank_count = self.no_of_planks_in_loop

            # for i in range(max(2,self.plank_count)):
            for i in range(self.plank_count):
                self.spawn_goalLoc()
                self.spawn_plank()
            
            if random > 0:
                self.num_planks_random = random
            elif random == 0 and type(random) != bool:
                self.num_planks_random = self.plank_count
            else:
                self.num_planks_random = 0

            if "problem" in env_name:
                self.set_test()

            self.experiment_flag = experiment_flag
            # traj_set = None
            # with open(Config.DATA_MISC_DIR+"env5/env5_data.p") as f:
            #     traj_set = cPickle.load(f)["env_states"]
            #     f.close()

            # self.execute_traj(traj_set[0])

        def execute_traj(self, traj):
            for t in traj:
                self.set_env_state(t)
                time.sleep(0.01)
            
            return True

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
        
        def load_rcr_data(self):
            with open(Config.DATA_MISC_DIR+"300_Keva_4.p","rb") as f:
                data_dict = cPickle.load(f)
                f.close()

            return data_dict["rcrs"]

        def show_region(self,region,pT,sample_count=100):
            discretizer = Config.get_discretizer("plank","plank")
            for i in range(sample_count):
                rel_pose = discretizer.convert_sample(region[0][:6],is_relative=True)
                rel_t = self.transform_from_pose(rel_pose)
                p = pT.dot(np.linalg.pinv(rel_t))
                self.trace.append(self.env.plot3(points = [p[0,3],p[1,3],p[2,3]], pointsize = 0.002, colors = np.array([255,255,0]), drawstyle = 1 ))

        def remove_traces(self):
            for t in self.trace:
                t.Close()               

        def store_init_config(self):
            bodies = self.env.GetBodies()
            for body in bodies:
                if body.GetName() != self.robot_name:
                    self.init_config[body.GetName()] = body.GetTransform()

        def get_relative_pose(self, pose1, pose2):
            #obj2 w.r.t. obj1
            transform1 = self.transform_from_pose(pose1)
            transform2 = self.transform_from_pose(pose2)
            return self.get_pose_from_transform((np.linalg.pinv(transform1).dot(transform2)))

        def sample_grasp_pose(self,object_name="",pose = []):
            if pose != []:
                world_T_obj = pose
            else:
                world_T_obj = self.env.GetKinBody(object_name).GetTransform()

            obj_T_robot = np.eye(4)
            obj_T_robot[1,3]= self.grasping_offset[Config.OBJECT_NAME[0]]
            
            t1 = orpy.matrixFromAxisAngle([ 0, -np.pi/2, 0])
            t2 = orpy.matrixFromAxisAngle([-np.pi/2, 0, 0])

            obj_T_robot = np.matmul(np.matmul(obj_T_robot,t1),t2)
            t = np.matmul(world_T_obj,obj_T_robot)
            pose = self.get_pose_from_transform(t)
            
            return pose

        def set_goalLoc(self,planks_to_set=None):
            goalLoc_list = []
            if planks_to_set is None:
                planks_to_set = self.no_of_planks_in_loop
            
            for i in range(planks_to_set-self.num_planks_random):
                goalLoc_list.append(self.env.GetKinBody(Config.LOCATION_NAME[2]+"_{}".format(i+1)))
                            
            droparea = self.env.GetKinBody("droparea")
            for i in range(planks_to_set-self.num_planks_random):
                obj = self.reference_env.GetKinBody(Config.OBJECT_NAME[0]+"{}".format(i+1))
                if obj is None:
                    obj = self.reference_env.GetKinBody(Config.LOCATION_NAME[2]+"_{}".format(i+1))
                t = obj.GetTransform()
                y_offset = np.eye(4)
                y_offset[1,3] = 0.0
                goal_transform = np.matmul(droparea.GetTransform().dot(y_offset),t)
                z_offset = np.eye(4)
                z_offset[2,3] += (self.clearance+self.base_offset)
                goal_transform = z_offset.dot(goal_transform)
                goalLoc = goalLoc_list[i]
                if self.set_y:
                    goal_transform = self.set_goalLoc_y(goal_transform)

                goalLoc.SetTransform(goal_transform)
                self.added_planks.append(goalLoc)
            
            if self.num_planks_random > 0:
                for j in range(self.num_planks_random):
                    i = len(goalLoc_list) + j
                    goalLoc = self.env.GetKinBody(Config.LOCATION_NAME[2]+"_{}".format(i+1))
                    rot = orpy.matrixFromAxisAngle([ 0, 0, -np.pi/2])
                    t = self.object_randomizer(goalLoc)
                    t = t.dot(rot)
                    t[2,3] += (self.clearance+self.base_offset)
                    if self.set_y:
                        t = self.set_goalLoc_y(t)
                    goalLoc.SetTransform(t)
                    goalLoc_list.append(goalLoc)
                    self.added_planks.append(goalLoc)
            
            for obj in goalLoc_list:
                self.added_planks.remove(obj)

            if self.order is not True:
                np.random.shuffle(goalLoc_list)
            
            return goalLoc_list

        def set_random_goalLoc(self,x_offsets=[0.02,0.02],y_offsets=[0.02,0.02]):
            rot = orpy.matrixFromAxisAngle([ 0, 0, -np.pi/2])
            goalLoc_list = []
            for i in range(self.plank_count):
                goalLoc = self.env.GetKinBody(Config.LOCATION_NAME[2]+"_{}".format(i+1))
                self.added_planks.append(goalLoc)
                t = self.object_randomizer(goalLoc,x_offsets=x_offsets,y_offsets=y_offsets)
                t = t.dot(rot)
                t[2,3] += (self.clearance+self.base_offset)
                if self.set_y:
                    t = self.set_goalLoc_y(t)
                goalLoc.SetTransform(t)
                goalLoc_list.append(goalLoc)

            for obj in goalLoc_list:
                self.added_planks.remove(obj)

            np.random.shuffle(goalLoc_list)        
            return goalLoc_list

        def set_goalLoc_y(self,current_t):
            t = matrixFromAxisAngle([0,0,np.pi/2])
            final_t = current_t.dot(t)
            final_t[2,3] = (self.clearance+Config.AXIS_MAP[Config.DOMAIN_NAME]["y"]) + 0.001

            # goalLoc.SetTransform(final_t)
            return final_t
        
        def get_init_pose(self):
            return self.init_pose

        def set_to_last_waypoint(self,trajectory):
            if type(trajectory) != list:
                num = trajectory.GetNumWaypoints()
                last_wp = trajectory.GetWaypoint(num-1)
            else:
                last_wp = trajectory[0]
            self.robot.SetActiveDOFValues(last_wp)
                
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
        
        def randomize_planks(self,x_offsets=[-20,-20],y_offsets=[-20,-20]):
            self.release()
            plank_transform_dict = {}
            to_remove_list = []
            for obj in self.collision_set:
                if obj.GetName().split("_")[0] in Config.OBJECT_NAME:
                    to_remove_list.append(obj)

            for obj in to_remove_list:
                self.collision_set.remove(obj)

            for plank in self.plank_list:
                if self.data_gen_flag:
                    x_offsets = [0.1,0.1]
                    y_offsets = [0.1,0.1]
                t = self.object_randomizer(plank,x_offsets,y_offsets)
                transform = np.eye(4)
                if self.robot_name != "yumi":
                    transform[2,3] = 0.012
                else:
                    transform[2,3] = -1.5
                t = transform.dot(t)
                plank.SetTransform(t)
                plank_transform_dict[plank] = t
                self.collision_set.add(plank)

            return plank_transform_dict
        
        def reset_planks(self,tranform_dict):
            self.release()
            for p in tranform_dict.keys():
                p.SetTransform(tranform_dict[p])
        
            return True
        
        def randomize_env(self,x_offsets=[-20,-20],y_offsets=[-20,-20],traj_config=None):
            self.added_planks = []
            transform_dict = self.randomize_planks(x_offsets,y_offsets)
            self.random_place_robot()
            if self.env.GetKinBody("droparea") is not None:
                self.drop_randomizer()

            if not(self.collision):
                if self.env.GetKinBody("droparea") is not None:
                    t = self.env.GetKinBody('droparea').GetTransform()
                    t[2,3] = 0.0
                    self.env.GetKinBody('droparea').SetTransform(t)
            
            for obj in self.bound_object_name:
                self.collision_set.add(self.env.GetKinBody(obj))   

            return transform_dict

        def set_camera_wrt_plank(self,plank_num,transform_num=1):
            plank = self.env.GetKinBody("plank_{}".format(plank_num))
            relative_t = np.load(Config.CAMERA_DIR+"keva_relative_camera_{}.npy".format(transform_num))
            self.env.GetViewer().SetCamera(plank.GetTransform().dot(relative_t))

        def save_camera_angle_wrt_plank(self,plank_num,transform_num=0):
            plank = self.env.GetKinBody("plank_{}".format(plank_num))
            camera_t = self.env.GetViewer().GetCameraTransform()

            relative_t = get_relative_transform(plank.GetTransform(),camera_t)
            np.save(Config.CAMERA_DIR+"keva_relative_camera_{}.npy".format(transform_num),relative_t)

        def start(self,complete_random = False):
            i = 0        
            flag = 0
            j = 0
            pbar = tqdm.tqdm(total=self.n)
            with self.env:
            # if True:
                if self.env.GetViewer() is not None and self.experiment_flag:
                    self.set_camera_wrt_obj("table6")

                while i < self.n:
                    transform_dict = self.randomize_env()

                    if self.random:                    
                        goalLoc_list = self.set_random_goalLoc()
                    else:
                        planks_to_set=self.no_of_planks_in_loop
                        if self.minimum_plank_count != self.no_of_planks_in_loop:
                            planks_to_set = np.random.randint(low=self.minimum_plank_count,high=self.no_of_planks_in_loop+1)
                        goalLoc_list = self.set_goalLoc(planks_to_set=planks_to_set)

                    if self.planks_in_init_state != -1:
                        planks_in_init_state = self.planks_in_init_state
                    else:
                        planks_in_init_state = np.random.randint(low=0,high=self.no_of_planks_in_loop)

                    goalLoc_in_init_state = []
                    for g in range(planks_in_init_state):
                        plank_to_set = self.env.GetKinBody("{}_{}".format(Config.OBJECT_NAME[0],str(goalLoc_list[g].GetName()).split("_")[-1]))
                        plank_to_set.SetTransform(goalLoc_list[g].GetTransform())
                        transform_dict[plank_to_set] = plank_to_set.GetTransform()
                        goalLoc_in_init_state.append(goalLoc_list[g])

                    for goalLoc in goalLoc_in_init_state:
                        goalLoc_list.remove(goalLoc)
                        
                    while j <= self.j:
                        state_list = []
                        self.robot.SetActiveDOFValues(self.init_pose)
                        self.reset_planks(transform_dict)

                        for goalLoc in goalLoc_list:
                            traj_1 = None
                            traj_2 = None
                            traj_3 = None
                            init_state = []
                            grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
                            init_state.append(self.get_one_state())

                            plank_count = int(str(goalLoc.GetName()).split("_")[-1])-1
                            #reaching plank
                            counter = 0
                            while traj_1 is None:
                                gp = self.sample_grasp_pose(self.plank_list[plank_count].GetName())
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
                            self.set_to_last_waypoint(traj_1)
                            grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
                            state1 = self.get_state_list(traj_1,grab_flag=grabbed_flag)
                            self.grab(self.plank_list[plank_count].GetName())
                            grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
                            one_state = self.get_one_state()
                            state1.append(one_state)

                            #taking plank to droparea
                            counter = 0
                            while traj_2 is None:
                                ep = self.sample_grasp_pose(goalLoc.GetName())
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
                            self.set_to_last_waypoint(traj_2)
                            self.collision_set.add(self.plank_list[plank_count])
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
                            self.set_to_last_waypoint(traj_3)
                            grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
                            state3 = self.get_state_list(traj_3,grab_flag=grabbed_flag)

                            states = [init_state,state1,state2,state3]
                            if complete_random:
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

        def random_config_robot(self,current_dof,robot=None,exp=False):
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
                if exp:
                    random_dof_values = [ 0., -2.26892803, 2.35619449, 0.52359878,  0. ,  0.6981317 , -0. ]
                else:
                    lower_limits,upper_limits = self.robot.GetActiveDOFLimits()
                    for i in range(len(current_dof)):
                        range_offset = abs(upper_limits[i]-lower_limits[i])/6.0
                        r = np.random.uniform(lower_limits[i]+range_offset,upper_limits[i]-range_offset)
                        random_dof_values.append(r)

            self.robot.SetActiveDOFValues(random_dof_values)
            t = self.gripper_link.GetTransform()
            if self.collision_check([self.robot]):
                self.random_config_robot(current_dof)

            self.robot.SetActiveDOFValues(current_dof)
            return random_dof_values, t

        def compute_delta_mp(self,direction,offset=0.1,robot=None):
            try:
                traj = self.delta_planner.PlanToEndEffectorOffset(self.robot, direction, offset,smoothingitrs=10,timelimit=10.)
                return traj
            
            except:
                return None

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
                    self.activate_manip_joints(collection_flag=True)

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
                    self.activate_manip_joints(collection_flag=True)

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

        def transform_from_pose(self,dof_vals):
            # print('last@transform_from_pose')
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

        def collision_check(self,obj_list):
            with self.env:
            # while True:
                collision = False
                for obj in obj_list:
                    if type(obj) == str:
                        if obj.split("_")[0] in Config.ROBOT_TYPES:
                            obj = self.robot_name
                        obj = self.env.GetKinBody(obj)

                    for c_obj in self.collision_set:
                        collision = self.env.CheckCollision(obj,c_obj) and obj.GetName() != c_obj.GetName() and c_obj not in self.robot.GetGrabbed()
                        if collision:
                            self.collision = collision
                            return collision

                self.collision = collision
                return collision
        
        def plank_checker(self,plank):
            t1 = plank.GetTransform()
            for pl in self.added_planks:
                if plank!=pl:
                    t2 = pl.GetTransform()
                    rel_t = get_relative_transform(t2,t1)
                    if (abs(rel_t[2,3]) > 0.15 and abs(rel_t[0,3]) > 0.05) or np.linalg.norm(rel_t[:3,3]) > 0.2:
                        continue
                    return True
            
            return False

        def object_randomizer(self,plank,x_offsets=[-20,-20],y_offsets=[-20,-20]):
            drop = self.env.GetKinBody("droparea")
            if drop is not None:
                drop_t = drop.GetTransform()
                drop_h = deepcopy(drop_t[2,3])
                drop_t[2,3] = 0.045
                self.env.GetKinBody("droparea").SetTransform(drop_t)

            while True:
                t = np.eye(4)
                t[0,3] = np.random.uniform(low = self.env_x_limits[0]+x_offsets[0], high = self.env_x_limits[1]-x_offsets[1])
                t[1,3] = np.random.uniform(low = self.env_y_limits[0]+y_offsets[0], high = self.env_y_limits[1]-y_offsets[1])

                if str(plank.GetName()).split("_")[0] in Config.OBJECT_NAME:
                    t[2,3] = 0.011

                t1 = orpy.matrixFromAxisAngle([-np.pi/2, 0, 0])
                t = t.dot(t1)
                plank.SetTransform(t)
                t2 = orpy.matrixFromAxisAngle([0,-np.pi/2, 0])
                plank.SetTransform(t.dot(t2))
                if not(self.collision_check([plank]) or self.plank_checker(plank)):
                    # self.object_randomizer(plank,x_offsets,y_offsets)
                    break

            if str(plank.GetName()).split("_")[0] in Config.OBJECT_NAME:
                self.added_planks.append(plank)

            if drop is not None:
                drop_t[2,3] = drop_h
                drop.SetTransform(drop_t)

            return plank.GetTransform()
        
        def spawn_plank(self):
            plank_name = 'plank_{}'.format(len(self.plank_list)+1)
            self.env.Load(Config.OBJECTS_DIR+"keva.dae")
            plank = self.env.GetKinBody('SketchUp')
            plank.SetName(plank_name)

            # plank = object_models.create_plank(self.env,plank_name,color=[0,0.999,0.999])
            
            self.plank_list.append(plank)
            t = np.eye(4)
            t[0,3] = np.random.uniform(low = self.env_x_limits[0]+0.1, high = self.env_x_limits[1]-0.1)
            t[1,3] = np.random.uniform(low = self.env_y_limits[0]+0.1, high = self.env_y_limits[1]-0.1)
            t[2,3] = -1.5

            plank.SetTransform(t)
            return 1
        
        def spawn_goalLoc(self):
            goalLoc_obj = object_models.create_cylinder(self.env,'goalLoc_{}'.format(len(self.goalLoc_list)+1),np.eye(4),[0,0])
            self.env.Add(goalLoc_obj)
            self.goalLoc_list.append(goalLoc_obj)
            return 1
        
        def drop_randomizer(self,x_offsets=[0.15,0.15],y_offsets=[0.15,0.15]):
            drop = self.env.GetKinBody('droparea')
            if drop in self.collision_set:
                self.collision_set.remove(drop)

            t = drop.GetTransform()
            t[0,3] = np.random.uniform(low = self.env_x_limits[0]+self.region_dim+x_offsets[0], high = self.env_x_limits[1]-self.region_dim-x_offsets[1])
            t[1,3] = np.random.uniform(low = self.env_y_limits[0]+self.region_dim+y_offsets[0], high = self.env_y_limits[1]-self.region_dim-y_offsets[1])
            t[2,3] = 0.0

            # t = np.eye(4) #FIXME
            drop.SetTransform(t)
            if self.collision_check([drop]):
                self.drop_randomizer(x_offsets,y_offsets)
            self.collision_set.add(drop)

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
            if self.plank_count > 6:
                if self.robot_name != "yumi":
                    self.setup_bigger_base()
                # x_offsets = [1.0,0.2]
                # y_offsets = [1.4,0.2]
                x_offsets = [-20,-20]
                y_offsets = [-20,-20]
            else:
                x_offsets = [-20,-20]
                y_offsets = [-20,-20]

            self.randomize_env(x_offsets,y_offsets)
            if self.random:
                self.set_random_goalLoc()
            else:
                self.set_goalLoc()

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
        
        def set_plank(self,plank,real_world_flag=False):
            if self.robot_name != "yumi":
                x_offsets = [0.2,0.6]
                y_offsets = [0.1,0.1]
            else:
                x_offsets=[-20,-20]
                y_offsets=[-20,-20]

            t = self.object_randomizer(plank,x_offsets,y_offsets)            
            if real_world_flag:
                t[:2,3] = [-0.15,0.3]
                plank.SetTransform(t)

            return self.get_current_state()

        def setup_exp(self,arg=None,req_relation=None,experiment_flag=False):
            if not experiment_flag:
                with self.env:
                # if True:
                    if self.random:
                        x_offsets = [0.2,0.28]
                        y_offsets = [0.2,0.2]
                        self.set_random_goalLoc(x_offsets=x_offsets,y_offsets=y_offsets)
                        
                        plank_transform_list = []
                        for i,p in enumerate(self.plank_list):
                            plank_transform_list.append(p.GetTransform())
                            p_num = str(p.GetName()).split("_")[1]
                            t = self.env.GetKinBody("goalLoc_{}".format(p_num)).GetTransform()
                            p.SetTransform(t)

                        goal_state = self.get_one_state()

                        for i,plank in enumerate(self.plank_list):
                            plank.SetTransform(plank_transform_list[i])
                    
                    else:
                        goal_state = self.set_test()
                    
                    init_state = self.get_one_state()

                return init_state, goal_state, None
                
        def set_test(self):
            plank_transform_list = []
            with self.env:
            # if True:
                self.drop_randomizer(x_offsets=[0.3,0.25],y_offsets=[0.25,0.4])
                self.set_goalLoc()
                
                for i,plank in enumerate(self.plank_list):
                    plank_transform_list.append(plank.GetTransform())
                    plank.SetTransform(self.env.GetKinBody(Config.LOCATION_NAME[2]+"_{}".format(i+1)).GetTransform())

                goal_state = self.get_one_state()
                
                for i,plank in enumerate(self.plank_list):
                    plank.SetTransform(plank_transform_list[i])

            return goal_state
    
    return Keva