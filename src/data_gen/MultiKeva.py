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
from src.robot_files.Robots import Fetch, MagicGripper, MagneticGripper
import math
import importlib

def GetDomain(robot_name,env_name,number_of_configs,number_of_mp,axis_for_offset,file_name="_data.p",reference_structure_name=None,visualize = False,plank_count=None, random=False,order=False,surface="",planks_in_init_state=0,quadrant=None,grasp_num=None,minimum_plank_count=None,mp=True,num_robots=1,structure_dependance=False,object_list=[],experiment_flag=False,real_world_experiment=False):
    class MultiKeva(object):
        def __init__(self,env_name,number_of_configs,number_of_mp,axis_for_offset,file_name="_data.p",reference_structure_name=None,visualize = False,plank_count=None, random=False,order=False,surface="",planks_in_init_state=0,quadrant=None,grasp_num=None,minimum_plank_count=None,mp=True,num_robots=1,structure_dependance=False,object_list=[],experiment_flag=False,real_world_experiment=False):
            self.env = orpy.Environment()
            self.env.SetDebugLevel(0)
            self.num_robots = num_robots
            self.robots = []
            self.robot_names = []
            self.structure_dependance = structure_dependance
            mod = importlib.import_module("src.robot_files.Robots")
            RobotClass = getattr(mod,robot_name)
            for i in range(1,self.num_robots+1):
                robot = RobotClass(self.env,id=i)
                setattr(self,"robot_{}".format(i),robot)
                self.robots.append(robot)

            self.robot_offset = self.robots[0].robot_offset

            # for i in range(1,self.num_robots+1):
            #     robot = getattr(self,"robot_{}".format(i))
            #     grabbed_flag = getattr(robot,"grabbed_flag_{}".format(i))
            #     setattr(self,"grabbed_flag_{}".format(i),grabbed_flag)
            #     grabbed_object = getattr(robot,"grabbed_object_{}".format(i))
            #     setattr(self,"grabbed_object_{}".format(i),grabbed_object)

            for robot in self.robots:
                self.robot_names.append(str(robot.robot.GetName()))

            self.order = order
            self.env_name = env_name

            print(self.env_name)
            self.reference_env = orpy.Environment()
            if reference_structure_name is None:
                self.reference_env.Load(Config.PLANK_PAIRS_DIR + self.env_name + ".dae")
            else:
                self.reference_env.Load(Config.REFERENCE_DIR + reference_structure_name + ".dae")
            self.n = number_of_configs
            self.j = number_of_mp
            self.data = []
            self.envs = []
            self.seeds = []
            self.object_names = set()
            self.init_config = {}
            self.env.Load(Config.ENV_DIR + env_name + ".dae")
            drop = self.env.GetKinBody("drop_area")
            drop.SetName("droparea")
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

            for robot in self.robots:
                if robot.robot_name != "yumi":
                    self.random_place_robot(robot)
                    robot.set_init_pose()

            self.store_init_config()
            self.base_offset = Config.AXIS_MAP[axis_for_offset]
            self.random = random
            self.planks_in_init_state = planks_in_init_state
            self.added_planks = []
            self.compute_mp = mp

            self.plank_h = 0.0115

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

        def store_init_config(self):
            bodies = self.env.GetBodies()
            for body in bodies:
                if body.GetName() not in self.robot_names:
                    self.init_config[body.GetName()] = body.GetTransform()

        def sample_grasp_pose(self,robot,object_name="",pose = []):
            if pose != []:
                world_T_obj = pose
            else:
                world_T_obj = self.env.GetKinBody(object_name).GetTransform()
                
            world_T_robot = self.transform_from_wp(robot.robot.GetActiveDOFValues())
            robot_T_world = np.linalg.inv(world_T_robot)

            obj_T_robot = np.eye(4)
            # obj_T_robot[1,3]= -0.07
            obj_T_robot[1,3]= robot.grasping_offset[Config.OBJECT_NAME[0]]
            
            t1 = orpy.matrixFromAxisAngle([ 0, -np.pi/2, 0])
            t2 = orpy.matrixFromAxisAngle([-np.pi/2, 0, 0])

            obj_T_robot = np.matmul(np.matmul(obj_T_robot,t1),t2)
            t = np.matmul(world_T_obj,obj_T_robot)
            pose = self.get_pose_from_transform(t)
            
            return pose

        def sample_goal_pose(self,plank_num=0):
            if self.random:
                t = self.env.GetKinBody("goalLoc"+"_{}".format(plank_num+1)).GetTransform()
                return t
            
            else:
                droparea = self.env.GetKinBody("droparea")
                t = self.reference_env.GetKinBody((self.plank_list[plank_num].GetName().split('_')[0]+self.plank_list[plank_num].GetName().split('_')[1])).GetTransform()
                goal_transform = np.matmul(droparea.GetTransform(),t)
                # goal_transform[2,3] += (self.clearance+self.base_offset)
                z_offset = np.eye(4)
                z_offset[2,3] += (self.clearance+self.base_offset)
                goal_transform = z_offset.dot(goal_transform)

                return goal_transform
        
        def set_goalLoc(self,planks_to_set=None):
            goalLoc_list = []
            if planks_to_set is None:
                planks_to_set = self.no_of_planks_in_loop
            
            for i in range(planks_to_set):
                goalLoc_list.append(self.env.GetKinBody("goalLoc"+"_{}".format(i+1)))
                            
            if self.order is not True:
                random.shuffle(goalLoc_list)
            
            droparea = self.env.GetKinBody("droparea")
            for i in range(planks_to_set):
                t = self.reference_env.GetKinBody(Config.OBJECT_NAME[0]+"{}".format(i+1)).GetTransform()
                goal_transform = np.matmul(droparea.GetTransform(),t)
                # goal_transform[2,3] += (self.clearance+self.base_offset)
                z_offset = np.eye(4)
                z_offset[2,3] += (self.clearance+self.base_offset)
                goal_transform = z_offset.dot(goal_transform)
                goalLoc = goalLoc_list[i]
                goalLoc.SetTransform(goal_transform)

            return goalLoc_list

        def set_random_goalLoc(self):
            rot = orpy.matrixFromAxisAngle([ 0, 0, -np.pi/2])
            goalLoc_list = []
            for i in range(self.plank_count):
                goalLoc = self.env.GetKinBody("goalLoc"+"_{}".format(i+1))
                t = self.object_randomizer(goalLoc)
                t = t.dot(rot)
                t[2,3] += (self.clearance+self.base_offset)
                goalLoc.SetTransform(t)
                goalLoc_list.append(goalLoc)

            np.random.shuffle(goalLoc_list)        
            return goalLoc_list

        def set_to_last_waypoint(self,robot,trajectory):
            if type(trajectory) != list:
                num = trajectory.GetNumWaypoints()
                last_wp = trajectory.GetWaypoint(num-1)
            else:
                last_wp = trajectory[0]
            robot.SetActiveDOFValues(last_wp)
                
        def random_place_robot(self,robot):
            t = robot.robot.GetTransform()
            wp = self.get_pose_from_transform(t)

            random_x = np.random.uniform(self.env_x_limits[0],self.env_x_limits[1])
            random_y = np.random.uniform(self.env_y_limits[0],self.env_y_limits[1])
            random_z = np.random.uniform(self.env_z_limits[0],self.env_z_limits[1])
            random_roll  = np.random.uniform(-3.14, 3.14)
            random_pitch = np.random.uniform(-3.14, 3.14)
            random_yaw   = np.random.uniform(-3.14, 3.14)
            
            init_pose = [random_x,random_y,random_z,random_roll,random_pitch,random_yaw]
            robot.robot.SetActiveDOFValues([random_x,random_y,random_z,random_roll,random_pitch,random_yaw])

            if self.collision_check([robot.robot]):
                self.random_place_robot(robot)
            
            return init_pose
        
        def randomize_planks(self,x_offsets=[0.1,0.1],y_offsets=[0.1,0.1]):
            for robot in self.robots:
                robot.release()
            plank_transform_dict = {}
            to_remove_list = []
            for obj in self.collision_set:
                if obj.GetName().split("_")[0] == Config.OBJECT_NAME:
                    to_remove_list.append(obj)

            for obj in to_remove_list:
                self.collision_set.remove(obj)

            for plank in self.plank_list:            
                t = self.object_randomizer(plank,x_offsets,y_offsets)
                plank_transform_dict[plank] = t
                self.collision_set.add(plank)

            return plank_transform_dict
        
        def reset_planks(self,tranform_dict):
            for robot in self.robots:
                robot.release()
                
            for p in tranform_dict.keys():
                p.SetTransform(tranform_dict[p])
        
            return True
        
        def randomize_env(self,x_offsets=[0.1,0.1],y_offsets=[0.1,0.1]):
            transform_dict = self.randomize_planks(x_offsets,y_offsets)
            robot_pose_dict = {}
            for robot in self.robots:
                rob_pose = self.random_place_robot(robot)
                robot_pose_dict["robot_{}".format(robot.id)] = rob_pose

            self.drop_randomizer(x_offsets,y_offsets)

            if not(self.collision):
                t = self.env.GetKinBody('droparea').GetTransform()
                t[2,3] = 0.0
                self.env.GetKinBody('droparea').SetTransform(t)
            
            for obj in self.bound_object_name:
                self.collision_set.add(self.env.GetKinBody(obj))   

            return transform_dict,robot_pose_dict
        
        def start(self):
            i = 0        
            flag = 0
            j = 0
            pbar = tqdm.tqdm(total=self.n)
            with self.env:
            # if True:
                while i < self.n:
                    transform_dict,robot_pose_dict = self.randomize_env()

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
                        plank_to_set = self.env.GetKinBody("{}_{}".format(Config.OBJECT_NAME,str(goalLoc_list[g].GetName()).split("_")[-1]))
                        plank_to_set.SetTransform(goalLoc_list[g].GetTransform())
                        transform_dict[plank_to_set] = plank_to_set.GetTransform()
                        goalLoc_in_init_state.append(goalLoc_list[g])

                    for goalLoc in goalLoc_in_init_state:
                        goalLoc_list.remove(goalLoc)

                    if self.random is False and self.structure_dependance:
                        while j <= self.j:
                            state_list = []

                            #ROBOT1 
                            robots_list = []
                            for rob in self.robots:
                                robots_list.append(rob)
                            robot = robots_list[0]
                            robots_list.remove(robot)

                            robot.robot.SetActiveDOFValues(robot_pose_dict["robot_{}".format(robot.id)])

                            robot2 = robots_list[0]
                            robots_list.remove(robot2)

                            robot2.robot.SetActiveDOFValues(robot_pose_dict["robot_{}".format(robot2.id)])

                            self.reset_planks(transform_dict)

                            goalLoc = goalLoc_list[0]

                            traj_1 = None
                            traj_2 = None
                            traj_3 = None
                            traj_4 = None
                            traj_5 = None
                            traj_6 = None

                            init_state = []
                            if not self.compute_mp:
                                init_state.append(self.get_one_state())

                            plank_count = int(str(goalLoc.GetName()).split("_")[-1])-1
                            
                            #reaching plank_1
                            counter = 0
                            while traj_1 is None:
                                gp = self.sample_grasp_pose(robot,self.plank_list[plank_count].GetName())
                                if self.compute_mp:
                                    traj_1 = self.compute_motion_plan(robot.robot,gp)
                                else:
                                    traj_1 = [gp]

                                counter+=1
                                if (counter == 10 and j < 1):
                                    flag = 1
                                    break
                                elif counter >= 30 and traj_1 is None:
                                    flag=2
                                    break
                                
                            if flag == 1:
                                break
                            elif flag == 2:
                                flag = 0
                                continue
                            
                            time.sleep(0.001)
                            self.set_to_last_waypoint(robot.robot,traj_1)
                            state1 = self.get_state_list(traj_1,robot)
                            robot.grab(self.plank_list[plank_count].GetName())
                            one_state = self.get_one_state()
                            state1.append(one_state)

                            #taking plank_1 to droparea
                            counter = 0
                            while traj_2 is None:
                                ep = self.sample_grasp_pose(robot,goalLoc.GetName())
                                if self.compute_mp:
                                    traj_2 = self.compute_motion_plan(robot.robot,ep)
                                else:
                                    traj_2 = [ep]

                                counter+=1
                                if (counter == 10 and j < 1) or (counter == 15 and traj_2 is None):
                                    flag = 1
                                    break
                                elif counter >= 30 and traj_2 is None:
                                    flag=2
                                    break
                                
                            if flag == 1:
                                break
                            elif flag == 2:
                                flag = 0
                                continue

                            time.sleep(0.001)
                            self.set_to_last_waypoint(robot.robot,traj_2)
                            self.collision_set.add(self.plank_list[plank_count])
                            state2 = self.get_state_list(robot=robot,traj=traj_2)

                            #ROBOT2

                            goalLoc = goalLoc_list[1]
                            
                            plank_count = int(str(goalLoc.GetName()).split("_")[-1])-1
                            #reaching plank_2
                            counter = 0
                            while traj_3 is None:
                                gp = self.sample_grasp_pose(robot2,self.plank_list[plank_count].GetName())
                                if self.compute_mp:
                                    traj_3 = self.compute_motion_plan(robot2.robot,gp)
                                else:
                                    traj_3 = [gp]

                                counter+=1
                                if (counter == 10 and j < 1):
                                    flag = 1
                                    break
                                elif counter >= 30 and traj_3 is None:
                                    flag=2
                                    break
                                
                            if flag == 1:
                                break
                            elif flag == 2:
                                flag = 0
                                continue
                            
                            time.sleep(0.001)
                            self.set_to_last_waypoint(robot2.robot,traj_3)
                            state3 = self.get_state_list(traj_3,robot2)
                            robot2.grab(self.plank_list[plank_count].GetName())
                            one_state = self.get_one_state()
                            state3.append(one_state)

                            #taking plank_2 to droparea
                            counter = 0
                            while traj_4 is None:
                                ep = self.sample_grasp_pose(robot2,goalLoc.GetName())
                                if self.compute_mp:
                                    traj_4 = self.compute_motion_plan(robot2.robot,ep)
                                else:
                                    traj_4 = [ep]
                                counter+=1
                                if (counter == 10 and j < 1) or (counter == 15 and traj_4 is None):
                                    flag = 1
                                    break
                                elif counter >= 30 and traj_4 is None:
                                    flag=2
                                    break
                                
                            if flag == 1:
                                break
                            elif flag == 2:
                                flag = 0
                                continue

                            time.sleep(0.001)
                            self.set_to_last_waypoint(robot2.robot,traj_4)
                            self.collision_set.add(self.plank_list[plank_count])
                            state4 = self.get_state_list(robot=robot2,traj=traj_4)
                            
                            robots_list = []
                            for rob in self.robots:
                                robots_list.append(rob)
                            
                            if len(robots_list) == 0:
                                r = 0
                            else:
                                r = np.random.randint(low=0,high=len(robots_list))
                            robot = robots_list[r]
                            robots_list.remove(robot)

                            if len(robots_list) == 0:
                                r = 0
                            else:
                                r = np.random.randint(low=0,high=len(robots_list))
                            robot2 = robots_list[r]
                            robots_list.remove(robot2)

                            robot.release()
                            one_state = self.get_one_state()
                            state4.append(one_state)

                            #random_placing_robot
                            while traj_5 is None:
                                ep,_ = self.random_config_robot(robot=robot,current_dof=robot.robot.GetActiveDOFValues())
                                if self.compute_mp:
                                    traj_5 = self.compute_motion_plan(robot.robot,ep)
                                else:
                                    traj_5 = [ep]
                                counter+=1
                                if (counter == 10 and j < 1) or (counter == 15 and traj_5 is None):
                                    flag = 1
                                    break
                                elif counter >= 30 and traj_5 is None:
                                    flag=2
                                    break
                                
                            if flag == 1:
                                break
                            elif flag == 2:
                                flag = 0
                                continue

                            time.sleep(0.001)
                            self.set_to_last_waypoint(robot.robot,traj_5)
                            state5 = self.get_state_list(traj=traj_5,robot=robot)
                            robot2.release()
                            one_state = self.get_one_state()
                            state5.append(one_state)                    

                            #random_placing_robot
                            while traj_6 is None:
                                ep,_ = self.random_config_robot(robot=robot2,current_dof=robot2.robot.GetActiveDOFValues())
                                if self.compute_mp:
                                    traj_6 = self.compute_motion_plan(robot2.robot,ep)
                                else:
                                    traj_6 = [ep]
                                counter+=1
                                if (counter == 10 and j < 1) or (counter == 15 and traj_6 is None):
                                    flag = 1
                                    break
                                elif counter >= 30 and traj_6 is None:
                                    flag=2
                                    break
                                
                            if flag == 1:
                                break
                            elif flag == 2:
                                flag = 0
                                continue

                            time.sleep(0.001)
                            self.set_to_last_waypoint(robot2.robot,traj_6)
                            state6 = self.get_state_list(traj=traj_6,robot=robot2)

                            state_list.extend(init_state)
                            state_list.extend(state1)
                            state_list.extend(state2)
                            state_list.extend(state3)
                            state_list.extend(state4)
                            state_list.extend(state5)
                            state_list.extend(state6)

                            if flag == 1:
                                break
                            elif flag == 2:
                                flag = 0
                                continue
                            
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

                    else:
                        robot = self.robots[0]
                        while j <= self.j:
                            state_list = []
                            for rob in self.robots:
                                rob.robot.SetActiveDOFValues(robot_pose_dict["robot_{}".format(rob.id)])
                                
                            self.reset_planks(transform_dict)

                            for goalLoc in goalLoc_list:                              
                                traj_1 = None
                                traj_2 = None
                                traj_3 = None
                                
                                init_state = []
                                if not self.compute_mp:
                                    init_state.append(self.get_one_state())

                                plank_count = int(str(goalLoc.GetName()).split("_")[-1])-1
                                #reaching plank
                                counter = 0
                                while traj_1 is None:
                                    gp = self.sample_grasp_pose(robot,self.plank_list[plank_count].GetName())
                                    if self.compute_mp:
                                        traj_1 = self.compute_motion_plan(robot.robot,gp)
                                    else:
                                        traj_1 = [gp]

                                    counter+=1
                                    if (counter == 10 and j < 1):
                                        flag = 1
                                        break
                                    elif counter >= 30 and traj_1 is None:
                                        flag=2
                                        break
                                    
                                if flag == 1:
                                    break
                                elif flag == 2:
                                    flag = 0
                                    continue
                                
                                time.sleep(0.001)
                                self.set_to_last_waypoint(robot.robot,traj_1)
                                state1 = self.get_state_list(traj_1,robot)
                                robot.grab(self.plank_list[plank_count].GetName())
                                one_state = self.get_one_state()
                                state1.append(one_state)

                                #taking plank to droparea
                                counter = 0
                                while traj_2 is None:
                                    ep = self.sample_grasp_pose(robot,goalLoc.GetName())
                                    if self.compute_mp:
                                        traj_2 = self.compute_motion_plan(robot.robot,ep)
                                    else:
                                        traj_2 = [ep]
                                    counter+=1
                                    if (counter == 10 and j < 1) or (counter == 15 and traj_2 is None):
                                        flag = 1
                                        break
                                    elif counter >= 30 and traj_2 is None:
                                        flag=2
                                        break
                                    
                                if flag == 1:
                                    break
                                elif flag == 2:
                                    flag = 0
                                    continue

                                time.sleep(0.001)
                                self.set_to_last_waypoint(robot.robot,traj_2)
                                self.collision_set.add(self.plank_list[plank_count])
                                state2 = self.get_state_list(robot=robot,traj=traj_2)
                                robot.release()
                                one_state = self.get_one_state()
                                state2.append(one_state)

                                #random_placing_robot
                                while traj_3 is None:
                                    ep,_ = self.random_config_robot(robot=robot,current_dof=robot.robot.GetActiveDOFValues())
                                    if self.compute_mp:
                                        traj_3 = self.compute_motion_plan(robot.robot,ep)
                                    else:
                                        traj_3 = [ep]
                                    counter+=1
                                    if (counter == 10 and j < 1) or (counter == 15 and traj_3 is None):
                                        flag = 1
                                        break
                                    elif counter >= 30 and traj_3 is None:
                                        flag=2
                                        break
                                    
                                if flag == 1:
                                    break
                                elif flag == 2:
                                    flag = 0
                                    continue

                                time.sleep(0.001)
                                self.set_to_last_waypoint(robot.robot,traj_3)
                                state3 = self.get_state_list(traj=traj_3,robot=robot)

                                state_list.extend(init_state)
                                state_list.extend(state1)
                                state_list.extend(state2)
                                state_list.extend(state3)

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
                cPickle.dump(final_data,open(Config.DATA_MISC_DIR+ self.env_name+"/"+ self.file_name ,"wb"),protocol=cPickle.HIGHEST_PROTOCOL)
                print("{} data saved".format(self.env_name))

        def get_object_name_list(self):
            for obj in self.env.GetBodies():
                if obj not in self.robots:
                    self.object_names.add(str(obj.GetName()))
                else:
                    for rob in self.robots:
                        for key in rob.robot_type_object_mappings:
                            self.object_names.add(rob.robot_type_object_mappings[key])

        def random_config_robot(self,robot,current_dof):
            random_dof_values = []
            random_x = np.random.uniform(self.env_x_limits[0],self.env_x_limits[1])
            random_y = np.random.uniform(self.env_y_limits[0],self.env_y_limits[1])
            random_z = np.random.uniform(self.env_z_limits[0]+0.3,self.env_z_limits[1])
            random_roll  = np.random.uniform(-3.14, 3.14)
            random_pitch = np.random.uniform(-3.14, 3.14)
            random_yaw   = np.random.uniform(-3.14, 3.14)

            random_dof_values = [random_x,random_y,random_z,random_roll,random_pitch,random_yaw]

            robot.robot.SetActiveDOFValues(random_dof_values)
            t = robot.GetTransform()
            if self.collision_check([robot.robot]):
                self.random_config_robot(robot,current_dof)

            robot.robot.SetActiveDOFValues(current_dof)   
            return random_dof_values, t

        def compute_motion_plan(self,robot,goal):
            try:
                traj = self.motion_planner.PlanToConfiguration(robot,goal)
                return traj

            except PlanningError as e:
                # print(e)
                # print("could not find motion plan")
                return None

        def get_state_list(self,traj,robot):
            state_list = []
            if type(traj) != list:
                len_traj = traj.GetNumWaypoints()
                wp = traj.GetWaypoint(0)
                if len(wp) == 3:
                    robot.activate_base_joints()
                else:
                    robot.activate_manip_joints()

                for i in range(len_traj):
                    wp = traj.GetWaypoint(i)
                    obj_dic = {}
                    robot.robot.SetActiveDOFValues(wp)
                    for obj in self.env.GetBodies():
                        if str(obj.GetName()) not in self.bound_object_name:
                            if str(obj.GetName()).split("_")[0] not in Config.ROBOT_TYPES:
                                name = str(obj.GetName())
                                obj_dic[name] = self.get_pose_from_transform(obj.GetTransform())
                            else:
                                for rob in self.robots:
                                    if rob.robot_name == str(obj.GetName()):
                                        for link in rob.robot_type_object_mappings.keys():
                                           name_type = rob.robot_type_object_mappings[link].split("_")[0]
                                           joints_activation_function = getattr(rob,"activate_{}".format(Config.ROBOT_TYPES[name_type]))
                                           joints_activation_function()
                                           name = rob.robot_type_object_mappings[link]
                                           obj_dic[name] = [rob.robot.GetActiveDOFValues(),self.get_pose_from_transform(rob.robot.GetLink(link).GetTransform())]

                    kwargs = {}

                    for r in self.robots:
                        grabbed_flag_key = "grabbed_flag_{}".format(r.id)
                        grabbed_object_key = "grabbed_object_{}".format(r.id)
                        grabbed_flag = getattr(r,grabbed_flag_key)
                        grabbed_object = getattr(r,grabbed_object_key)
                        kwargs[grabbed_flag_key] = grabbed_flag
                        kwargs[grabbed_object_key] = grabbed_object
                    
                    state = EnvState(obj_dic=obj_dic,keyword_arguments=kwargs,num_robots=self.num_robots)
                    state_list.append(state)
            
            else:
                wp = traj[0]
                if len(wp) == 3:
                    robot.activate_base_joints()
                else:
                    robot.activate_manip_joints()
                
                robot.robot.SetActiveDOFValues(wp)
                state_list.append(self.get_one_state())

            return state_list
        
        def get_one_state(self):
            obj_dic = {}
            for obj in self.env.GetBodies():
                if str(obj.GetName()) not in self.bound_object_name:
                    if str(obj.GetName()).split("_")[0] not in Config.ROBOT_TYPES:
                        name = str(obj.GetName())
                        obj_dic[name] = self.get_pose_from_transform(obj.GetTransform())
                    else:
                        for rob in self.robots:
                            if rob.robot_name == str(obj.GetName()):
                                for link in rob.robot_type_object_mappings.keys():
                                    name_type = rob.robot_type_object_mappings[link].split("_")[0]
                                    joints_activation_function = getattr(rob,"activate_{}".format(Config.ROBOT_TYPES[name_type]))
                                    joints_activation_function()
                                    name = rob.robot_type_object_mappings[link]
                                    obj_dic[name] = [rob.robot.GetActiveDOFValues(),self.get_pose_from_transform(rob.robot.GetLink(link).GetTransform())]

            kwargs = {}
            for r in self.robots:
                grabbed_flag_key = "grabbed_flag_{}".format(r.id)
                grabbed_object_key = "grabbed_object_{}".format(r.id)
                grabbed_flag = getattr(r,grabbed_flag_key)
                grabbed_object = getattr(r,grabbed_object_key)
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

        def collision_check(self,obj_list):
            with self.env:
            # while True:
                collision = False
                for obj in obj_list:
                    if type(obj) == str:
                        obj = self.env.GetKinBody(obj)
                        
                    for c_obj in self.collision_set:
                        grabbed_objects = []
                        for i in range(1,self.num_robots+1):
                            for rob in self.robots:
                                if rob.id == i:
                                    break
                            grabbed_objects.append(getattr(rob,"grabbed_object_{}".format(i)))
                        collision = self.env.CheckCollision(obj,c_obj) and obj.GetName() != c_obj.GetName() and str(c_obj.GetName()) not in grabbed_objects
                        if collision:
                            return collision

                self.collision = collision
                return collision
        
        def plank_checker(self,plank):
            t_y = plank.GetTransform()[1,3]
            t_x = plank.GetTransform()[0,3]
            for pl in self.added_planks:
                if plank!=pl:
                    p_y = pl.GetTransform()[1,3]
                    p_x = pl.GetTransform()[0,3]
                    if abs(p_y-t_y) < self.robot_offset:
                        if abs(p_x-t_x) > self.get_object_dims(str(plank.GetName()))[0]:
                            continue
                        return True
            
            return False

        def object_randomizer(self,plank,x_offsets=[0.1,0.1],y_offsets=[0.1,0.1]):
            drop_t = self.env.GetKinBody("droparea").GetTransform()
            drop_h = deepcopy(drop_t[2,3])
            drop_t[2,3] = drop_h + 0.015
            self.env.GetKinBody("droparea").SetTransform(drop_t)
            plank_h = 0.0
            if "plank" in str(plank.GetName()):
                plank_h = self.plank_h

            while True:
                t = np.eye(4)
                t[0,3] = np.random.uniform(low = self.env_x_limits[0]+x_offsets[0], high = self.env_x_limits[1]-x_offsets[1])
                t[1,3] = np.random.uniform(low = self.env_y_limits[0]+y_offsets[0], high = self.env_y_limits[1]-y_offsets[1])

                if str(plank.GetName()).split("_")[0] in Config.OBJECT_NAME:
                    t[2,3] = drop_h+plank_h+self.clearance

                t1 = orpy.matrixFromAxisAngle([-np.pi/2, 0, 0])
                t = t.dot(t1)
                plank.SetTransform(t)

                if not (self.collision_check([plank]) or self.plank_checker(plank)):
                    break

            if str(plank.GetName()).split("_")[0] in Config.OBJECT_NAME:
                self.added_planks.append(plank)

            drop_t[2,3] = drop_h
            self.env.GetKinBody("droparea").SetTransform(drop_t)

            return plank.GetTransform()
        
        def spawn_plank(self):
            self.env.Load(Config.OBJECTS_DIR+"keva.dae")
            plank = self.env.GetKinBody('SketchUp')
            plank.SetName('plank_{}'.format(len(self.plank_list)+1))
            self.plank_list.append(plank)

            t = np.eye(4)
            t[0,3] = np.random.uniform(low = self.env_x_limits[0]+0.1, high = self.env_x_limits[1]-0.1)
            t[1,3] = np.random.uniform(low = self.env_y_limits[0]+0.1, high = self.env_y_limits[1]-0.1)
            t[2,3] = -0.5

            plank.SetTransform(t)
            return 1
        
        def spawn_goalLoc(self):
            goalLoc_obj = object_models.create_cylinder(self.env,'goalLoc_{}'.format(len(self.goalLoc_list)+1),np.eye(4),[0,0])
            self.env.Add(goalLoc_obj)
            self.goalLoc_list.append(goalLoc_obj)
            return 1
        
        def drop_randomizer(self,x_offsets=[0.1,0.1],y_offsets=[0.1,0.1]):
            drop = self.env.GetKinBody('droparea')
            if drop in self.collision_set:
                self.collision_set.remove(drop)

            t = drop.GetTransform()
            t[0,3] = np.random.uniform(low = self.env_x_limits[0]+self.region_dim+x_offsets[0], high = self.env_x_limits[1]-self.region_dim-x_offsets[1])
            t[1,3] = np.random.uniform(low = self.env_y_limits[0]+self.region_dim+y_offsets[0], high = self.env_y_limits[1]-self.region_dim-y_offsets[1])
            t[2,3] = 0.0

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
                self.setup_bigger_base()
                x_offsets = [1.0,0.2]
                y_offsets = [1.4,0.2]
            else:
                x_offsets = [0.1,0.1]
                y_offsets = [0.1,0.1]

            self.randomize_env(x_offsets,y_offsets)
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
        
        def setup_exp(self,arg=None,rcr_dict=None):
            with self.env:
            # if True:
                if self.random:
                    goal_state = self.set_random_goalLoc()
                
                else:
                    goal_state = self.set_test()
                
                planks_to_place = len(self.plank_list) - self.planks_in_init_state
                for plank in self.plank_list[::-1][:planks_to_place]:
                    if "yumi" not in self.robot_names:
                        x_offsets = [0.2,0.6]
                        y_offsets = [0.1,0.1]
                    else:
                        x_offsets=[0.2,0.35]
                        y_offsets=[0.6,0.02]

                    self.object_randomizer(plank,x_offsets,y_offsets)
                
                droparea = self.env.GetKinBody("droparea")
                self.env.Remove(droparea)

                init_state = self.get_one_state()

            return init_state, goal_state, None
            
        def set_test(self):
            with self.env:
            # if True:
                self.drop_randomizer(x_offsets=[0.275,0.4],y_offsets=[0.35,0.35])
                self.set_goalLoc()
                
                for i,plank in enumerate(self.plank_list):
                    plank.SetTransform(self.env.GetKinBody("goalLoc"+"_{}".format(i+1)).GetTransform())

                goal_state = self.get_one_state()
                        
            return goal_state
    
    return MultiKeva(env_name,number_of_configs,number_of_mp,axis_for_offset,file_name,reference_structure_name,visualize,plank_count,random,order,surface,planks_in_init_state,quadrant,grasp_num,minimum_plank_count,mp,num_robots,structure_dependance,object_list,experiment_flag)