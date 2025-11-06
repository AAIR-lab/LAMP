import numpy as np
import pickle
import tqdm
import os
import time
from Config import Config
import useful_functions
from copy import deepcopy

SimClass = Config.get_simulator_module("SimClass").SimClass
Object = Config.get_simulator_module("Object").Object
model_gen_utils = Config.get_simulator_module("model_gen_utils")

class Keva(object):
    def __init__(self,env_name,number_of_configs=1,number_of_mp=1,axis_for_offset="x",file_name="_data",reference_structure_name=None,visualize=False,object_count=None,random=False,order=False,surface="",objects_in_init_state=0,quadrant=None,grasp_num=None,minimum_object_count=None,mp=True,num_robots=1,structure_dependance=False,object_list=[],experiment_flag=False,real_world_experiment=False,set_y=False,complete_random=False,data_gen=True,robot_name=Config.ROBOT_NAME):
        self.sim_object = SimClass(robot_name=robot_name,
                                    visualize=visualize)

        self.sim_object.robot.activate_manip_joints(collection_flag=data_gen)
        self.data_gen_flag = data_gen
        self.order = order
        self.env_name = env_name
        print(self.env_name)
        self.reference_sim = SimClass()
        self.robot_name = robot_name
        default_reference_flag = False
        successful_load = False
        if reference_structure_name is None:
            successful_load = self.reference_sim.load_env("{}.dae".format(env_name))
        else:
            if "env" in reference_structure_name:
                successful_load = self.reference_sim.load_env(reference_structure_name + ".dae")
            else:    
                successful_load = self.reference_sim.load_env(reference_structure_name + ".dae",reference = True)
        if not successful_load:
            if "problem" in env_name:
                self.reference_sim.load_env("{}.dae".format(env_name),reference=True)
            else:
                self.reference_sim.load_env("env.dae")
        self.n = number_of_configs
        self.j = number_of_mp
        self.data = []
        self.envs = []
        self.seeds = []
        self.object_names = set()
        self.init_config = {}
        self.sim_object.load_env("env.dae")
        drop = self.sim_object.get_obj("drop_area")
        drop_t = drop.get_transform()
        self.sim_object.remove_obj(drop)
        drop = Object(model_gen_utils.create_flat_area(self.sim_object.env,"droparea",[0.99,0.99,0.99],t=drop_t))
        self.sim_object.add_obj(drop)
        spawn = self.sim_object.get_obj("spawn_area")
        self.sim_object.remove_obj(spawn)
        self.env_obstacle_num = 10
        self.bound_object_name = ['table6']
        self.sim_object.collision_set = set([drop,spawn])
        for obj_name in self.bound_object_name:
            self.sim_object.collision_set.add(self.sim_object.get_obj(obj_name))
        self.clearance = 0.0
        self.obj_h = 0.05
        self.env_limits = self.sim_object.get_env_limits(self.bound_object_name)
        self.env_y_limits = self.env_limits[1] #[y_min,y_max]
        self.env_x_limits = self.env_limits[0] #[x_min,x_max]
        self.env_z_limits = [self.env_limits[-1],1.0]
        self.region_dim = 0.075
        self.collision = False
        self.plank_list = []
        self.goalLoc_list = []
        self.file_name = file_name
        if robot_name != "yumi": ###
            random_place, _ = self.sim_object.robot.random_config_robot(collision_fn=self.sim_object.collision_check)
            self.sim_object.robot.set_active_dof_values(random_place)
            self.sim_object.robot.get_init_pose()
        self.axis_for_offset = axis_for_offset
        self.base_offset = Config.AXIS_MAP[axis_for_offset]
        if type(random) != bool:
            self.random = random>=0
        elif random is not None:
            self.random = random
        
        self.complete_random = complete_random

        self.objects_in_init_state = objects_in_init_state
        self.added_planks = []
        self.compute_mp = mp
        self.num_robots = num_robots
        self.robot_1 = self
        self.robots = [self.robot_1]
        self.set_y = set_y

        if object_count is None:
            self.object_count = len(self.reference_sim.get_objects())
            self.no_of_planks_in_loop = self.object_count

        elif object_count > len(self.reference_sim.get_objects()) > 0:
            self.no_of_planks_in_loop = len(self.reference_sim.get_objects())
            self.object_count=object_count

        else:
            self.object_count=object_count
            self.no_of_planks_in_loop = self.object_count

        self.minimum_object_count = minimum_object_count
        if self.minimum_object_count is None:
            self.minimum_object_count = self.no_of_planks_in_loop

        # for i in range(max(2,self.object_count)):
        for i in range(self.object_count):
            self.spawn_goalLoc()
            self.spawn_plank()
        
        if random > 0:
            self.num_planks_random = random
        elif random == 0 and type(random) != bool:
            self.num_planks_random = self.object_count
        else:
            self.num_planks_random = 0

        if "problem" in env_name:
            self.set_test()

        self.experiment_flag = experiment_flag

    def sample_grasp_pose(self,object_name="",pose = []):
        if pose != []:
            world_T_obj = pose
        else:
            world_T_obj = self.sim_object.get_obj(object_name).get_transform()

        obj_T_robot = np.eye(4)
        obj_T_robot[1,3]= self.sim_object.robot.grasping_offset[Config.OBJECT_NAME[0]]
        
        t1 = useful_functions.rotmat_from_rotvec([ 0, -np.pi/2, 0])
        t2 = useful_functions.rotmat_from_rotvec([-np.pi/2, 0, 0])

        obj_T_robot = np.matmul(np.matmul(obj_T_robot,t1),t2)
        t = np.matmul(world_T_obj,obj_T_robot)
        pose = useful_functions.sixd_pose_from_transform(t)
        
        return pose

    def set_goalLoc(self,planks_to_set=None):
        goalLoc_list = []
        if planks_to_set is None:
            planks_to_set = self.no_of_planks_in_loop
        
        for i in range(planks_to_set-self.num_planks_random):
            goalLoc_list.append(self.sim_object.get_obj(Config.LOCATION_NAME[-1]+"_{}Target_{}".format(Config.OBJECT_NAME[0].split("_")[Config.OBJ_TYPE_IND],i+1)))
                        
        droparea = self.sim_object.get_obj("droparea")
        for i in range(planks_to_set-self.num_planks_random):
            obj = self.reference_sim.get_obj(Config.OBJECT_NAME[0]+"{}".format(i+1))
            if obj is None:
                obj = self.reference_sim.get_obj(Config.LOCATION_NAME[-1]+"_{}Target_{}".format(Config.OBJECT_NAME[0].split("_")[Config.OBJ_TYPE_IND],i+1))
            t = obj.get_transform()
            y_offset = np.eye(4)
            y_offset[1,3] = 0.0
            goal_transform = np.matmul(droparea.get_transform().dot(y_offset),t)
            z_offset = np.eye(4)
            z_offset[2,3] += (self.clearance+self.base_offset)
            goal_transform = z_offset.dot(goal_transform)
            goalLoc = goalLoc_list[i]
            if self.set_y:
                goal_transform = self.set_goalLoc_y(goal_transform)

            goalLoc.set_transform(goal_transform)
            self.added_planks.append(goalLoc)
        
        if self.num_planks_random > 0:
            for j in range(self.num_planks_random):
                i = len(goalLoc_list) + j
                goalLoc = self.sim_object.get_obj(Config.LOCATION_NAME[-1]+"_{}Target_{}".format(Config.OBJECT_NAME[0].split("_")[Config.OBJ_TYPE_IND],i+1))
                rot = useful_functions.rotmat_from_rotvec([ 0, 0, -np.pi/2])
                t = self.object_randomizer(goalLoc)
                t = t.dot(rot)
                t[2,3] += (self.clearance+self.base_offset)
                if self.set_y:
                    t = self.set_goalLoc_y(t)
                goalLoc.set_transform(t)
                goalLoc_list.append(goalLoc)
                self.added_planks.append(goalLoc)
        
        for obj in goalLoc_list:
            self.added_planks.remove(obj)

        if self.order is not True:
            np.random.shuffle(goalLoc_list)
        
        return goalLoc_list

    def set_random_goalLoc(self,x_offsets=[0.02,0.02],y_offsets=[0.02,0.02]):
        rot = useful_functions.rotmat_from_rotvec([ 0, 0, -np.pi/2])
        goalLoc_list = []
        for i in range(self.object_count):
            goalLoc = self.sim_object.get_obj(Config.LOCATION_NAME[-1]+"_{}Target_{}".format(Config.OBJECT_NAME[0].split("_")[Config.OBJ_TYPE_IND],i+1))
            self.added_planks.append(goalLoc)
            t = self.object_randomizer(goalLoc,x_offsets=x_offsets,y_offsets=y_offsets)
            t = t.dot(rot)
            t[2,3] += (self.clearance+self.base_offset)
            if self.set_y:
                t = self.set_goalLoc_y(t)
            goalLoc.set_transform(t)
            goalLoc_list.append(goalLoc)

        for obj in goalLoc_list:
            self.added_planks.remove(obj)

        np.random.shuffle(goalLoc_list)        
        return goalLoc_list

    def set_goalLoc_y(self,current_t):
        t = useful_functions.rotmat_from_rotvec([0,0,np.pi/2])
        final_t = current_t.dot(t)
        final_t[2,3] = (self.clearance+Config.AXIS_MAP["y"]) + 0.001

        # goalLoc.SetTransform(final_t)
        return final_t
            
    def randomize_planks(self,x_offsets=[-20,-20],y_offsets=[-20,-20]):
        self.sim_object.robot.release()
        plank_transform_dict = {}
        to_remove_list = []
        for obj in self.sim_object.collision_set:
            if obj.get_name().split("_")[Config.OBJ_TYPE_IND] in Config.OBJECT_NAME:
                to_remove_list.append(obj)

        for obj in to_remove_list:
            self.sim_object.collision_set.remove(obj)

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
            plank.set_transform(t)
            plank_transform_dict[plank] = t
            self.sim_object.collision_set.add(plank)

        return plank_transform_dict
    
    def reset_planks(self,tranform_dict):
        self.sim_object.robot.release()
        for p in tranform_dict.keys():
            p.set_transform(tranform_dict[p])
    
        return True
    
    def randomize_env(self,x_offsets=[-20,-20],y_offsets=[-20,-20],traj_config=None):
        self.added_planks = []
        transform_dict = self.randomize_planks(x_offsets,y_offsets)
        #self.sim_object.robot.tuck_arm()
        random_place, _ = self.sim_object.robot.random_config_robot(collision_fn=self.sim_object.collision_check)
        self.sim_object.robot.set_active_dof_values(random_place)
        if self.sim_object.get_obj("droparea") is not None:
            self.drop_randomizer()

        if not(self.collision):
            if self.sim_object.get_obj("droparea") is not None:
                t = self.sim_object.get_obj('droparea').get_transform()
                t[2,3] = 0.0
                self.sim_object.get_obj('droparea').set_transform(t)
        
        for obj in self.bound_object_name:
            self.sim_object.collision_set.add(self.sim_object.get_obj(obj))   

        return transform_dict

    @SimClass.conditional_env_lock
    def start(self,complete_random = False):
        i = 0        
        flag = 0
        j = 0
        pbar = tqdm.tqdm(total=self.n)

        if self.sim_object.env.GetViewer() is not None and self.experiment_flag:
            self.sim_object.set_camera_wrt_obj("table6")

        while i < self.n:
            transform_dict = self.randomize_env()

            if self.random:                    
                goalLoc_list = self.set_random_goalLoc()
            else:
                planks_to_set=self.no_of_planks_in_loop
                if self.minimum_object_count != self.no_of_planks_in_loop:
                    planks_to_set = np.random.randint(low=self.minimum_object_count,high=self.no_of_planks_in_loop+1)
                goalLoc_list = self.set_goalLoc(planks_to_set=planks_to_set)

            if self.objects_in_init_state != -1:
                objects_in_init_state = self.objects_in_init_state
            else:
                objects_in_init_state = np.random.randint(low=0,high=self.no_of_planks_in_loop)

            goalLoc_in_init_state = []
            for g in range(objects_in_init_state):
                plank_to_set = self.sim_object.get_obj("{}_{}".format(Config.OBJECT_NAME[0],str(goalLoc_list[g].get_name()).split("_")[-1]))
                plank_to_set.set_transform(goalLoc_list[g].get_transform())
                transform_dict[plank_to_set] = plank_to_set.get_transform()
                goalLoc_in_init_state.append(goalLoc_list[g])

            for goalLoc in goalLoc_in_init_state:
                goalLoc_list.remove(goalLoc)
                
            while j <= self.j:
                state_list = []
                self.sim_object.robot.set_active_dof_values(self.sim_object.robot.init_pose)
                self.reset_planks(transform_dict)

                for goalLoc in goalLoc_list:
                    traj_1 = None
                    traj_2 = None
                    traj_3 = None
                    init_state = []
                    grabbed_flag = getattr(self.sim_object.robot,"grabbed_flag_{}".format(self.sim_object.robot.id))
                    init_state.append(self.sim_object.get_one_state())

                    object_count = int(str(goalLoc.get_name()).split("_")[-1])-1
                    #reaching plank
                    counter = 0
                    while traj_1 is None:
                        gp = self.sample_grasp_pose(self.plank_list[object_count].get_name())
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
                    self.sim_object.set_to_last_waypoint(traj_1)
                    grabbed_flag = getattr(self.sim_object.robot,"grabbed_flag_{}".format(self.sim_object.robot.id))
                    state1 = self.sim_object.get_state_list(traj_1)
                    self.sim_object.robot.grab(self.plank_list[object_count].get_name())
                    grabbed_flag = getattr(self.sim_object.robot,"grabbed_flag_{}".format(self.sim_object.robot.id))
                    one_state = self.sim_object.get_one_state()
                    state1.append(one_state)

                    #taking plank to droparea
                    counter = 0
                    while traj_2 is None:
                        ep = self.sample_grasp_pose(goalLoc.get_name())
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
                    self.sim_object.set_to_last_waypoint(traj_2)
                    self.sim_object.collision_set.add(self.plank_list[object_count])
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
                    self.sim_object.set_to_last_waypoint(traj_3)
                    grabbed_flag = getattr(self.sim_object.robot,"grabbed_flag_{}".format(self.sim_object.robot.id))
                    state3 = self.sim_object.get_state_list(traj_3)

                    states = [init_state,state1,state2,state3]
                    if complete_random:
                        last_ind = np.random.randint(low=1,high=len(states))
                    else:
                        last_ind = len(states)

                    for state_num in range(last_ind):
                        state_list.extend(states[state_num])

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

    def plank_checker(self,plank):
        t1 = plank.get_transform()
        for pl in self.added_planks:
            if plank!=pl:
                t2 = pl.get_transform()
                rel_t = useful_functions.get_relative_transform(t2,t1)
                if (abs(rel_t[2,3]) > 0.15 and abs(rel_t[0,3]) > 0.05) or np.linalg.norm(rel_t[:3,3]) > 0.2:
                    continue
                return True
        
        return False

    def object_randomizer(self,plank,x_offsets=[-20,-20],y_offsets=[-20,-20]):
        drop = self.sim_object.get_obj("droparea")
        if drop is not None:
            drop_t = drop.get_transform()
            drop_h = deepcopy(drop_t[2,3])
            drop_t[2,3] = 0.045
            self.sim_object.get_obj("droparea").set_transform(drop_t)

        while True:
            t = np.eye(4)
            t[0,3] = np.random.uniform(low = self.env_x_limits[0]+x_offsets[0], high = self.env_x_limits[1]-x_offsets[1])
            t[1,3] = np.random.uniform(low = self.env_y_limits[0]+y_offsets[0], high = self.env_y_limits[1]-y_offsets[1])

            if str(plank.get_name()).split("_")[Config.OBJ_TYPE_IND] in Config.OBJECT_NAME:
                t[2,3] = 0.011

            t1 = useful_functions.rotmat_from_rotvec([-np.pi/2, 0, 0])
            t = t.dot(t1)
            plank.set_transform(t)
            t2 = useful_functions.rotmat_from_rotvec([0,-np.pi/2, 0])
            plank.set_transform(t.dot(t2))
            if not(self.sim_object.collision_check([plank]) or self.plank_checker(plank)):
                # self.object_randomizer(plank,x_offsets,y_offsets)
                break

        if str(plank.get_name()).split("_")[Config.OBJ_TYPE_IND] in Config.OBJECT_NAME:
            self.added_planks.append(plank)

        if drop is not None:
            drop_t[2,3] = drop_h
            drop.set_transform(drop_t)

        return plank.get_transform()
    
    def spawn_plank(self):
        plank_name = 'plank_{}'.format(len(self.plank_list)+1)
        plank = self.sim_object.load_obj("keva.dae")
        plank.set_name(plank_name)
        self.sim_object.add_obj(plank)

        # plank = model_gen_utils.create_plank(self.env,plank_name,color=[0,0.999,0.999])
        
        self.plank_list.append(plank)

        t = np.eye(4)
        t[0,3] = np.random.uniform(low = self.env_x_limits[0]+0.1, high = self.env_x_limits[1]-0.1)
        t[1,3] = np.random.uniform(low = self.env_y_limits[0]+0.1, high = self.env_y_limits[1]-0.1)
        t[2,3] = -1.5

        plank.set_transform(t)
        return 1
    
    def spawn_goalLoc(self):
        goalLoc_obj = Object(model_gen_utils.create_cylinder(self.sim_object.env,'loc_{}Target_{}'.format(Config.OBJECT_NAME[0].split("_")[Config.OBJ_TYPE_IND],len(self.goalLoc_list)+1),np.eye(4),[0,0]))
        self.sim_object.add_obj(goalLoc_obj)
        self.goalLoc_list.append(goalLoc_obj)
        return 1
    
    def drop_randomizer(self,x_offsets=[0.15,0.15],y_offsets=[0.15,0.15]):
        drop = self.sim_object.get_obj('droparea')
        if drop in self.sim_object.collision_set:
            self.sim_object.collision_set.remove(drop)

        t = np.eye(4)
        while True:
            t[0,3] = np.random.uniform(low = self.env_x_limits[0]+self.region_dim+x_offsets[0], high = self.env_x_limits[1]-self.region_dim-x_offsets[1])
            t[1,3] = np.random.uniform(low = self.env_y_limits[0]+self.region_dim+y_offsets[0], high = self.env_y_limits[1]-self.region_dim-y_offsets[1])
            t[2,3] = 0.0

            drop.set_transform(t)
            if not self.sim_object.collision_check([drop]):
                break

        self.sim_object.collision_set.add(drop)

        return 1
    
    def setup_base_env(self):
        if self.object_count > 6:
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

        return True
        
    def setup_bigger_base(self):
        for obj_name in self.bound_object_name:
            obj = self.sim_object.get_obj(obj_name)
            self.bound_object_name.remove(obj_name)
            self.sim_object.collision_set.remove(obj)
            self.sim_object.env.remove(obj)

        dims = [1,1,0.25]
        new_bound_object = Object(model_gen_utils.create_box(self.sim_object.env,'base',np.eye(4),dims,[0.75,0.5,0]))
        self.sim_object.collision_set.add(new_bound_object)
        self.bound_object_name.append("base")
        self.sim_object.env.Add(new_bound_object)
        t = np.eye(4)
        t[2,3] = -0.38
        new_bound_object.set_transform(t)

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
            x_offsets=[0.1,0.6]
            y_offsets=[0.8,0.1]

        t = self.object_randomizer(plank,x_offsets,y_offsets)            
        if real_world_flag:
            t[:2,3] = [-0.15,0.3]
            plank.set_transform(t)

        return self.sim_object.get_current_state()

    def setup_exp(self,arg=None,req_relation=None,experiment_flag=False):#TODO: hanndle the new naming in Keva
        if not experiment_flag:
            with self.sim_object.env:
            # if True:
                if self.random:
                    x_offsets = [0.2,0.28]
                    y_offsets = [0.2,0.2]
                    self.set_random_goalLoc(x_offsets=x_offsets,y_offsets=y_offsets)
                    
                    plank_transform_list = []
                    for i,p in enumerate(self.plank_list):
                        plank_transform_list.append(p.get_transform())
                        p_num = str(p.get_name()).split("_")[Config.OBJ_ID_IND]
                        t = self.sim_object.get_obj("loc_{}Target_{}".format(Config.OBJECT_NAME[0].split("_")[Config.OBJ_TYPE_IND],p_num)).get_transform()
                        p.set_transform(t)

                    goal_state = self.sim_object.get_one_state()

                    for i,plank in enumerate(self.plank_list):
                        plank.set_transform(plank_transform_list[i])
                
                else:
                    goal_state = self.set_test()
                
                init_state = self.sim_object.get_one_state()

            self.sim_object.remove_droparea()
            return init_state, goal_state, None

    def set_test(self):
        plank_transform_list = []
        with self.sim_object.env:
        # if True:
            self.drop_randomizer(x_offsets=[0.3,0.25],y_offsets=[0.25,0.4])
            self.set_goalLoc()
            
            for i,plank in enumerate(self.plank_list):
                plank_transform_list.append(plank.get_transform())
                plank.set_transform(self.sim_object.get_obj(Config.LOCATION_NAME[-1]+"_{}Target_{}".format(Config.OBJECT_NAME[0].split("_")[Config.OBJ_TYPE_IND],i+1)).get_transform())

            goal_state = self.sim_object.get_one_state()
            
            for i,plank in enumerate(self.plank_list):
                plank.set_transform(plank_transform_list[i])

        return goal_state
