from time import sleep
import numpy as np
import importlib
import os
import sys 

from prpy.planning import BiRRTPlanner

def get_parent_with_file(file_name):
    current_dir = os.getcwd()
    while True:
        if file_name in os.listdir(current_dir):
            return current_dir
        else:
            current_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
    
    raise Exception("File Not Found in any Parent")

ROOT_DIR = get_parent_with_file("__init__.py")
if ROOT_DIR is not None and ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

from src.data_structures.EnvState import EnvState
from SimConfig import SimConfig
from Simulator import Simulator
from Object import Object
import model_gen_utils
import sim_utils
import Robots
from useful_functions import sixd_pose_from_transform, transform_from_sixd_pose 

sim = Simulator()
class SimClass(Simulator):
    def __init__(self,robot_name=None,visualize=False,real_world_exp=False):
        self.env = sim.Environment()
        self.env.SetDebugLevel(0)

        self.objects = {}
        self.collision_set = set([])
        self.real_world_exp = real_world_exp

        if robot_name is not None:
            RobotClass = getattr(Robots,robot_name)
            self.robot = RobotClass(env=self.env,id=1,real_world_exp=self.real_world_exp)
            self.robots = [self.robot]

            self.num_robots = len(self.robots)

            self.motion_planner = BiRRTPlanner()
            self.clearance = 0.0

        cc = sim.RaveCreateCollisionChecker(self.env,'pqp')
        cc.SetCollisionOptions(sim.CollisionOptions.AllGeometryContacts)
        self.env.SetCollisionChecker(cc)

        self.viewer = None
        self.visualize = visualize
        if visualize:
            self.viewer = self.visualize_sim()

    @staticmethod
    def conditional_env_lock(method,*args):
        def new_method(self,*args):
            if not self.sim_object.visualize:
                with self.sim_object.env:
                    return method(self,*args)
            else:
                return method(self,*args)

        return new_method
    
    @staticmethod
    def env_lock(method,*args,**kwargs):
        def new_method(self,*args,**kwargs):
            with self.sim_object.env:
                return method(self,*args,**kwargs)

        return new_method

    def load_stl_object(self,object_name,t=np.eye(4)):
        obj = Object(self.env.ReadKinBodyXMLFile(SimConfig.OBJECTS_DIR+object_name+".stl"))
        obj.set_name(object_name)

        obj.set_transform(t)

        return obj
    
    def load_obj(self,object_name):
        self.env.Load(SimConfig.OBJECTS_DIR + object_name)
        obj = Object(self.env.GetBodies()[-1])
        obj.set_name(object_name)

        return obj

    def set_to_last_waypoint(self,trajectory):
        if type(trajectory) != list:
            num = trajectory.GetNumWaypoints()
            last_wp = trajectory.GetWaypoint(num-1)
        else:
            last_wp = trajectory[0]

        if len(last_wp) == 3:
            self.robot.activate_base_joints()
        else:
            self.robot.activate_manip_joints()

        self.robot.set_active_dof_values(last_wp)

    def get_random_pose(self,obj,range_x,range_y,range_z):
        current_t = obj.get_transform()
        while True:
            t = np.eye(4)
            t[0,3] = np.random.uniform(low=range_x[0],high=range_x[1])
            t[1,3] = np.random.uniform(low=range_y[0],high=range_y[1])
            t[2,3] = np.random.uniform(low=range_z[0],high=range_z[1])
            
            obj.set_transform(t)

            if not self.collision_check([obj]):
                obj.set_transform(current_t)
                return t

    def set_pickup_station(self,table_h):
        t = np.eye(4)
        if SimConfig.DOMAIN_NAME == "Keva":
            t[:2,3] = [-0.10625,0.2847]
        elif SimConfig.DOMAIN_NAME == "Jenga":
            table_t = self.get_obj("smalltable").get_transform()
            t[:3,3] = [table_t[0,3]+0.15,table_t[1,3]+0.75,table_h + 0.001]

        pickup_station = self.load_stl_object("pickupstation",t)

        return pickup_station
    
    def remove_planks(self,num_to_remove):
        if type(num_to_remove) == int:
            for obj in self.env.GetBodies()[num_to_remove:]:
                self.env.Remove(obj)
        else:
            for num in num_to_remove:
                self.env.Remove(self.env.GetKinBody("plank_{}".format(num)))

    def rotate_on_x(self,plank_num,rd):
        plank = self.env.GetKinBody("plank_{}".format(plank_num))
        rot_x = sim.matrixFromAxisAngle([rd,0,0])
        plank.SetTransform(plank.GetTransform().dot(rot_x))

    def rotate_on_y(self,plank_num,rd):
        plank = self.env.GetKinBody("plank_{}".format(plank_num))
        rot_y = sim.matrixFromAxisAngle([0,rd,0])
        plank.SetTransform(plank.GetTransform().dot(rot_y))

    def rotate_on_z(self,plank_num,rd):
        plank = self.env.GetKinBody("plank_{}".format(plank_num))
        rot_z = sim.matrixFromAxisAngle([0,0,rd])
        plank.SetTransform(plank.GetTransform().dot(rot_z))

    def translate_in_x(self,plank_num,td):
        plank = self.env.GetKinBody("plank_{}".format(plank_num))
        t = np.eye(4)
        t[0,3] = td
        plank.SetTransform(plank.GetTransform().dot(t))

    def translate_in_y(self,plank_num,td):
        plank = self.env.GetKinBody("plank_{}".format(plank_num))
        t = np.eye(4)
        t[1,3] = td
        plank.SetTransform(plank.GetTransform().dot(t))

    def translate_in_z(self,plank_num,td):
        plank = self.env.GetKinBody("plank_{}".format(plank_num))
        t = np.eye(4)
        t[2,3] = td
        plank.SetTransform(plank.GetTransform().dot(t))
    
    def replace_planks(self,plank1,plank2):
        plank1 = self.env.GetKinBody(plank1)
        plank2 = self.env.GetKinBody(plank2)
        t1 = plank1.GetTransform()
        t2 = plank2.GetTransform()
        plank1.SetTransform(t2)
        plank2.SetTransform(t1)

    def get_bigger_drop_area(self,color=[0.75,0.75,0.75],dims=[0.1,0.1,0],dt=None):
        drop_t = self.remove_droparea()
        if dt is not None:
            drop_t = dt
        droparea = Object(model_gen_utils.create_flat_area(self.env,name="droparea",t=drop_t,color=color,dims=dims))

        return droparea
    
    def remove_droparea(self):
        droparea = self.get_obj("droparea")
        if droparea is not None:
            d_t = droparea.get_transform()
            self.remove_obj("droparea")

            return d_t
        
        return np.eye(4)

    def set_planks_at_goalLoc(self,plank_num=0):
        if plank_num == 0:
            for obj in self.objects.values():
                if SimConfig.OBJECT_NAME[0] in obj.get_name():
                    t = self.get_obj("goalLoc_{}".format(obj.get_name().split("_")[SimConfig.OBJ_ID_IND])).get_transform()
                    obj.set_transform(t)
        else:
            if type(plank_num) == int:
                plank_num = [plank_num]

            for p in plank_num:
                obj = self.get_obj("{}_{}".format(SimConfig.OBJECT_NAME[0],p))
                t = self.get_obj("goalLoc_{}".format(p)).get_transform()
                obj.set_transform(t)
        
        return self.get_current_state()
    
    def move_goalLoc(self,t=np.eye(4),gl=[]):
        if len(gl) == 0:
            for obj in self.objects.values():
                if "goalLoc" in str(obj.get_name()):
                    obj.set_transform(t.dot(obj.get_transform()))
        else:
            for gl_num in gl:
                obj = self.get_obj("goalLoc_{}".format(gl_num))
                obj.set_transform(t.dot(obj.get_transform()))
            
        self.set_planks_at_goalLoc()

        return self.get_current_state()
    
    def rename_planks(self):
        for i, obj in enumerate(self.objects.values()):
            obj.set_name("plank_{}".format(i+1))
    
    def update_env_state(self,env_state,from_state=None):
        if from_state is None: 
            from_state = self.get_current_state()
        for obj in env_state.object_dict.keys():
            if "goalLoc" in obj:
                env_state.object_dict[obj] = from_state.object_dict[obj]
        
        return env_state

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
                    
            for obj_name in env_state.object_dict.keys():
                if obj_name.split("_")[SimConfig.OBJ_TYPE_IND] not in SimConfig.ROBOT_TYPES.keys():
                    obj = self.get_obj(obj_name)
                    if obj is not None:
                        obj.set_transform(transform_from_sixd_pose(env_state.object_dict[obj_name]))
                    else:
                        objects_not_found.append(obj_name)
                else:
                    for rob in self.robots:
                        if rob.id == int(obj_name.split("_")[SimConfig.OBJ_ID_IND]):
                            joints_activation_function = getattr(rob,"activate_{}".format(SimConfig.ROBOT_TYPES[obj_name.split("_")[SimConfig.OBJ_TYPE_IND]]))
                            joints_activation_function()
                            rob.set_active_dof_values(env_state.object_dict[obj_name][0])

            for rob in self.robots:
                if rob.id in id_dict.keys():
                    rob.grab(id_dict[rob.id]["grabbed_object_{}".format(rob.id)])
            
        return objects_not_found
    
    def get_current_state(self):
        return self.get_one_state()
    
    def set_camera_wrt_obj(self,object_name,transform_num=1):
        obj = self.get_obj(object_name)
        relative_t = np.load(SimConfig.CAMERA_ARRAYS+"camera_wrt_{}_{}.npy".format(object_name.split("_")[SimConfig.OBJ_TYPE_IND],transform_num))
        self.env.GetViewer().SetCamera(obj.get_transform().dot(relative_t))

    def save_camera_angle_wrt_obj(self,object_name,transform_num=0):
        obj = self.get_obj(object_name)
        camera_t = self.env.GetViewer().GetCameraTransform()

        relative_t = get_relative_transform(obj.get_transform(),camera_t)
        np.save(SimConfig.CAMERA_ARRAYS+"camera_wrt_{}_{}.npy".format(object_name,transform_num),relative_t)

    def show_region_wrt_plank(self,region_num,plank_num=1,sample_count=100,invert=False):
        plank = self.get_obj("plank_{}".format(plank_num))
        pT = plank.get_transform()
        region = self.regions[region_num-1]

        self.show_region(region,pT,sample_count=sample_count,invert=invert)

    def show_region(self,region,pT,discretizer,sample_count=100,invert=False):
        for i in range(sample_count):
            rel_pose = discretizer.convert_sample(region[0][:6],is_relative=True)
            rel_t = transform_from_sixd_pose(rel_pose)
            if invert:
                p = pT.dot(rel_t)
            else:
                p = pT.dot(np.linalg.pinv(rel_t))
            self.trace.append(self.env.plot3(points = [p[0,3],p[1,3],p[2,3]], pointsize = 0.002, colors = np.array([255,255,0]), drawstyle = 1 ))
    
    def remove_traces(self):
        for t in self.trace:
            t.Close()

    def execute_refinement(self,traj,robot,obj_name=None,lock=True,move_gripper=False,delta_mp=False):
        sleep(0.1)
        if lock:
            with self.env:
            # if True:
                if type(traj).__name__ == "bool":
                    if traj:
                        robot.grab(obj=obj_name,delta_mp=delta_mp)
                    else:
                        robot.release()
                else:
                    try:
                        wp = traj.GetWaypoint(0)
                        if len(wp) == 3:
                            robot.activate_base_joints()
                        else:
                            robot.activate_manip_joints()

                        for i in range(traj.GetNumWaypoints()):
                            wp = traj.GetWaypoint(i)
                            robot.set_active_dof_values(wp)
                            sleep(0.01)
                    except:
                        if len(traj[:-SimConfig.SENSOR_COUNT]) < 4:
                            robot.activate_base_joints()
                        else:
                            robot.activate_manip_joints()

                        robot.set_active_dof_values(traj)

        else:
            if type(traj).__name__ == "bool":
                if traj:
                    if move_gripper:
                        robot.closeGripper(obj=obj_name)
                    else:
                        robot.grab(obj=obj_name)
                else:
                    if move_gripper:
                        robot.openGripper()
                    else:
                        robot.release()
            else:
                try:
                    wp = traj.GetWaypoint(0)
                    if len(wp) == 3:
                        robot.activate_base_joints()
                    else:
                        robot.activate_manip_joints()

                    for i in range(traj.GetNumWaypoints()):
                        wp = traj.GetWaypoint(i)
                        robot.set_active_dof_values(wp)
                        sleep(0.01)
                except:
                    if len(traj) == 3:
                        robot.activate_base_joints()
                    else:
                        robot.activate_manip_joints()

                    robot.set_active_dof_values(traj)

        env_state = self.get_current_state()
        return env_state

    def get_object_dims(self,object_name,use_ref=False):
        if use_ref:
            obj = self.reference_env.GetKinBody(object_name)
        else:            
            obj = self.env.GetKinBody(object_name)
        limits = sim_utils.get_object_limits(obj)
        obj_dim = [abs(limits[1]-limits[0])/2.0,abs(limits[3]-limits[2])/2.0,limits[-1]]
        
        return obj_dim

    def get_env_limits(self,bound_object_name):
        table_1 = self.env.GetKinBody(bound_object_name[0])
        table_1_limits = sim_utils.get_object_limits(table_1)

        env_x = list(table_1_limits[:2])
        env_y = list(table_1_limits[2:-1])

        return [[min(env_x),max(env_x)],[min(env_y),max(env_y)],table_1_limits[-1]]

    def collision_check(self,obj_list,use_collision_set=True):
        collision_list = set(obj_list)
        if use_collision_set:
            collision_list.update(self.collision_set)
            
        with self.env:
        # if True:
            collision = False
            for obj in obj_list:
                if type(obj) == str:
                    if obj in self.robot.robot_type_object_mappings.values():
                        obj = self.robot
                    else:
                        obj = self.get_obj(obj)
                        
                for c_obj in collision_list:
                    if obj != c_obj:
                        grabbed_objects = []
                        for i in range(1,self.num_robots+1):
                            for rob in self.robots:
                                if rob.id == i:
                                    break
                            grabbed_objects.append(getattr(rob,"grabbed_object_{}".format(i)))
                        if c_obj == self.robot and obj.get_name() in grabbed_objects:
                            collision_flag = False
                        else:
                            collision_flag = self.env.CheckCollision(obj.obj,c_obj.obj)
                        collision = collision_flag and \
                                    c_obj.get_name() not in grabbed_objects and \
                                    obj.get_name().split("_")[SimConfig.OBJ_TYPE_IND] not in SimConfig.LOCATION_NAME and \
                                    c_obj.get_name().split("_")[SimConfig.OBJ_TYPE_IND] not in SimConfig.LOCATION_NAME
                        if collision:
                            return collision

        self.collision = collision
        return collision

    def compute_motion_plan(self,goal,robot=None):
        if len(goal) == 3:
            self.robot.activate_base_joints()
        else:
            self.robot.activate_manip_joints()

        try:
            traj = self.motion_planner.PlanToConfiguration(self.robot.obj,goal)
            return traj
        except:
            return None

    def change_obj_name(self,current_name, desired_name):
        obj = self.get_obj(current_name)
        obj.set_name(desired_name)

        del self.objects[current_name]
        
        self.objects[desired_name] = obj
    
    def get_state_block(self,traj,config_tuple,grab=None):
        sleep(0.02)
        object_name,_ = config_tuple
        state = []
        self.set_to_last_waypoint(traj)
        if type(traj) != list:
            state = self.get_state_list(traj)
        else:
            state.append(self.get_one_state())
        
        if grab is not None:
            if grab:
                self.robot.grab(object_name)
            else:
                self.robot.release()

            one_state = self.get_one_state()
            state.append(one_state)

        return state
    
    def get_state_list(self,traj):
        state_list = []
        if type(traj) != list:
            len_traj = traj.GetNumWaypoints()

            for i in range(len_traj):
                wp = traj.GetWaypoint(i)
                if len(wp) == 3:
                    self.robot.activate_base_joints()
                else:
                    self.robot.activate_manip_joints()
                    
                obj_dic = {}
                self.robot.set_active_dof_values(wp)
                for obj in self.objects.values():
                    if obj.get_name() not in SimConfig.BOUND_OBJECT_NAME:
                        if obj != self.robot:
                            name = obj.get_name()
                            obj_dic[name] = sixd_pose_from_transform(obj.get_transform())
                        else:
                            for link in self.robot.robot_type_object_mappings.keys():
                                name_type = self.robot.robot_type_object_mappings[link].split("_")[SimConfig.OBJ_TYPE_IND]
                                joints_activation_function = getattr(self.robot,"activate_{}".format(SimConfig.ROBOT_TYPES[name_type]))
                                joints_activation_function()
                                name = self.robot.robot_type_object_mappings[link]
                                obj_dic[name] = [self.robot.get_active_dof_values(),sixd_pose_from_transform(self.robot.get_link(link).GetTransform())]

                kwargs = {}

                for n in range(1, self.num_robots+1):
                    grabbed_flag_key = "grabbed_flag_{}".format(n)
                    grabbed_object_key = "grabbed_object_{}".format(n)
                    kwargs[grabbed_flag_key] = getattr(self.robot,grabbed_flag_key)
                    kwargs[grabbed_object_key] = getattr(self.robot,grabbed_object_key)
                
                state = EnvState(obj_dic=obj_dic,keyword_arguments=kwargs,num_robots=self.num_robots)
                state_list.append(state)
        
        else:
            wp = traj[0]
            if len(wp) == 3:
                self.robot.activate_base_joints()
            else:
                self.robot.activate_manip_joints()

            self.robot.set_active_dof_values(wp)
            state_list.append(self.get_one_state())

        return state_list
    
    def get_one_state(self):
        obj_dic = {}
        for obj in self.objects.values():
            if obj.get_name() not in SimConfig.BOUND_OBJECT_NAME:
                if obj != self.robot:
                    name = obj.get_name()
                    obj_dic[name] = sixd_pose_from_transform(obj.get_transform())
                else:
                    for link in self.robot.robot_type_object_mappings.keys():
                        name_type = self.robot.robot_type_object_mappings[link].split("_")[SimConfig.OBJ_TYPE_IND]
                        joints_activation_function = getattr(self.robot,"activate_{}".format(SimConfig.ROBOT_TYPES[name_type]))
                        joints_activation_function()
                        name = self.robot.robot_type_object_mappings[link]
                        obj_dic[name] = [self.robot.get_active_dof_values(),sixd_pose_from_transform(self.robot.get_link(link).GetTransform())]
        
        kwargs = {}

        for n in range(1, self.num_robots+1):
            grabbed_flag_key = "grabbed_flag_{}".format(n)
            grabbed_object_key = "grabbed_object_{}".format(n)
            kwargs[grabbed_flag_key] = getattr(self.robot,grabbed_flag_key)
            kwargs[grabbed_object_key] = getattr(self.robot,grabbed_object_key)
        
        state = EnvState(obj_dic=obj_dic,keyword_arguments=kwargs,num_robots=self.num_robots)

        return state
    
    def load_env(self,env_name,reference=False):            
        if reference:
            env_path = SimConfig.REFERENCE_DIR + env_name
        else:
            env_path = SimConfig.ENV_DIR + env_name
        
        successful_load = self.env.Load(env_path)
        for o in self.env.GetBodies():
            obj = Object(o)
            self.objects[obj.get_name()] = obj
        
        return successful_load
    
    def save_env(self,path):
        self.env.Save(path)

    def visualize_sim(self):
        self.env.SetViewer("qtcoin")
        return self.env.GetViewer()
        
    def get_objects(self):
        return self.objects.values() #TODO: make this return ABSTRACT_OBJECTS

    def add_obj(self,obj):
        if obj.get_name() in self.objects.keys() or obj.obj in self.env.GetBodies():
            self.remove_obj(obj)

        self.objects[obj.get_name()] = obj
        self.env.Add(obj.obj)

    def remove_obj(self,obj):
        obj_name = ""
        if type(obj) == str:
            obj_name = obj
            obj = self.get_obj(obj)
        else:
            obj_name = obj.get_name()

        if obj_name in self.objects.keys():
            del self.objects[obj_name]
        
        self.env.Remove(obj.obj)
        obj_names = [o for o in self.collision_set if str(o.get_name()) == obj_name]
        for o in obj_names:
            self.collision_set.remove(o)

    def get_obj(self,obj_name):
        if obj_name not in self.objects:
            for k,v in self.objects.items():
                if obj_name == v.get_name():
                    temp_o = Object(v.obj)
                    del self.objects[k]
                    self.objects[obj_name] = temp_o
                    return temp_o

            return None

        return self.objects[obj_name]

    def get_robot(self):
        return self.robot
    
    def get_robots(self):
        return self.robots

    def __getattr__(self, name):
        if hasattr(self.env,name) and callable(getattr(self.env,name)):
            return getattr(self.env,name)
        else:
            return getattr(sim,name)