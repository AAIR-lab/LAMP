from Environments.Keva import object_models
from src.data_structures.EnvState import EnvState
from time import sleep
import numpy as np
import importlib
import openravepy as orpy
import Config
from src.useful_functions import get_relative_transform,create_dag,print_set
import cPickle

def GetRaveExecution(domain_name,robot_name):
    mod = importlib.import_module("src.data_gen.{}".format(domain_name))
    DomainFunction = getattr(mod,"GetDomain")
    DomainClass = DomainFunction(robot_name)
    
    class RaveExecution(DomainClass):
        def __init__(self,env_name,axis_for_offset,test_structure_name=None,visualize = False,plank_count=1, random=False,surface="",planks_in_init_state=0,num_robots=1,order=False,object_list=["bowl","glass"],experiment_flag=False,real_world_experiment=False,prefixes=[None,None]):
            super(RaveExecution,self).__init__(env_name=env_name,
                                               number_of_configs=0,
                                               number_of_mp=0,
                                               file_name=env_name+"_data.p",
                                               axis_for_offset=axis_for_offset,
                                               visualize=visualize,
                                               plank_count=plank_count,
                                               random=random,
                                               reference_structure_name=test_structure_name,
                                               surface=surface,
                                               planks_in_init_state=planks_in_init_state,
                                               order=True,
                                               num_robots=num_robots,
                                               object_list=object_list,
                                               experiment_flag=experiment_flag,
                                               real_world_experiment=real_world_experiment,
                                               data_gen=False)
            
            if type(random) != bool:
                self.random = random==0
            else:
                self.random = random

            if not experiment_flag:
                self.setup_base_env()
                if "Jenga" == Config.DOMAIN_NAME:
                    self.set_pickup_station()
            else:
                if "Keva" == Config.DOMAIN_NAME or "Jenga" == Config.DOMAIN_NAME:
                    self.remove_droparea()
                    if "Jenga" == Config.DOMAIN_NAME:
                        self.set_pickup_station()

            # self.trace = []
            # self.env_prefix,self.file_prefix = prefixes
            # self.regions = self.get_regions_from_dag(structure_name=test_structure_name)
            # self.env.Remove(self.env.GetKinBody("table6"))
            # self.get_bigger_drop_area(color=[0.999,0.999,0.999],dt=np.eye(4))
            # table_t = self.env.GetKinBody("table6").GetTransform()
            # dims = self.get_object_dims("table6")
            # dims[2] = 0
            # self.env.Remove(self.env.GetKinBody("table6"))
            # new_table = self.get_bigger_drop_area(color=[0.999,0.999,0.999],dims=dims)
            self.clearance = 0.0

        def load_stl_object(self,object_name,t=np.eye(4)):
            obj = self.env.ReadKinBodyXMLFile(Config.OBJECTS_DIR+object_name+".stl")
            obj.SetName(object_name)

            obj.SetTransform(t)
            self.env.Add(obj)
            self.collision_set.add(obj)

            return obj
        
        def set_pickup_station(self):
            t = np.eye(4)
            if Config.DOMAIN_NAME == "Keva":
                t[:2,3] = [-0.10625,0.2847]
            elif Config.DOMAIN_NAME == "Jenga":
                table_t = self.env.GetKinBody("smalltable").GetTransform()

                t[:3,3] = [table_t[0,3]+0.15,table_t[1,3]+0.75,self.table_h + 0.001]

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
            rot_x = orpy.matrixFromAxisAngle([rd,0,0])
            plank.SetTransform(plank.GetTransform().dot(rot_x))

        def rotate_on_y(self,plank_num,rd):
            plank = self.env.GetKinBody("plank_{}".format(plank_num))
            rot_y = orpy.matrixFromAxisAngle([0,rd,0])
            plank.SetTransform(plank.GetTransform().dot(rot_y))

        def rotate_on_z(self,plank_num,rd):
            plank = self.env.GetKinBody("plank_{}".format(plank_num))
            rot_z = orpy.matrixFromAxisAngle([0,0,rd])
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
            droparea = object_models.create_flat_area(self.env,name="droparea",t=drop_t,color=color,dims=dims)
            return droparea
        
        def load_dag(self,structure_name,create=False,rcr_dict={}):
            try:
                with open(Config.DAG_DIR + structure_name + "_dag.p","rb") as f:
                    data = cPickle.load(f)
                    f.close()
            except:
                create = True
            
            if create:
                if len(rcr_dict.values()) == 0:
                    with open(Config.DATA_MISC_DIR+self.file_prefix+"og_"+"rcr_indices.p","rb") as f:
                        data = cPickle.load(f)
                        rcr_dict = data["rcr_dict"]
                        f.close()

                discretizer = Config.get_discretizer(obj1=Config.OBJECT_NAME[0],obj2=Config.OBJECT_NAME[0])
                data=create_dag(structure=structure_name,
                                cr_list=rcr_dict[Config.OBJECT_NAME[0]][Config.OBJECT_NAME[0]],
                                discretizer=discretizer)
        
            return data["dag"],data["unknown_plank_relations"],rcr_dict[Config.OBJECT_NAME[0]][Config.OBJECT_NAME[0]]
        
        def remove_droparea(self):
            droparea = self.env.GetKinBody("droparea")
            if droparea is not None:
                d_t = droparea.GetTransform()
                self.env.Remove(droparea)
                self.collision_set.remove(droparea)

                return d_t
            
            return np.eye(4)

        def set_planks_at_goalLoc(self,plank_num=0):
            if plank_num == 0:
                for obj in self.env.GetBodies():
                    if Config.OBJECT_NAME[0] in str(obj.GetName()):
                        t = self.env.GetKinBody("goalLoc_{}".format(str(obj.GetName()).split("_")[1])).GetTransform()
                        obj.SetTransform(t)
            else:
                if type(plank_num) == int:
                    plank_num = [plank_num]
                for p in plank_num:
                    obj = self.env.GetKinBody("{}_{}".format(Config.OBJECT_NAME[0],p))
                    t = self.env.GetKinBody("goalLoc_{}".format(p)).GetTransform()
                    obj.SetTransform(t)
            
            return self.get_current_state()
        
        def move_goalLoc(self,t=np.eye(4),gl=[]):
            if len(gl) == 0:
                for obj in self.env.GetBodies():
                    if "goalLoc" in str(obj.GetName()):
                        obj.SetTransform(t.dot(obj.GetTransform()))
            else:
                for gl_num in gl:
                    obj = self.env.GetKinBody("goalLoc_{}".format(gl_num))
                    obj.SetTransform(t.dot(obj.GetTransform()))
                
            self.set_planks_at_goalLoc()

            return self.get_current_state()
        
        def rename_planks(self):
            for i, obj in enumerate(self.env.GetBodies()):
                obj.SetName("plank_{}".format(i+1))
        
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
        
        def get_current_state(self):
            return self.get_one_state()
        
        def set_camera_wrt_obj(self,object_name,transform_num=1):
            obj = self.env.GetKinBody(object_name)
            relative_t = np.load(Config.ROOT_DIR+"camera_wrt_{}_{}.npy".format(object_name.split("_")[0],transform_num))
            self.env.GetViewer().SetCamera(obj.GetTransform().dot(relative_t))

        def save_camera_angle_wrt_obj(self,object_name,transform_num=0):
            obj = self.env.GetKinBody(object_name)
            camera_t = self.env.GetViewer().GetCameraTransform()

            relative_t = get_relative_transform(obj.GetTransform(),camera_t)
            np.save(Config.ROOT_DIR+"camera_wrt_{}_{}.npy".format(object_name,transform_num),relative_t)

        def get_regions_from_dag(self,structure_name):
            _, missing_relations,known_rcr_regions = self.load_dag(structure_name)
            for p in missing_relations.keys():
                for node in missing_relations[p]:
                    edge = node[1]
                    regions = edge[2]
                    known_rcr_regions.append(regions)
            
            return known_rcr_regions

        def show_region_wrt_plank(self,region_num,plank_num=1,sample_count=100,invert=False):
            plank = self.env.GetKinBody("plank_{}".format(plank_num))
            pT = plank.GetTransform()
            region = self.regions[region_num-1]

            self.show_region(region,pT,sample_count=sample_count,invert=invert)

        def show_region(self,region,pT,sample_count=100,invert=False):
            discretizer = Config.get_discretizer("plank","plank")
            for i in range(sample_count):
                rel_pose = discretizer.convert_sample(region[0][:6],is_relative=True)
                rel_t = self.transform_from_pose(rel_pose)
                if invert:
                    p = pT.dot(rel_t)
                else:
                    p = pT.dot(np.linalg.pinv(rel_t))
                self.trace.append(self.env.plot3(points = [p[0,3],p[1,3],p[2,3]], pointsize = 0.002, colors = np.array([255,255,0]), drawstyle = 1 ))
        
        def remove_traces(self):
            for t in self.trace:
                t.Close()

        def execute_refinement(self,traj,robot,obj_name="plank_1",lock=True,move_gripper=False):
            sleep(0.1)
            if lock:
                with self.env:
                # if True:
                    if type(traj).__name__ == "bool":
                        if traj:
                            robot.grab(obj=obj_name)
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
                                robot.robot.SetActiveDOFValues(wp)
                                sleep(0.01)
                        except:
                            if len(traj[:-1]) < 4:
                                robot.activate_base_joints()
                            else:
                                robot.activate_manip_joints()

                            robot.robot.SetActiveDOFValues(traj)

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
                            robot.robot.SetActiveDOFValues(wp)
                            sleep(0.01)
                    except:
                        if len(traj) == 3:
                            robot.activate_base_joints()
                        else:
                            robot.activate_manip_joints()

                        robot.robot.SetActiveDOFValues(traj)

            env_state = self.get_current_state()
            return env_state
        
    return RaveExecution