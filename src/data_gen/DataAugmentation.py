from src.data_structures.BinnedEnvState import BinnedEnvState
from openravepy import *
import Config
import numpy as np
import cPickle
import tqdm
import time

class DataAugmenter(object):
    def __init__(self,env_name,itteration_count = None,num_robots=1):
        self.env_name = env_name
        print(self.env_name)
        self.file_name = env_name+"_data.p"
        self.outliers = []
        self.data = self.load_data()
        if itteration_count is not None:
            self.itteration_count = itteration_count
        else:
            self.itteration_count = len(self.data)
        self.aug_data = []
        self.binned_data = []
        self.num_robots = num_robots
        
    def load_data(self):
        print("loading {} data".format(self.env_name))
        data_load = cPickle.load(open(Config.DATA_MISC_DIR+self.env_name+"/"+self.file_name))
        data = data_load["env_states"]
        return data

    def augment_data(self,trajectories,object_pair = None):
        i = 0
        print("augmenting {} data file".format(self.env_name))
        pbar = tqdm.tqdm(total = self.itteration_count)
        data = trajectories
        
        while i < self.itteration_count:
            binned_state_list = []
            for j in range((np.shape(data[i]))[0]):
                env_state = data[i][j]
                object_names = []
                for object_name in env_state.object_dict.keys():
                    object_names.append(object_name)
                object_names.sort()
                relative_object_dict = {}
                object_pair_list = []
                for a,obj1 in enumerate(object_names):
                    obj1_type = obj1.split("_")[0]
                    if obj1_type not in Config.NON_RELATION_OBJECTS:
                        for b, obj2 in enumerate(object_names):
                            obj2_type = obj2.split("_")[0]
                            if object_pair is not None:
                                if obj1_type not in object_pair and obj2_type not in object_pair:
                                    continue
                            if obj2 != obj1 and self.filter(obj1_type,obj2_type) and frozenset([obj1,obj2]) not in object_pair_list:
                                object_pair_list.append(frozenset([obj1,obj2]))
                                if (obj1_type not in Config.ROBOT_TYPES and obj2_type not in Config.ROBOT_TYPES) or (obj1_type in Config.ROBOT_TYPES and obj2_type in Config.ROBOT_TYPES):
                                    prim_object = obj1
                                    sec_object = obj2
                                else:
                                    if obj1_type in Config.ROBOT_TYPES:
                                        prim_object = obj1
                                        sec_object = obj2
                                    else:
                                        prim_object = obj2
                                        sec_object = obj1
                                        
                                if prim_object not in relative_object_dict.keys():
                                    relative_object_dict[prim_object] = {}
                                if sec_object not in relative_object_dict[prim_object]:
                                    relative_object_dict[prim_object][sec_object] = None

                for prim_object in relative_object_dict.keys():
                    relative_transform_dict = relative_object_dict[prim_object]
                    prim_type = prim_object.split("_")[0]
                    if prim_type in Config.ROBOT_TYPES:
                        prim_pose = env_state.object_dict[prim_object][1]
                    else:
                        prim_pose =env_state.object_dict[prim_object]

                    for sec_object in relative_transform_dict.keys():
                        sec_type = sec_object.split("_")[0]
                        if object_pair is not None:
                            if prim_type not in object_pair and sec_type not in object_pair:
                                continue
                        if sec_type in Config.ROBOT_TYPES:
                            sec_pose = env_state.object_dict[sec_object][1]
                        else:
                            sec_pose = env_state.object_dict[sec_object]
                            
                        if (prim_type in Config.OBJECT_NAME and sec_type in Config.LOCATION_NAME) or (prim_type in Config.LOCATION_NAME and sec_type in Config.OBJECT_NAME):
                            if prim_object.split("_")[1] == sec_object.split("_")[1]:
                                relative_transform_dict[sec_object] = env_state.get_relative_pose(prim_pose,sec_pose)
                        elif not(prim_type in Config.LOCATION_NAME and sec_type in Config.LOCATION_NAME):
                            relative_transform_dict[sec_object] = env_state.get_relative_pose(prim_pose,sec_pose)
                        
                for prim_object in relative_object_dict.keys():
                    for sec_object in relative_object_dict[prim_object].keys():
                        if relative_object_dict[prim_object][sec_object] is None:
                            del relative_object_dict[prim_object][sec_object]
                            
                kwargs = {}
                for n in range(1, self.num_robots+1):
                    grabbed_flag_key = "grabbed_flag_{}".format(n)
                    grabbed_object_key = "grabbed_object_{}".format(n)
                    kwargs[grabbed_flag_key] = getattr(env_state,grabbed_flag_key)
                    kwargs[grabbed_object_key] = getattr(env_state,grabbed_object_key)

                binned_state = BinnedEnvState(relative_object_dict,keyword_arguments=kwargs)
                binned_state_list.append(binned_state)

            pbar.update(1)
           
            self.binned_data.append(binned_state_list)
            i = i+1
            time.sleep(0.1)
        
        final_binned_data = {"binned_env_states": self.binned_data}
        pbar.close()
        time.sleep(5)
        cPickle.dump(final_binned_data,open(Config.DATA_MISC_DIR+self.env_name+"/" +"binned_"+self.file_name ,"wb"),protocol=cPickle.HIGHEST_PROTOCOL)
        print("binned {} saved".format(self.env_name))   

    def filter(self,obj1,obj2):
        robots_and_location = not (obj1 in Config.ROBOT_TYPES.keys() and obj2 in Config.LOCATION_NAME) and not (obj2 in Config.ROBOT_TYPES.keys() and obj1 in Config.LOCATION_NAME)
        if Config.DOMAIN_NAME == "DinnerTable":
            robots_and_location = not (obj1 == "gripper" and obj2 in Config.LOCATION_NAME) and not (obj2 == "gripper" and obj1 in Config.LOCATION_NAME)

        location_and_surface = not (obj1 == Config.SURFACE_NAME and obj2 in Config.LOCATION_NAME) and not (obj2 == Config.SURFACE_NAME and obj1 in Config.LOCATION_NAME)
        surface_check = not (obj1 == Config.SURFACE_NAME and obj2 == Config.SURFACE_NAME)
        surface_and_gripper = not (obj1 == Config.SURFACE_NAME and obj2 == Config.GRIPPER_NAME) and not (obj2 == Config.SURFACE_NAME and obj1 == Config.GRIPPER_NAME)
        if Config.DOMAIN_NAME == "DinnerTable":
            location_and_surface = not (obj1 == Config.SURFACE_NAME or obj2 == Config.SURFACE_NAME)
            surface_check = not (obj1 == Config.SURFACE_NAME or obj2 == Config.SURFACE_NAME)
            surface_and_gripper = not (obj1 == Config.SURFACE_NAME or obj2 == Config.SURFACE_NAME)
        
        non_relation_objects_check = obj2 not in Config.NON_RELATION_OBJECTS
        object_and_base_filter = True
        if Config.DOMAIN_NAME == "CafeWorld":
            # object_and_base_filter = not (obj1 == Config.BASE_NAME and obj2 in Config.OBJECT_NAME) and not (obj1 in Config.OBJECT_NAME and obj2 == Config.BASE_NAME)
            if "goalLoc" in obj1 or "goalLoc" in obj2:
                return False
                
        return robots_and_location and location_and_surface and surface_check and non_relation_objects_check and surface_and_gripper and object_and_base_filter

if __name__ == "__main__":
    import IPython
    IPython.embed()
