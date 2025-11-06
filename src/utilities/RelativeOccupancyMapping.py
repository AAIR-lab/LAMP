from Config import Config
import numpy as np
from copy import deepcopy
from useful_functions import print_set, get_relative_bin_arr_size, load_pickle
from itertools import product
from scipy.ndimage import label
import os
from math import ceil
import sys

import pickle

class RelativeOccupancyMap(object):
    def __init__(self,env_name_list,traj_per_env,total_traj_count,overwrite=False,visualize=False,load_all=False,num_robots=1,process_count=1000000000,prefix=0):
        self.env_name_list = env_name_list
        self.prefix = prefix
        self.env_num_list = []
        for env in self.env_name_list:
            self.env_num_list.append(int(env[3:]))
        #print(self.env_name_list)
        self.file_name = "binned_"+env_name_list[0]+"_to_"+env_name_list[-1]+"_data" + Config.PICKLE_SUFFIX
        self.traj_per_env = traj_per_env        
        self.overwrite = overwrite
        self.rcr_indices_dic = {}
        self.env_data = {}
        self.visualize = visualize
        self.load_all = load_all
        self.num_robots = num_robots
        self.process_count = process_count
        self.file_prefix = "{}_{}_".format(total_traj_count,prefix)
        self.total_traj_count = total_traj_count
        self.env_traj_dict = {}
        self.matrix_under_use = None
        self.processed_env_count = 0

        if self.load_all:
            self.load_all_data()
    
    def load_data(self,env_name):
        file_name = "binned_"+env_name+"_data" + Config.PICKLE_SUFFIX
        data_load = load_pickle(open(Config.DATA_MISC_DIR+env_name+"/"+file_name,"rb"))
        data = data_load["binned_env_states"]

        object_list = load_pickle(open(Config.DATA_MISC_DIR + env_name + "/{}_object_list{}".format(env_name,Config.PICKLE_SUFFIX),"rb"))

        return data, object_list

    def get_sensor_arr(self,arr,sensor_indices):
        temp_arr = deepcopy(arr)
        for i in sensor_indices[::-1]:
            temp_arr = temp_arr[...,i]
        
        return temp_arr

    def filter_rcr(self,pair_data,key,total_traj_count,angle_bin_ranges):
        prim_obj = key.split("_")[0]
        sec_obj = key.split("_")[1]

        traj_count = total_traj_count
        object_pair_set = set([prim_obj,sec_obj])
                
        pair_data = pair_data.astype(np.float16,copy=False)/float(traj_count)        

        threshold = Config.get_eval_thresholding_values(prim_obj,sec_obj)

        secondary_threshold = threshold
        new_secondary_threshold = threshold

        assert self.processed_env_count < len(self.env_traj_dict.keys())
        
        clear_flag = False
        while not clear_flag:
            data = self.binarize(pair_data,
                                 threshold=threshold,
                                 secondary_threshold=secondary_threshold,
                                 angle_bin_ranges=angle_bin_ranges)

            sensor_dims = [tuple([0]*Config.SENSOR_COUNT)]
            if Config.GRIPPER_NAME in key:
                sensor_dims = list(product(*[range(_) for _ in Config.SENSOR_BIN_LIST]))
            
            sensor_dim_dict = {}
            for sensor_tuple in sensor_dims:
                sensor_key_string = "".join("_{}".format(_) for _ in sensor_tuple)
                k = "data" + sensor_key_string
                sensor_arr = self.get_sensor_arr(data,sensor_tuple)
                if len(sensor_arr[sensor_arr > 0.0]) > 0:
                    sensor_dim_dict[k] = deepcopy(sensor_arr)

            dic = self.labeling(array_dict=sensor_dim_dict)
            clear_flag = self.test_thresholding(dic=dic,
                                                array_dict=sensor_dim_dict,
                                                prim_name=prim_obj,
                                                sec_name=sec_obj,
                                                angle_bin_ranges=angle_bin_ranges,
                                                clear_flag=clear_flag)

            if not clear_flag:
                new_secondary_threshold = secondary_threshold - (threshold*0.05)
                secondary_threshold = deepcopy(max(new_secondary_threshold,0.0))

            del dic
            del sensor_dim_dict
        
        if self.matrix_under_use is None:
            self.matrix_under_use = np.zeros(np.shape(data))
        
        self.matrix_under_use = np.logical_or(self.matrix_under_use,data)
        del data

        #print(threshold,secondary_threshold)
        self.processed_env_count += 1

        del pair_data

    def identify_rcr(self,key,angle_bin_ranges):
        prim_obj = key.split("_")[0]
        sec_obj = key.split("_")[1]

        #print("generating RCRs")
        data = self.matrix_under_use
        sensor_dims = [tuple([0]*Config.SENSOR_COUNT)]
        if Config.GRIPPER_NAME in key:
            sensor_dims = list(product(*[range(_) for _ in Config.SENSOR_BIN_LIST]))
        
        sensor_dim_dict = {}
        for sensor_tuple in sensor_dims:
            sensor_key_string = "".join("_{}".format(_) for _ in sensor_tuple)
            k = "data" + sensor_key_string
            sensor_dim_dict[k] = deepcopy(self.get_sensor_arr(data,sensor_tuple))

        del data

        dic = self.labeling(array_dict=sensor_dim_dict)
        clear_flag = self.learn_and_store_rcrs(dic=dic,
                                               array_dict=sensor_dim_dict,
                                               prim_name=prim_obj,
                                               sec_name=sec_obj,
                                               angle_bin_ranges=angle_bin_ranges)
        
        del dic
        del sensor_dim_dict

        self.matrix_under_use = None
        self.processed_env_count = 0
    
    def create_frequency_map(self,obj1, obj2, env_list):
        key = obj1+"_"+obj2
        total_traj_count = 0
        
        discretizer = Config.get_discretizer(obj1,obj2)        
        angle_bins = discretizer.world_n_bins[3:]
        angle_bin_ranges = [(0,angle_bins[0])]
        for i, ab in enumerate(angle_bins[1:]):
            _, prev_end = angle_bin_ranges[i]
            angle_bin_ranges.append((prev_end,prev_end + ab))
            
        relative_bin_arr_size = get_relative_bin_arr_size(discretizer)
        frequency_map = np.zeros(relative_bin_arr_size,dtype=np.uint16)
        
        for env_num in env_list:
            env = "env{}".format(env_num)
            if self.load_all:
                data,object_list = self.env_data[env]
            else:
                data,object_list = self.load_data(env)

            if not set([obj1,obj2]).issubset(set([o.split("_")[Config.OBJ_TYPE_IND] for o in object_list])):
                self.processed_env_count += 1
                continue

            trajectories_to_process = len(self.env_traj_dict[int(env[3:])])
            trajectory_indices = self.env_traj_dict[int(env[3:])]
            total_traj_count = trajectories_to_process

            for i in trajectory_indices:
                relation_flag_dic = set()
                for j in range((np.shape(data[i]))[0]): 
                    state = data[i][j]
                    binned_pose_dict = state.binned_pose_dict
                    for obj1_name in binned_pose_dict.keys():
                        obj1_type = obj1_name.split("_")[Config.OBJ_TYPE_IND]
                        if obj1_type == obj1:
                            for obj2_name in binned_pose_dict[obj1_name].keys():
                                obj2_type = obj2_name.split("_")[Config.OBJ_TYPE_IND]
                                if obj2_type == obj2:
                                    index = deepcopy(binned_pose_dict[obj1_name][obj2_name])
                                    if -1 not in index:
                                        grab_val = 0
                                        if obj1_type not in Config.ROBOT_TYPES or obj2_type not in Config.ROBOT_TYPES:                                            
                                            if (obj2_type == Config.GRIPPER_NAME):
                                                id = obj2_name.split("_")[Config.OBJ_ID_IND]
                                                grabbed = False
                                                for n in range(1,self.num_robots+1):
                                                    grabbed = (grabbed or getattr(state,"grabbed_flag_{}".format(n)))
                                                
                                                if grabbed:
                                                    if obj1_name == getattr(state,"grabbed_object_{}".format(id)):
                                                        grab_val = 1
                                                    else:
                                                        grab_val = 2
    
                                            elif (obj1_type == Config.GRIPPER_NAME):
                                                id = obj1_name.split("_")[Config.OBJ_ID_IND]
                                                grabbed = False
                                                for n in range(1,self.num_robots+1):
                                                    grabbed = (grabbed or getattr(state,"grabbed_flag_{}".format(n)))
                                                
                                                if grabbed:
                                                    if obj2_name == getattr(state,"grabbed_object_{}".format(id)):
                                                        grab_val = 1
                                                    else:
                                                        grab_val = 2

                                        index.extend(Config.SENSOR_COUNT*[0])
                                        index[Config.GRAB_INDEX] = grab_val

                                        pos_ind = index[:3]
                                        angles = index[3:-Config.SENSOR_COUNT]
                                        sensor_ind = index[-Config.SENSOR_COUNT:]
                                        
                                        if tuple(index) not in relation_flag_dic:
                                            relation_flag_dic.add(tuple(index))
                                            item_indices = tuple(pos_ind + [0] + sensor_ind)                      
                                            if Config.PYTHON_VER_INT == 2:
                                                frequency_map.itemset(item_indices, frequency_map.item(item_indices) + 1)
                                            else:
                                                frequency_map[item_indices] = frequency_map[item_indices] + 1
                                                
                                            for ai,(start, _) in enumerate(angle_bin_ranges):
                                                a = start + angles[ai] + 1
                                                item_indices = tuple(pos_ind + [a] + sensor_ind)
                                                if Config.PYTHON_VER_INT == 2:
                                                    frequency_map.itemset(item_indices, frequency_map.item(item_indices) + 1)
                                                else:
                                                    frequency_map[item_indices] = frequency_map[item_indices] + 1
                
                del relation_flag_dic
            
            if not self.load_all:
                del data

            self.filter_rcr(frequency_map,key,total_traj_count,angle_bin_ranges)
        
        assert self.processed_env_count == len(self.env_traj_dict.keys())
        self.identify_rcr(key,angle_bin_ranges)

    def test_thresholding(self,dic=None,array_dict=None,prim_name=None,sec_name=None,angle_bin_ranges=[],clear_flag=False):
        local_rcr_indices_dic = {}
        arr_dict = array_dict
        for key in arr_dict.keys():
            grip_list = set([])
            arr_new = arr_dict[key][:,:,:,1:]
            angle_list = []
            angle_list.extend(np.argwhere(arr_new==1.0))
            if prim_name not in local_rcr_indices_dic.keys():
                local_rcr_indices_dic[prim_name] = {}
            if sec_name not in local_rcr_indices_dic[prim_name].keys():
                local_rcr_indices_dic[prim_name][sec_name] = set([])

            lab = dic[key][0]
            num = dic[key][1]

            sensor_ind = [int(_) for _ in key.split("_")[1:]]
            
            for label_val in range(1,num+1):
                rcr = []
                ind = np.argwhere(lab==label_val)
                for i in ind:
                    x,y,z = i
                    rcr.append([x,y,z])
                new_rcr = []
                for ind in rcr:
                    [x,y,z] = ind[:3]
                    pos = [x,y,z]
                    r_list = []
                    p_list = []
                    y_list = []
                    orienation_indices = [r_list,p_list,y_list]
                    
                    for index in angle_list:
                        if list(index[:3]) == [x,y,z]:
                            val = index[3]
                            for orn_index, orn_list in enumerate(orienation_indices):
                                start,end = angle_bin_ranges[orn_index]
                                if start <= val < end:
                                    orn_list.append(val - start)
                                
                    if len(r_list) == 0:
                        return False 
                    if len(p_list) == 0:
                        return False
                    if len(y_list) == 0:
                        return False 

                    combinations = list(product(r_list,p_list,y_list))
                    for comb in combinations:
                        temp_arr = deepcopy(pos)
                        temp_arr.extend(comb)
                        temp_arr.extend(sensor_ind)
                        new_rcr.append(tuple(temp_arr))

                grip_list.add(tuple(new_rcr))

            local_rcr_indices_dic[prim_name][sec_name] = local_rcr_indices_dic[prim_name][sec_name].union(grip_list)
            
        return True            
    
    def binarize(self,arr,threshold,secondary_threshold,angle_bin_ranges):
        temp = deepcopy(arr[:,:,:,:1,:])
        temp[temp>=threshold] = 1
        temp[temp<threshold] = 0

        temp2 = deepcopy(arr[:,:,:,1:,:])
        if secondary_threshold > 0.0:
            temp2[temp2>=secondary_threshold] = 1
            temp2[temp2<secondary_threshold] = 0
        else:
            temp2[temp2>secondary_threshold] = 1
            temp2[temp2<=secondary_threshold] = 0

        return np.concatenate((temp,temp2),axis=3)
    
    def labeling(self,array_dict):
        #print("labeling")
        dic = {}
        for key in array_dict:
            arr = deepcopy(array_dict[key].astype(np.float32,copy=False)[:,:,:,0])
            lab,num = label(arr)
            dic[key] = [lab,num]
        
        return dic
    
    def learn_and_store_rcrs(self,dic=None,array_dict=None,prim_name=None,sec_name=None,angle_bin_ranges=[]):
        #print("storing critical regions")
        local_rcr_indices_dic = {}
        arr_dict = array_dict
        for key in arr_dict.keys():
            grip_list = set([])
            arr_new = arr_dict[key][:,:,:,1:]
            angle_list = []
            angle_list.extend(np.argwhere(arr_new==1.0))
            if prim_name not in local_rcr_indices_dic.keys():
                local_rcr_indices_dic[prim_name] = {}
            if sec_name not in local_rcr_indices_dic[prim_name].keys():
                local_rcr_indices_dic[prim_name][sec_name] = set([])

            lab = dic[key][0]
            num = dic[key][1]

            sensor_ind = [int(_) for _ in key.split("_")[1:]]
            
            for label_val in range(1,num+1):
                rcr = []
                ind = np.argwhere(lab==label_val)
                for i in ind:
                    x,y,z = i
                    rcr.append([x,y,z])
                new_rcr = []
                for ind in rcr:
                    [x,y,z] = ind[:3]
                    pos = [int(x),int(y),int(z)]
                    r_list = []
                    p_list = []
                    y_list = []
                    orienation_indices = [r_list,p_list,y_list]
                    
                    for index in angle_list:
                        if list(index[:3]) == [x,y,z]:
                            val = index[3]
                            for orn_index, orn_list in enumerate(orienation_indices):
                                start,end = angle_bin_ranges[orn_index]
                                if start <= val < end:
                                    orn_list.append(int(val - start))
                                
                    if len(r_list) == 0:
                        continue
                    if len(p_list) == 0:
                        continue
                    if len(y_list) == 0:
                        continue

                    combinations = list(product(r_list,p_list,y_list))
                    for comb in combinations:
                        temp_arr = deepcopy(pos)
                        temp_arr.extend(comb)
                        temp_arr.extend(sensor_ind)
                        new_rcr.append(tuple(temp_arr))

                if len(new_rcr) != 0:
                    grip_list.add(tuple(new_rcr))

            local_rcr_indices_dic[prim_name][sec_name] = local_rcr_indices_dic[prim_name][sec_name].union(grip_list)
        
        if prim_name not in self.rcr_indices_dic.keys():
            self.rcr_indices_dic[prim_name] = {}
        if sec_name not in self.rcr_indices_dic[prim_name].keys():
            self.rcr_indices_dic[prim_name][sec_name] = set([])
        self.rcr_indices_dic[prim_name][sec_name] = self.rcr_indices_dic[prim_name][sec_name].union(local_rcr_indices_dic[prim_name][sec_name])
        
        return True

    def load_all_data(self):
        for env in self.env_name_list:
            self.env_data[env] = self.load_data(env)
        return True

    def create_traj_pool(self):
        traj_pool = {}
        for env in self.env_name_list:
            env_num = int(env[3:])
            total_traj = len(self.env_data[env][0])
            for traj_num in range(total_traj):
                if env_num not in traj_pool:
                    traj_pool[env_num] = []
                traj_pool[env_num].append(traj_num)
        
        return traj_pool

    def get_random_sampling(self,traj_pool):
        samples = []
        for env in traj_pool.keys():
            min_env_count = int(ceil(self.traj_per_env*len(traj_pool[env])))
            sample_traj = np.random.choice(traj_pool[env],size=min_env_count,replace=False)
            for traj_num in sample_traj:
                samples.append((env,traj_num))
        
        self.total_traj_count = len(samples)
        print("total_traj_count = {}".format(self.total_traj_count))
        self.file_prefix = "{}_{}_".format(self.total_traj_count,self.prefix)

        return samples

    def get_env_traj_dict(self,random_samplings):
        env_traj_dict = {}
        for env_num,traj_num in random_samplings:
            if env_num not in env_traj_dict.keys():
                env_traj_dict[env_num] = []
            env_traj_dict[env_num].append(traj_num)
        
        return env_traj_dict

    def start(self,key_string_set=None):       
        traj_pool = self.create_traj_pool() 
        trajectories_to_use = self.get_random_sampling(traj_pool)
        self.env_traj_dict = self.get_env_traj_dict(trajectories_to_use)
        key_set = set([])
        for env_num in self.env_traj_dict.keys():
            env_name = "env{}".format(env_num)
            temp_data,_ = self.load_data(env_name)
            traj_list = self.env_traj_dict[env_num]
            for t in traj_list:
                traj = temp_data[t]
                if len(traj) > 0:
                    break
                
            dic = traj[0].binned_pose_dict
            for obj1_name in dic.keys():
                for obj2_name in dic[obj1_name].keys():
                    break_flag = False
                    obj1 = obj1_name.split("_")[Config.OBJ_TYPE_IND]
                    obj2 = obj2_name.split("_")[Config.OBJ_TYPE_IND]
                    key = obj1+"_"+obj2
                    if key not in key_set:
                        key_set.add(key)
                        if key_string_set is None:
                            if obj1 not in self.rcr_indices_dic.keys():
                                self.rcr_indices_dic[obj1] = {}
                            if obj2 not in self.rcr_indices_dic[obj1].keys() or self.overwrite:
                                self.rcr_indices_dic[obj1][obj2] = set([])

                            key_set.add(key)
                            self.create_frequency_map(obj1,obj2,self.env_traj_dict.keys())

                            if self.visualize:
                                for re in self.rcr_indices_dic[obj1][obj2]:
                                    print(re)

                            print(key,len(self.rcr_indices_dic[obj1][obj2]))

                        else:
                            if set([obj1,obj2]) == key_string_set:
                                if obj1 not in self.rcr_indices_dic.keys():
                                    self.rcr_indices_dic[obj1] = {}
                                if obj2 not in self.rcr_indices_dic[obj1].keys() or self.overwrite:
                                    self.rcr_indices_dic[obj1][obj2] = set([])

                                key_set.add(key)
                                self.create_frequency_map(obj1,obj2,self.env_traj_dict.keys())
                                    
                                for re in self.rcr_indices_dic[obj1][obj2]:
                                    print(re)
                                print(key,len(self.rcr_indices_dic[obj1][obj2]))
                                
                                break_flag = True
                                break
                    
                    if break_flag:
                        break
        
        self.save()
    
    def save(self):
        for prim_obj in self.rcr_indices_dic.keys():
            for sec_obj in self.rcr_indices_dic[prim_obj].keys():
                rcr_list = self.rcr_indices_dic[prim_obj][sec_obj]
                new_pair_list = []
                for rcr in rcr_list:
                    new_rcr = []
                    for region in rcr:
                        new_rcr.append(list(region))
                    new_pair_list.append(new_rcr)
                self.rcr_indices_dic[prim_obj][sec_obj] = new_pair_list

        with open(Config.DATA_MISC_DIR+self.file_prefix+"trajs_used"+Config.PICKLE_SUFFIX,"wb") as f:
            pickle.dump(self.env_traj_dict,f,protocol=Config.PICKLE_PROTOCOL )
            f.close()
        
        with open(Config.DATA_MISC_DIR+self.file_prefix+"og_rcr_indices"+Config.PICKLE_SUFFIX,"wb") as f:
            pickle.dump(self.rcr_indices_dic,f,protocol=Config.PICKLE_PROTOCOL)
            f.close()
        
        with open(Config.DATA_MISC_DIR+self.file_prefix+"rcr_indices"+Config.PICKLE_SUFFIX,"wb") as f:
            pickle.dump(self.rcr_indices_dic,f,protocol=Config.PICKLE_PROTOCOL )
            f.close()
            
if __name__ == "__main__":
    import IPython
    IPython.embed()
