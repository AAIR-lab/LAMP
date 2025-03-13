import Config
import numpy as np
import cPickle
from copy import deepcopy
from src.useful_functions import blockPrinting, print_set
from itertools import product
from scipy.ndimage import label
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import ceil

#env_list = " ".join("env"+str(i) for i in range(1,23)).split(" ")
class RelativeOccupancyMap(object):
    def __init__(self,env_name_list,traj_per_env,total_demo_count,overwrite=False,print_analysis=False,plot=False,debug=False,load_all=False,num_robots=1,process_count=1000000000,seed=0):
        self.env_name_list = env_name_list
        self.seed = seed
        self.env_num_list = []
        for env in self.env_name_list:
            self.env_num_list.append(int(env[3:]))
        #print(self.env_name_list)
        self.file_name = "binned_"+env_name_list[0]+"_to_"+env_name_list[-1]+"_data.p"
        self.traj_per_env = traj_per_env        
        self.overwrite = overwrite
        self.rcr_indices_dic = {}
        self.env_data = {}
        self.print_analysis = print_analysis
        self.plot=plot
        self.debug = debug
        self.load_all = load_all
        self.num_robots = num_robots
        self.process_count = process_count
        self.file_prefix = "{}_{}_".format(total_demo_count,seed)
        self.total_demo_count = total_demo_count
        self.env_traj_dict = {}
        self.matrix_under_use = None
        self.processed_env_count = 0

        if self.debug or self.load_all:
            self.load_all_data()
    
    # @blockPrinting
    def load_data(self,env_name):
        #print("loading {} data".format(env_name))
        file_name = "binned_"+env_name+"_data.p"
        data_load = cPickle.load(open(Config.DATA_MISC_DIR+env_name+"/"+file_name))
        data = data_load["binned_env_states"]
        return data
    
    # @blockPrinting
    def identify_rcr(self,pair_data,key,total_traj_count,angle_bins):
        prim_obj = key.split("_")[0]
        sec_obj = key.split("_")[1]

        traj_count = total_traj_count
        object_pair_set = set([prim_obj,sec_obj])
                
        whole_max = np.max(pair_data[:,:,:,:1,:])
        pair_data = pair_data.astype(np.float16,copy=False)/float(traj_count)        
        decimal_max = np.max(pair_data[:,:,:,:1,:])
        threshold = Config.get_thresholding_value(prim_obj,sec_obj)
        if threshold == "MAX":
            threshold = np.max(np.max(pair_data[:,:,:,:1,:]))

        if object_pair_set == frozenset([Config.OBJECT_NAME[0],Config.GRIPPER_NAME]) or object_pair_set == frozenset([Config.OBJECT_NAME[-1],Config.GRIPPER_NAME]):
            indices = np.argwhere(pair_data[:,:,:,:1,1:]>=threshold)
        else:
            indices = np.argwhere(pair_data[:,:,:,:1,:]>=threshold)
        if self.print_analysis:
            no = len(indices)
            #print(whole_max,decimal_max,no)

        if self.plot:
            #print("while plotting")
            import IPython;IPython.embed()
            #print("exited the plot part")

        secondary_threshold = threshold

        if self.processed_env_count < len(self.env_traj_dict.keys()):
            clear_flag = False
            while clear_flag is not True:               
                data = self.binarize(pair_data,threshold=threshold,secondary_threshold=secondary_threshold)

                grabbed_dim = 1
                if Config.GRIPPER_NAME in key:
                    grabbed_dim = Config.grab_bins
                
                grab_dim_dict = {}
                for d in range(grabbed_dim):
                    k = "data_{}".format(d)
                    grab_dim_dict[k] = deepcopy(data[:,:,:,:,d])

                dic = self.labeling(array_dict=grab_dim_dict)
                clear_flag = self.test_thresholding(dic=dic,array_dict=grab_dim_dict,prim_name=prim_obj,sec_name=sec_obj,angle_bins=angle_bins)
                if not clear_flag:
                    new_secondary_threshold = secondary_threshold - (threshold*0.05)
                    secondary_threshold = deepcopy(max(new_secondary_threshold,0.0))

                del dic
                del grab_dim_dict
            
            if self.matrix_under_use is None:
                self.matrix_under_use = np.zeros(np.shape(data))
            
            self.matrix_under_use = np.logical_or(self.matrix_under_use,data)
            del data

            #print(threshold,secondary_threshold)
            self.processed_env_count += 1

            del pair_data

        if self.processed_env_count == len(self.env_traj_dict.keys()):
            #print("generating RCRs")
            data = self.matrix_under_use
            grabbed_dim = 1
            if Config.GRIPPER_NAME in key:
                grabbed_dim = Config.grab_bins
            
            grab_dim_dict = {}
            for d in range(grabbed_dim):
                k = "data_{}".format(d)
                grab_dim_dict[k] = deepcopy(data[:,:,:,:,d])

            del data

            dic = self.labeling(array_dict=grab_dim_dict)
            clear_flag = self.learn_and_store_rcrs(dic=dic,array_dict=grab_dim_dict,prim_name=prim_obj,sec_name=sec_obj,angle_bins=angle_bins)
            
            del dic
            del grab_dim_dict

            self.matrix_under_use = None
            self.processed_env_count = 0

    def save_plots(self,ax):
        with open(Config.DATA_MISC_DIR+"plot.p" ,"wb") as f:
            cPickle.dump(ax,f,protocol=cPickle.HIGHEST_PROTOCOL)
            f.close()
    
    def plot_points(self,indices):
        x_points=[]
        y_points=[]
        z_points=[]
        ax = plt.axes(projection="3d")

        for ind in indices:
            x_points.append(ind[0])
            y_points.append(ind[1])
            z_points.append(ind[2])

        ax.scatter3D(x_points,y_points,z_points)

        return ax
    
    # @blockPrinting
    def create_frequency_map(self,obj1, obj2, env_list):
        key = obj1+"_"+obj2
        total_traj_count = 0
        #print("generating occupancy for {}".format(key))
        discretizer = Config.get_discretizer(obj1,obj2)        
        angle_bins = discretizer.world_n_bins[3]-2
        relative_bin_arr_size = Config.get_relative_bin_arr_size(discretizer)
        frequency_map = np.zeros(relative_bin_arr_size,dtype=np.uint16)
        # for env in self.env_name_list:
        for env_num in env_list:
            # i = 0
            env = "env{}".format(env_num)
            if self.debug or self.load_all:
                data = self.env_data[env]
            else:
                data = self.load_data(env)

            trajectories_to_process = len(self.env_traj_dict[int(env[3:])])
            trajectory_indices = self.env_traj_dict[int(env[3:])]
            total_traj_count = trajectories_to_process
            # test_ind = [[77,87, 96],#  0  1]
            #             # [77,87, 96],#  0  0]
            # #             [87,103,  96],#   0   0]
            # #             [87,104,  96],#   0   0]
            # #             [87,105,  96],#   0   0]
            # #             [90,76, 96],#  0  0]
            # #             [99,80, 96],#  0  0]
            #             [99,80, 96],#  0  1]
            # #             [100, 81,  96],#   0   0]
            # #             [101, 81,  96],#   0   0]
            # #             [101, 83,  96],#   0   0]
            # #             [102, 81,  96],#   0   0]
            # #             [102, 83,  96],#   0   0]
            # #             [103, 89,  96],#   0   0]
            #             [103, 89,  96],#   0   1]]
            # ]
            # some_indices = []
            
            # while i < trajectories_to_process:
            for i in trajectory_indices:
                relation_flag_dic = set()
                for j in range((np.shape(data[i]))[0]): 
                    state = data[i][j]
                    binned_pose_dict = state.binned_pose_dict
                    for obj1_name in binned_pose_dict.keys():
                        obj1_type = obj1_name.split("_")[0]
                        if obj1_type == obj1:
                            for obj2_name in binned_pose_dict[obj1_name].keys():
                                obj2_type = obj2_name.split("_")[0]
                                if obj2_type == obj2:
                                    index = deepcopy(binned_pose_dict[obj1_name][obj2_name])
                                    if -1 not in index:
                                        # if obj1_type == Config.GRIPPER_NAME and obj2_type == Config.GRIPPER_NAME:
                                        if obj1_type in Config.ROBOT_TYPES and obj2_type in Config.ROBOT_TYPES:
                                            index.append(0)

                                        elif (obj2_type == Config.GRIPPER_NAME):
                                            id = obj2_name.split("_")[1]
                                            grabbed = False
                                            for n in range(1,self.num_robots+1):
                                                grabbed = (grabbed or getattr(state,"grabbed_flag_{}".format(n)))
                                            
                                            if grabbed:
                                                if obj1_name == getattr(state,"grabbed_object_{}".format(id)):
                                                    index.append(1)
                                                else:
                                                    index.append(2)
                                            else:
                                                index.append(0)

                                        elif (obj1_type == Config.GRIPPER_NAME):
                                            id = obj1_name.split("_")[1]
                                            grabbed = False
                                            for n in range(1,self.num_robots+1):
                                                grabbed = (grabbed or getattr(state,"grabbed_flag_{}".format(n)))
                                            
                                            if grabbed:
                                                if obj2_name == getattr(state,"grabbed_object_{}".format(id)):
                                                    index.append(1)
                                                else:
                                                    index.append(2)
                                            else:
                                                index.append(0)
                                            
                                        else:
                                            index.append(0)

                                        x = index[0]
                                        y = index[1]
                                        z = index[2]
                                        g = index[-1]
                                        angles = index[3:-1]
                                        # ind = [x,y,z,0,g]
                                        for ai,angle in enumerate(discretizer.relational_n_bins[3:]):
                                            a = (ai*angle_bins)+angles[ai]+1                                        
                                        ind = [x,y,z,a,g]
                                        if tuple(index) not in relation_flag_dic:
                                            relation_flag_dic.add(tuple(index))                                    
                                            frequency_map[x,y,z,0,g] +=1
                                            for ai,angle in enumerate(discretizer.relational_n_bins[3:]):
                                                # if angles[ai] != 0 and angles[ai] != angle-1:
                                                a = (ai*angle_bins)+angles[ai]+1
                                                frequency_map[x,y,z,a,g] += 1
                                            # if [x,y,z] in test_ind:
                                            #     some_indices.append((i,j))
                
                i = i+1
                del relation_flag_dic
            # self.save_plots(some_indices)
            if not (self.debug or self.load_all):
                del data
                    
            self.identify_rcr(frequency_map,key,total_traj_count,angle_bins)

    def test_thresholding(self,dic=None,array_dict=None,prim_name=None,sec_name=None,angle_bins=None):
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
            shape = list(lab.shape)
            shape.append(1)
            g = int(key.split("_")[1])
            
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
                    angle_bound = angle_bins
                    for index in angle_list:
                        if list(index[:3]) == [x,y,z]:
                            if index[3]<angle_bound:
                                r_list.append(index[3])
                            if index[3]>=angle_bound and index[3]<angle_bound*2:
                                p_list.append(index[3]-angle_bound)
                            if index[3]>=angle_bound*2:
                                y_list.append(index[3]-angle_bound*2)
                                
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
                        temp_arr.append(g)
                        new_rcr.append(tuple(temp_arr))

                grip_list.add(tuple(new_rcr))

            local_rcr_indices_dic[prim_name][sec_name] = local_rcr_indices_dic[prim_name][sec_name].union(grip_list)
            
            return True            
    
    @blockPrinting
    def binarize(self,arr,threshold,secondary_threshold):
        #print("binarizing")
        
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
    
    @blockPrinting
    def labeling(self,array_dict):
        #print("labeling")
        dic = {}
        for key in array_dict:
            arr = deepcopy(array_dict[key].astype(np.float32,copy=False)[:,:,:,0])
            lab,num = label(arr)
            dic[key] = [lab,num]
        
        return dic

    # @blockPrinting
    def learn_and_store_rcrs(self,dic=None,array_dict=None,prim_name=None,sec_name=None,angle_bins=None):
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
            shape = list(lab.shape)
            shape.append(1)
            g = int(key.split("_")[1])
            
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
                    angle_bound = angle_bins
                    for index in angle_list:
                        if list(index[:3]) == [x,y,z]:
                            if index[3]<angle_bound:
                                r_list.append(index[3])
                            if index[3]>=angle_bound and index[3]<angle_bound*2:
                                p_list.append(index[3]-angle_bound)
                            if index[3]>=angle_bound*2:
                                y_list.append(index[3]-angle_bound*2)
                                
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
                        temp_arr.append(g)
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
            total_traj = len(self.env_data[env])
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
        
        self.total_demo_count = len(samples)
        print("total_traj_count = {}".format(self.total_demo_count))
        self.file_prefix = "{}_{}_".format(self.total_demo_count,self.seed)

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
            # key_set = set()      
            temp_data = self.load_data(env_name)
            # for traj in temp_data:
            traj_list = self.env_traj_dict[env_num]
            for t in traj_list:
                traj = temp_data[t]
                if len(traj) > 0:
                    break
                
            dic = traj[0].binned_pose_dict
            for obj1_name in dic.keys():
                for obj2_name in dic[obj1_name].keys():
                    break_flag = False
                    obj1 = obj1_name.split("_")[0]
                    obj2 = obj2_name.split("_")[0]
                    key = obj1+"_"+obj2
                    if key not in key_set:
                        key_set.add(key)
                        if key_string_set is None:
                            # if key not in key_set:
                                if obj1 not in self.rcr_indices_dic.keys():
                                    self.rcr_indices_dic[obj1] = {}
                                if obj2 not in self.rcr_indices_dic[obj1].keys() or self.overwrite:
                                    self.rcr_indices_dic[obj1][obj2] = set([])
                                # if len(self.rcr_indices_dic[obj1][obj2])==0 or self.overwrite:
                                    # if not (obj1 == "can" and obj2 == "freight") and not (obj1=="freight" and obj2=="freight"):
                                # #print(env_name,len(self.env_traj_dict[env_num]))
                                #print(key)
                                key_set.add(key)
                                self.create_frequency_map(obj1,obj2,self.env_traj_dict.keys())
                            
                                # for re in self.rcr_indices_dic[obj1][obj2]:
                                #     print(re)
                                print(key,len(self.rcr_indices_dic[obj1][obj2]))

                        else:
                            if set([obj1,obj2]) == key_string_set:
                                if obj1 not in self.rcr_indices_dic.keys():
                                    self.rcr_indices_dic[obj1] = {}
                                if obj2 not in self.rcr_indices_dic[obj1].keys() or self.overwrite:
                                    self.rcr_indices_dic[obj1][obj2] = set([])
                                # if len(self.rcr_indices_dic[obj1][obj2])==0 or self.overwrite:
                                #print(key)
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
        # self.test_function(self.rcr_indices_dic["gripper"]["can"]) 
        #print("saving critical regions data")
        for prim_obj in self.rcr_indices_dic.keys():
            for sec_obj in self.rcr_indices_dic[prim_obj].keys():
                # rcr_list = self.test_function(self.rcr_indices_dic[prim_obj][sec_obj])
                rcr_list = self.rcr_indices_dic[prim_obj][sec_obj]
                #print("{}_{}".format(prim_obj,sec_obj))
                new_pair_list = []
                for rcr in rcr_list:
                    new_rcr = []
                    for region in rcr:
                        new_rcr.append(list(region))
                    new_pair_list.append(new_rcr)
                    #print(new_rcr)
                self.rcr_indices_dic[prim_obj][sec_obj] = new_pair_list

        data_dict = {
            "rcr_dict":self.rcr_indices_dic,
            "env_traj_dict": self.env_traj_dict
        }
        
        with open(Config.DATA_MISC_DIR+self.file_prefix+"og_rcr_indices.p","wb") as f:
            cPickle.dump(data_dict,f,protocol=cPickle.HIGHEST_PROTOCOL)
            f.close()
        
        with open(Config.DATA_MISC_DIR+self.file_prefix+"rcr_indices.p","wb") as f:
            cPickle.dump(data_dict,f,protocol=cPickle.HIGHEST_PROTOCOL)
            f.close()
            
        #print("critical region saved")

    def test_function(self,regions):
        a = []
        for re in regions:
            if len(a) == 0:
                a.append(set(re))
            else:
                for R in re:
                    found_flag = False
                    test_a = list(R[:3])
                    test_a.append(R[-1])
                    for r in a:
                        for b in r:
                            if test_a[:3] == list(b[:3]) and test_a[-1] == b[-1] :
                                r.add(R)
                                found_flag = True
                                break
                        if found_flag:
                            break
                    if not found_flag:
                        a.append(set(re))
                        break
        
        return a

if __name__ == "__main__":
    import IPython
    IPython.embed()
