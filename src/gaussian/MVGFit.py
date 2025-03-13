import numpy as np
import Config
import pickle
from copy import deepcopy
from scipy.stats import multivariate_normal
# from state import x_bins,y_bins,theta_bins,grab_bins

class MultiVariateGaussian(object):

    def __init__(self, env_name_list, segragated):
        self.env_name_list = env_name_list
        print(self.env_name_list)
        self.segragated = segragated
        self.file_name = "critical_region_"+env_name_list[0]+"_to_"+env_name_list[-1]+"_data.p"
        self.data = self.load_data()
        self.fit_dic = {}

    def load_data(self):
        print("loading {} data".format(self.env_name_list))
        data_load = pickle.load(open(Config.DATA_MISC_DIR+"critical_region_relative_occupancy_map_data.p"))
        import IPython;IPython.embed()
        
        return data_load

    def extract_data(self,arr):
        shape = np.shape(arr)
        # index_list = []

        # for a in range(shape[0]):
        #     for b in range(shape[1]):
        #         for c in range(shape[2]):
        #             if len(shape)>3:
        #                 for d in range(shape[3]):
        #                     if arr[a,b,c,d]:
        #                         index_list.append([a,b,c,d])
                        
        #             else:
        #                 if arr[a,b,c]:
        #                     index_list.append([a,b,c])

        index_list = np.argwhere(arr==1.0)        
        return np.array(index_list,dtype='float64').transpose()
    
    def fit(self,arr):
        dist_param_dic = {}

        dist_param_dic["mean"] = np.mean(arr, axis = 1)
        dist_param_dic["cov_mat"] = np.cov(arr)
    
        return dist_param_dic

    def process_rcr(self):
        for prim_obj_name in self.data.keys():
            dic = self.data[prim_obj_name]
            self.fit_dic[prim_obj_name] = {}

            if self.segragated:
                key1 = "rcr_dic"
                key2 = "grabbed"
                key3 = "not_grabbed"
            else:
                key1 = "binarized"

            for sec_obj_name in dic.keys():
                rcr_list = dic[sec_obj_name][key1]
                            
                if type(rcr_list) == dict:
                    sec_obj_param_dic = {}
                    sec_obj_param_dic[key2] = []
                    sec_obj_param_dic[key3] = []
                    for key in sec_obj_param_dic.keys():
                        l = rcr_list[key]
                        for i in range(len(l)):
                            arr = l[i]
                            arr = self.extract_data(arr)
                            param_dic = self.fit(arr)
                            sec_obj_param_dic[key].append(param_dic)
                else:
                    sec_obj_param_dic = {}
                    arr = self.extract_data(rcr_list)
                    sec_obj_param_dic = self.fit(arr)

                self.fit_dic[prim_obj_name][sec_obj_name] = sec_obj_param_dic                           
    
    def save_samples_files(self):
        rcr_samples_dic = {}
        for prim_obj in self.data.keys():                            
            prim_dic = self.data[prim_obj]
            rcr_samples_dic[prim_obj] = {}        
            for sec_obj in prim_dic.keys():
                rcr_dic = prim_dic[sec_obj]['rcr_dic']
                rcr_samples_dic[prim_obj][sec_obj] = rcr_dic

        for prim_obj in rcr_samples_dic.keys():
            for sec_obj in rcr_samples_dic[prim_obj].keys():
                for grip in rcr_samples_dic[prim_obj][sec_obj].keys():
                    rcr_list = []
                    for arr in rcr_samples_dic[prim_obj][sec_obj][grip]:
                        l = np.argwhere(arr==1.0)
                        rcr_list.append(l)
                    rcr_samples_dic[prim_obj][sec_obj][grip] = rcr_list
        
        print("saving rcr samples data")
        pickle.dump(rcr_samples_dic,open("./rcr_samples_dic.p","wb"))                

    def save(self):
        if self.segragated:
            flag = "segragated_"
        else:
            flag = "combined_"

        print("saving multi-variate parameter {} data".format(self.env_name_list))
        pickle.dump(self.fit_dic,open(Config.DATA_MISC_DIR+flag+"mvg_param_"+self.env_name_list[0]+"_to_"+self.env_name_list[-1]+"_data.p" ,"wb"))
        print("mvg params for {} saved".format(self.env_name_list))
    
if __name__ == "__main__":
    import IPython
    IPython.embed()
