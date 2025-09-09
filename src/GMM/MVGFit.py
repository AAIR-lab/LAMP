import numpy as np
from Config import Config
import pickle
from copy import deepcopy
from scipy.stats import multivariate_normal
from sklearn.mixture import GaussianMixture

class MultiVariateGaussian(object):
    def __init__(self, gaussian):
        self.gaussian = gaussian
        self.threshold = Config.MVG_THRESHOLD

    @staticmethod
    def fit(arr, n_components=Config.DEFAULT_NUM_COMPONENTS):
        gaussian = GaussianMixture(n_components=n_components)
        gaussian.fit(np.array(arr))    
        mvg = MultiVariateGaussian(gaussian)
        return mvg

    def sample(self): 
        i = 0
        best_prob = 0 
        best_sample = None
        while i < 100: 
            sample,_ = self.gaussian.sample()
            proba = self.gaussian.predict_proba(sample) 
            prob = np.max(proba)
            if prob > self.threshold: 
                return sample
            else:
                if prob > best_prob: 
                    best_prob = prob
                    best_sample = sample
            i += 1 
        return best_sample 
        
    def sample_from_component(self,component):
        i = 0
        best_prob = 0 
        best_sample = None

        while True: 
            sample,_ = self.gaussian.sample()
            proba = self.gaussian.predict_proba(sample)[0]
            max_prob = np.max(proba)
            component_prob = proba[component]

            if max_prob == component_prob:            
                if max_prob > self.threshold: 
                    return sample
                else:
                    if max_prob > best_prob: 
                        best_prob = max_prob
                        best_sample = sample
                i += 1 

            if i == 100:
                break

        return best_sample 
        
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