import numpy as np
import copy

class EnvState(object):
    def __init__(self,obj_dic,keyword_arguments,num_robots=1):
        self.object_dict = obj_dic        
        self.num_robots = num_robots
        for key in keyword_arguments.keys():
            setattr(self,key,keyword_arguments[key])

    def __deepcopy__(self,memodict={}):
        keyword_arguments = {}
        for key in vars(self).keys():
            if key not in ["object_dict","num_robots"]:
                keyword_arguments[key] = copy.deepcopy(vars(self)[key])
        
        new_object_dict = copy.deepcopy(self.object_dict)
        new_env_state = EnvState(new_object_dict,keyword_arguments,self.num_robots)
        return new_env_state