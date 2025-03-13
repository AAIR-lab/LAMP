import Config

class BinnedEnvState(object):
    def __init__(self,relative_object_dic,keyword_arguments):
        self.binned_pose_dict = {}
        for obj1_name in relative_object_dic.keys():
            self.binned_pose_dict[obj1_name] = {}
            for obj2_name in relative_object_dic[obj1_name].keys():
                relative_pose = relative_object_dic[obj1_name][obj2_name]

                obj1_type = obj1_name.split("_")[0]
                obj2_type = obj2_name.split("_")[0]
                discretizer = Config.get_discretizer(obj1_type,obj2_type)
                
                bin_list = []
                for i in range(len(relative_pose)):
                    bin_list.append(discretizer.get_bin_from_ll(relative_pose[i],jointIdx=i,is_relative=True))
                self.binned_pose_dict[obj1_name][obj2_name] = bin_list
                
        for key in keyword_arguments.keys():
            setattr(self,key,keyword_arguments[key])