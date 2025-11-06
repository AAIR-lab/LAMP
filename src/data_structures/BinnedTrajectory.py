from Config import Config

class BinnedTrajectory(object):
    def __init__(self, relative_tensor_dict, keyword_argument_lists, traj_distribution_list, object_id_mapping):
        self.relative_tensor_dict = relative_tensor_dict
        self.traj_distribution_list = traj_distribution_list
        for key in keyword_arguments.keys():
            setattr(self,key,keyword_arguments[key])