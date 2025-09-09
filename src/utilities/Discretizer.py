import math
import numpy as np

DEFAULT_START = np.array([-0.5,-0.5,-0.5,-math.radians(180),-math.radians(180),-math.radians(180)])
DEFAULT_END = np.array([0.5,0.5,0.5,math.radians(180),math.radians(180),math.radians(180)])
DEFAULT_BIN_COUNT = np.array([224,224,224,11,11,11])

class Discretizer(object):

    def __init__(self,env_start=DEFAULT_START,env_end = DEFAULT_END,world_n_bins=DEFAULT_BIN_COUNT ):
        # self.n_dofs = self.robot.get_num_active_dofs()
        env_ranges = env_end-env_start
        self.env_start = env_start
        self.env_end = env_end
        self.n_dofs = len(env_ranges)
        self.world_n_bins = world_n_bins
        self.env_ranges = env_ranges
        self.relational_n_bins = []
        # self.relational_n_bins.extend(((self.world_n_bins[:3]-2)*2*math.sqrt(2))+2)
        self.relational_n_bins.extend(((self.world_n_bins[:3])*2*math.sqrt(2)))
        self.relational_n_bins.extend(self.world_n_bins[3:])
        for i in range(len(self.relational_n_bins)):
            self.relational_n_bins[i] = int(math.ceil(self.relational_n_bins[i]))

        self.bins = self.__create_bins()

    def __create_bins(self):
        world_bins = {}
        relational_bins = {}
        # llimits = self.robot.get_dof_lower_limits()
        # ulimits = self.robot.get_dof_upper_limits()

        llimits = self.env_start
        ulimits = self.env_end
        dof_range = ulimits - llimits

        relational_llimits = -(ulimits-llimits)*math.sqrt(2)
        relational_ulimits = -relational_llimits
        relational_dof_range = relational_ulimits-relational_llimits

        # for x-dof
        i = 0
        world_bins[i] = {}
        start = llimits[i]
        bin_step = dof_range[i] / (self.world_n_bins[i])
        # bin_step = dof_range[i] / (self.world_n_bins[i]-2)
        # world_bins[i]["bin_start"] = [-np.inf]
        world_bins[i]["bin_start"] = []
        # world_bins[i]["bin_end"] = [start]
        world_bins[i]["bin_end"] = []
        # for j in range((self.world_n_bins[i]-2)):
        for j in range((self.world_n_bins[i])):
            world_bins[i]["bin_start"].append(start)
            world_bins[i]["bin_end"].append(start + bin_step)
            start += bin_step
        # world_bins[i]["bin_start"].append(start)
        # world_bins[i]["bin_end"].append(np.inf)

        relational_bins[i] = {}
        bin_step = relational_dof_range[i] / (self.relational_n_bins[i]-1)
        start = relational_llimits[i]-bin_step/2.0
        relational_bins[i]["bin_start"] = []
        # relational_bins[i]["bin_start"] = [-np.inf]
        relational_bins[i]["bin_end"] = []
        # relational_bins[i]["bin_end"] = [start]
        # for j in range(self.relational_n_bins[i]-2):
        for j in range(self.relational_n_bins[i]):
            relational_bins[i]["bin_start"].append(start)
            relational_bins[i]["bin_end"].append(start + bin_step)
            start += bin_step
        # relational_bins[i]["bin_start"].append(start)
        # relational_bins[i]["bin_end"].append(np.inf)
        
        # for y-dof
        i = 1
        world_bins[i] = {}
        # world_bins[i]["bin_start"] = [np.inf]
        world_bins[i]["bin_start"] = []
        # world_bins[i]["bin_end"] = [start-bin_step]
        world_bins[i]["bin_end"] = []
        start = ulimits[i]
        # bin_step = dof_range[i] / (self.world_n_bins[i]-2)
        bin_step = dof_range[i] / (self.world_n_bins[i])
        # for j in range((self.world_n_bins[i]-2)):
        for j in range((self.world_n_bins[i])):
            world_bins[i]["bin_start"].append(start - bin_step)
            world_bins[i]["bin_end"].append(start)
            start -= bin_step
        # world_bins[i]["bin_start"].append(start)
        # world_bins[i]["bin_end"].append(-np.inf)

        relational_bins[i] = {}
        bin_step = relational_dof_range[i] / (self.relational_n_bins[i]-1)
        start = relational_ulimits[i]+bin_step/2.0
        # relational_bins[i]["bin_start"] = [np.inf]
        relational_bins[i]["bin_start"] = []
        # relational_bins[i]["bin_end"] = [start-bin_step]
        relational_bins[i]["bin_end"] = []
        # for j in range(self.relational_n_bins[i]-2):
        for j in range(self.relational_n_bins[i]):
            relational_bins[i]["bin_start"].append(start - bin_step)
            relational_bins[i]["bin_end"].append(start)
            start -= bin_step
        # relational_bins[i]["bin_start"].append(start)
        # relational_bins[i]["bin_end"].append(-np.inf)
    
        #for z-dof
        i = 2
        world_bins[i] = {}
        start = llimits[i]
        # bin_step = dof_range[i] / (self.world_n_bins[i]-2)
        bin_step = dof_range[i] / (self.world_n_bins[i])
        world_bins[i]["bin_start"] = []
        # world_bins[i]["bin_start"] = [-np.inf]
        world_bins[i]["bin_end"] = []
        # world_bins[i]["bin_end"] = [start]
        # for j in range((self.world_n_bins[i]-2)):
        for j in range((self.world_n_bins[i])):
            world_bins[i]["bin_start"].append(start)
            world_bins[i]["bin_end"].append(start + bin_step)
            start += bin_step
        world_bins[i]["bin_start"].append(start)
        world_bins[i]["bin_end"].append(np.inf)

        relational_bins[i] = {}
        # relational_bins[i]["bin_start"] = [-np.inf]
        # relational_bins[i]["bin_end"] = [start]
        bin_step = relational_dof_range[i] / (self.relational_n_bins[i]-1)
        start = relational_llimits[i]-bin_step/2.0
        relational_bins[i]["bin_start"] = []
        relational_bins[i]["bin_end"] = []
        # for j in range(self.relational_n_bins[i]-2):
        for j in range(self.relational_n_bins[i]):
            relational_bins[i]["bin_start"].append(start)
            relational_bins[i]["bin_end"].append(start + bin_step)
            start += bin_step
        # relational_bins[i]["bin_start"].append(start)
        # relational_bins[i]["bin_end"].append(np.inf)

        #for other dofs
        for i in range(3,len(self.relational_n_bins)):
            world_bins[i] = {}            
            # bin_step = dof_range[i] / (self.relational_n_bins[i]-3)
            bin_step = dof_range[i] / (self.relational_n_bins[i]-2)
            start = llimits[i]-bin_step
            # world_bins[i]["bin_start"] = [-np.inf]
            # world_bins[i]["bin_end"] = [start]
            world_bins[i]["bin_start"] = []
            world_bins[i]["bin_end"] = []
            # for j in range(self.relational_n_bins[i]-2):
            for j in range(self.relational_n_bins[i]):
                world_bins[i]['bin_start'].append(start)
                world_bins[i]['bin_end'].append(start + bin_step)
                start += bin_step
            # world_bins[i]["bin_start"].append(start)
            # world_bins[i]["bin_end"].append(np.inf)

            relational_bins[i] = world_bins[i]

        bins = {"absolute":world_bins, "relative":relational_bins}      
        return bins

    def get_bins(self,is_relative):
        if is_relative:
            return self.bins["relative"]
        else:
            return self.bins["absolute"]
        
    def convert_sample(self,dof_sample,is_relative):
        dof_values = []
        bins = self.get_bins(is_relative)

        for i in range(len(dof_sample)):
            mean = (bins[i]['bin_start'][dof_sample[i]] + bins[i]['bin_end'][dof_sample[i]]) / 2.0
            std_dev = abs((bins[i]['bin_end'][dof_sample[i]]-bins[i]['bin_start'][dof_sample[i]]) / 5.0)
            dof_value = np.inf
            while not (dof_value < bins[i]['bin_end'][dof_sample[i]] and dof_value > bins[i]['bin_start'][dof_sample[i]]):
                dof_value = np.random.normal(loc=mean, scale=std_dev)

            dof_values.append(dof_value)

        return dof_values

    def get_bin_from_ll(self, dofval, jointIdx, is_relative):        
        # bins = self.bins[jointIdx]
        bins = self.get_bins(is_relative)[jointIdx]
        if jointIdx>2:
            dofval = round(round(dofval/0.05)*0.05,2)

        if jointIdx == 1:
            # if dofval > bins['bin_start'][1]:
            #     return 0
            # if dofval < bins['bin_end'][-2]:
            #     return len(bins['bin_end'])-1

            if dofval > bins['bin_end'][0] or dofval < bins['bin_start'][-1]:
                return -1
        else:
            # if dofval < bins['bin_start'][1]:
            #     return 0
            # if dofval > bins['bin_end'][-2]:
            #     return len(bins['bin_end'])-1
       
            if dofval < bins['bin_start'][0] or dofval > bins['bin_end'][-1]:
                # if jointIdx > 2:
                #     print(dofval)
                return -1
            
        for j in range(len(bins['bin_start'])):
            if bins['bin_start'][j] <= dofval <= bins['bin_end'][j]:
                return j
            
    def get_discretized_pose(self,input_pose,is_relative):
        pose = []
        for jointIdx in range(len(input_pose)):
            bins = self.get_bins(is_relative)[jointIdx]
            dofval = input_pose[jointIdx]
            if jointIdx>2:
                dofval = round(round(dofval/0.05)*0.05,2)
                
            if jointIdx == 1:
                # if dofval > bins['bin_start'][1]:
                #     pose.append(0)
                #     continue
                # if dofval < bins['bin_end'][-2]:
                #     pose.append(len(bins['bin_end'])-1)
                #     continue

                if dofval > bins['bin_end'][0] or dofval < bins['bin_start'][-1]:
                    pose.append(-1)
                    continue
            else:
                # if dofval < bins['bin_start'][1]:
                #     pose.append(0)
                #     continue
                # if dofval > bins['bin_end'][-2]:
                #     pose.append(len(bins['bin_end'])-1)
                #     continue
           
                if dofval < bins['bin_start'][0] or dofval > bins['bin_end'][-1]:
                    pose.append(-1)
                    continue
                
            for j in range(len(bins['bin_start'])):
                if bins['bin_start'][j] <= dofval <= bins['bin_end'][j]:
                    pose.append(j)
                    break

        return pose
