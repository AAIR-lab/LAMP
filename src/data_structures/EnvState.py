from openravepy import *
import numpy as np
import copy

class EnvState(object):
    def __init__(self,obj_dic,keyword_arguments,num_robots):
        self.object_dict = obj_dic        
        self.num_robots = num_robots
        for key in keyword_arguments.keys():
            setattr(self,key,keyword_arguments[key])
    
    def transform_from_pose(self,dof_vals):
        if len(dof_vals) == 3:
            quat = quatFromAxisAngle([0,0,dof_vals[-1]])
            pos = [dof_vals[0],dof_vals[1],0]
        else:
            quat = quatFromAxisAngle(dof_vals[3:])
            pos = dof_vals[:3]
        pose = []
        pose.extend(quat)
        pose.extend(pos)
        transform = matrixFromPose(pose)
        return transform

    def pose_from_transform(self,transform):
        # print('last@get_pose_from_transform')
        pose = poseFromMatrix(transform)
        quat = pose[:4]
        eul = axisAngleFromQuat(quat)
        dofs = []
        dofs.extend(pose[4:])
        dofs.extend(eul)

        return dofs
    
    def get_relative_pose(self, pose1, pose2):
        #obj2 w.r.t. obj1
        transform1 = self.transform_from_pose(pose1)
        transform2 = self.transform_from_pose(pose2)
        return self.pose_from_transform((np.linalg.pinv(transform1).dot(transform2)))

    def __deepcopy__(self,memodict={}):
        keyword_arguments = {}
        for key in vars(self).keys():
            if key not in ["object_dict","num_robots"]:
                keyword_arguments[key] = copy.deepcopy(vars(self)[key])
        
        new_object_dict = copy.deepcopy(self.object_dict)
        new_env_state = EnvState(new_object_dict,keyword_arguments,self.num_robots)
        return new_env_state