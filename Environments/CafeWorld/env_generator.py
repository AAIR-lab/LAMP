import object_models
import utils
import numpy as np
import json
from openravepy import *
import argparse
from openravepy.misc import DrawAxes
import os
from copy import deepcopy

rot_X = matrixFromAxisAngle([-np.pi/2, 0, 0])
rot_Y = matrixFromAxisAngle([0,-np.pi/2, 0])
rot_Z = matrixFromAxisAngle([0, 0, -np.pi/2])

#TODO: update this offset
robot_offset = 0.075
TABLE_NAMES = {"rll": "rll_table", "small": "table6"}

class environment():    
    def __init__(self,env_name=None,interactive=False):
        self.env = Environment()
        # self.env.Load('empty_env.dae')
        if env_name is not None:
            if interactive:
                self.env.Load('/workspaces/LAMP/Environments/CafeWorld/reference_structure/{}.dae'.format(env_name[0]))
            else:
                self.env.Load('plank_relation_structures/{}.dae'.format(env_name[0]))

            self.env_name = env_name[0]

        self.env.SetViewer('qtcoin')        
        self.collision = False
        self.can_list = []
        self.table_list = []
        
        #defining the pickup_region.
        self.gripper_offset = 0.03

        #defining object_limits
        self.object_x_limits = [0.03, 0.05] #[x_min,x_max]  
        self.object_y_limits = [0.03, 0.05] #[y_min,y_max]
        self.object_pose_array = []
        self.object_name_array = []
        self.clearance = 0.01 #clearance from walls of pickup regions.
        self.num_objects = 1
        self.object_count = 0

        self.can_radius=0.05
        self.can_height=0.2
        self.OBJECT_PREFIX = "can"

        self.table_h = 0.63
        self.table_top_thickness = 0.1

        self.countertop_h = 1.15
        self.countertop_range_x = [1.5,2.0]
        self.countertop_range_y = [0.3,1.5]

        self.large_range_x = [-0.5,6]
        self.large_range_y = [-12,-3]

        self.near_countertop_x = [0.5,1.7]
        self.near_countertop_y = [0,2.2]

        if interactive:
            import IPython;IPython.embed()
        else:
            self.save()

    def transform_from_wp(self,dof_vals):
        # print('last@transform_from_wp')
        quat = quatFromAxisAngle(dof_vals[3:])
        pose = []
        pose.extend(quat)
        pose.extend(dof_vals[:3])
        transform = matrixFromPose(pose)
        return transform

    def get_pose_from_transform(self,transform):
        # print('last@get_pose_from_transform')
        pose = poseFromMatrix(transform)
        quat = list(pose[1:4])
        quat.append(pose[0])
        pos = list(pose[4:])
        pos.extend(quat)

        return pos

    def spawn_can(self):
        DIFF = 0.02
        radius = 0.05
        height = 0.2
        color = [0,0.8,1]

        body_name = "can" + "_{}".format(len(self.can_list)+1)

        t = matrixFromPose([1, 0, 0, 0, x, y, z])
        cylinder = object_models.create_cylinder(self.env, body_name, t, [radius, height], color)

        self.can_list.append(cylinder)
        
    def spawn_table(self):
        thickness = 0.1
        legheight = 0.55
        color = [1,0.8,0]
        pose = [3, -5, 0.63]

        name = "table_{}".format(len(self.table_list)+1)        
        table = object_models.create_table(self.env,
                                           name,
                                           0.90, #dim1
                                           0.90, #dim2
                                           thickness,
                                           0.1, #legdim1
                                           0.1, #legdim2
                                           legheight,
                                           pose,
                                           color)
        
        self.env.Add(table)
        self.table_list.append(table)
    
    def save(self,file_num=None,json=False):
        if json:
            object_dict = {}
            for obj in self.env.GetBodies():
                obj_name = obj.GetName()

                if obj_name != "gripper":
                    obj_state = self.get_pose_from_transform(obj.GetTransform())
                else:
                    obj_state = self.robot.GetActiveDOFValues()
                
                obj_base_pose = self.get_pose_from_transform(obj.GetTransform())

                object_dict[obj_name] = [None, obj_state, obj_base_pose]

            if file_num is not None:
                file_name = 'plank_relation_structures/env{}.json'.format(file_num)
            else:
                file_name = 'plank_relation_structures/{}.json'.format(self.env_name)

            with open(file_name,"wb") as f:
                json.dump(object_dict,f)

        else:
            if file_num is not None:
                file_name = '/workspaces/LAMP/Environments/CafeWorld/env/env{}.dae'.format(file_num)
            else:
                file_name = '/workspaces/LAMP/Environments/CafeWorld/env/{}.dae'.format(self.env_name)

            with open(file_name,"wb") as f:
                self.env.Save(f)

        print("saved Environment file")  

    def get_object_dims(self,object_name):
        obj = self.env.GetKinBody(object_name)
        limits = utils.get_object_limits(obj)
        obj_dim = [abs(limits[1]-limits[0])/2.0,abs(limits[3]-limits[2])/2.0,limits[-1]]
        return obj_dim
    
    def get_env_limits(self):
        bounds = self.get_object_dims(self.bound_object_name)
        x = bounds[0]-0.1
        y = bounds[1]-0.1

        return [[-x,x],[-y,y]]
    
    def set_env(self):
        tf_rob = self.robot.GetTransform()
        tf_rob[2,3] = self.obj_h
        self.robot.SetTransform(tf_rob)

        for obj in self.env.GetBodies():
            if obj.GetName().split('_')[0] == self.bound_object_name:
                tf_walls = obj.GetTransform()
                tf_walls[2,3] = 2*self.region_h
                obj.SetTransform(tf_walls)
            
    def remove_planks(self,num_to_remove):
        if type(num_to_remove) == int:
            for obj in self.env.GetBodies()[num_to_remove:]:
                self.plank_list.remove(obj)
                self.env.Remove(obj)
        else:
            for num in num_to_remove:
                self.env.Remove(self.env.GetKinBody("plank{}".format(num)))
                self.plank_list.remove(self.env.GetKinBody("plank{}".format(num)))
    
    def rename_planks(self):
        for i, obj in enumerate(self.env.GetBodies()):
            obj.SetName("plank{}".format(i+1))
        
    def load_plank(self):
        self.env.Load("/workspaces/LAMP/Environments/Keva/objects/keva.dae")
        plank = self.env.GetKinBody('SketchUp')
        plank.SetName('plank{}'.format(len(self.plank_list)+1))
        self.plank_list.append(plank)

if __name__ == "__main__":
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n","--name", help = "name of env", nargs='*')
    argParser.add_argument("-i","--set_interactive", help = "flag to use interactive session", action="store_true")
    args = argParser.parse_args()
    env_name = args.name
    interactive = args.set_interactive

    env = environment(env_name,interactive=interactive)