import object_models
import utils
import numpy as np
import json
from openravepy import *
import argparse
from openravepy.misc import DrawAxes

plank_len_x = 0.118
plank_len_z = 0.0051
rot_X = matrixFromAxisAngle([-np.pi/2, 0, 0])
rot_Y = matrixFromAxisAngle([0,-np.pi/2, 0])
rot_Z = matrixFromAxisAngle([0, 0, -np.pi/2])

robot_offset = 0.075

class environment():    
    def __init__(self,env_name=None,interactive=False):
        self.env = Environment()
        # self.env.Load('empty_env.dae')
        if env_name is not None:
            if interactive:
                # self.env.Load('/workspaces/OLAMP/Environments/Keva/plank_relation_structures/{}.dae'.format(env_name[0]))
                self.env.Load('/workspaces/OLAMP/Environments/Jenga/reference_structure/{}.dae'.format(env_name[0]))
            else:
                self.env.Load('plank_relation_structures/{}.dae'.format(env_name[0]))

            self.env_name = env_name[0]

        self.env.SetViewer('qtcoin')
        self.bound_object_name = ['table6','table60']
        
        self.collision = False
        #self.env_num = 7
        
        #defining the pickup_region.
        self.pickup_a = 0.015
        self.pickup_b = 0.25
        self.color_range = [0.3,0.8]
        self.gripper_offset = 0.03

        #defining object_limits
        self.object_x_limits = [0.03, 0.05] #[x_min,x_max]  
        self.object_y_limits = [0.03, 0.05] #[y_min,y_max]
        self.object_pose_array = []
        self.object_name_array = []
        self.clearance = 0.01 #clearance from walls of pickup regions.
        self.num_objects = 1
        self.object_count = 0

        #defining drop_area parameters
        self.region_dim = 0.2
        self.drop_color = [1,0.8,0]
        self.spawn_color = [0,0.8,1]
        self.drop_area_pos = []
        self.plank_list = []
        for obj in self.env.GetBodies():
            if str(obj.GetName())[:5] == "jenga":
                self.plank_list.append(obj)

        #defining obstacle parameters
        self.obstacle_num = 10
        self.obstacle_limits = [0.1,0.3]
        self.obstacle_color = [0.65,0.65,0.65]
        self.obstacle_count = 0
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
                file_name = 'plank_relation_structures/env{}.dae'.format(file_num)
            else:
                file_name = 'plank_relation_structures/{}.dae'.format(self.env_name)

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
                self.env.Remove(obj)
        else:
            for num in num_to_remove:
                self.env.Remove(self.env.GetKinBody("jenga{}".format(num)))
    
    def rename_planks(self):
        for i, obj in enumerate(self.env.GetBodies()):
            obj.SetName("jenga{}".format(i+1))
        
    def get_relative_transform(self, transform1, transform2):
        #obj2 w.r.t. obj1
        return np.linalg.pinv(transform1).dot(transform2)
        
    def load_plank(self):
        self.env.Load("/workspaces/OLAMP/Environments/Jenga/objects/jenga.dae")
        plank = self.env.GetKinBody('SketchUp')
        plank.SetName('plank{}'.format(len(self.plank_list)+1))
        self.plank_list.append(plank)

    def swap_names(self,plank1,plank2):
        plank_1 = self.env.GetKinBody(plank1)
        plank_2 = self.env.GetKinBody(plank2)

        plank_1_name = plank1
        plank_2_name = plank2

        plank_1.SetName("{}_temp".format(plank_1_name))
        plank_2.SetName(plank_1_name)
        plank_1.SetName(plank_2_name)

if __name__ == "__main__":
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n","--name", help = "name of env", nargs='*')
    argParser.add_argument("-i","--set_interactive", help = "flag to use interactive session", action="store_true")
    args = argParser.parse_args()
    env_name = args.name
    interactive = args.set_interactive

    env = environment(env_name,interactive=interactive)