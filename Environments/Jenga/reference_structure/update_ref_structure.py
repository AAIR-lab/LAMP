from openravepy import *
import numpy as np 
import os 

os.chdir("/home/naman/OLAMP/Environments/KevaBig/reference_structure/")


def spawn_plank(env,plank_name,t):
    # plank_name = 'plank_{}'.format(plank_name)
    env.Load("../objects/keva_big.dae")
    plank = env.GetKinBody('SketchUp')
    plank.SetName(plank_name)

    # plank = object_models.create_plank(self.env,plank_name)
    
    # t = np.eye(4)
    plank.SetTransform(t)
    return 1

struct_name = "2d_house.dae"
env = Environment() 
env.SetViewer('qtcoin')
env.Load("../../Keva/reference_structure/{}".format(struct_name))
# env.Load("pi_tower.dae")

x_factor = 0.0762 / 0.05884
y_factor = 0.0254 / 0.01162
z_factor = 0.01524 / 0.003875


old_planks = { }
old_planks_transform = {}

for p in env.GetBodies():
    num = int(p.GetName().split("plank")[-1])
    old_planks[num] = p
    old_planks_transform[num] = p.GetTransform() 
    env.Remove(p)

t = np.eye(4)
t[2,3] = -1
new_planks = {} 

for i in range(1,len(old_planks.keys())+1): 
    spawn_plank(env,"new_plank{}".format(i),t)
    new_planks[i] = env.GetKinBody("new_plank{}".format(i))

def get_relative_pose(p1,p2): 
    return np.linalg.pinv(p1).dot(p2)

def get_reverse_pose(p1,p12): 
    return p1.dot(p12)


done_planks = [1,5]
new_planks[1].SetTransform(old_planks_transform[1])

# z_factor = 1

import IPython
IPython.embed()

    

env.Save(struct_name)



