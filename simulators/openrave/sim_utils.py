import os
import sys
import numpy as np
import math
import time

def get_parent_with_file(file_name):
    current_dir = os.getcwd()
    while True:
        if file_name in os.listdir(current_dir):
            return current_dir
        else:
            current_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
    
    raise Exception("File Not Found in any Parent")

ROOT_DIR = get_parent_with_file("__init__.py")
if ROOT_DIR is not None and ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

from Simulator import Simulator

sim = Simulator()

def transform_from_pose(dof_vals):
    if len(dof_vals) == 3:
        quat = sim.quatFromAxisAngle([0,0,dof_vals[-1]])
        pos = [dof_vals[0],dof_vals[1],0]
    else:
        quat = sim.quatFromAxisAngle(dof_vals[3:])
        pos = dof_vals[:3]
    pose = []
    pose.extend(quat)
    pose.extend(pos)
    transform = sim.matrixFromPose(pose)
    return transform

def pose_from_transform(transform):
    # print('last@get_pose_from_transform')
    pose = sim.poseFromMatrix(transform)
    quat = pose[:4]
    eul = sim.axisAngleFromQuat(quat)
    dofs = []
    dofs.extend(pose[4:])
    dofs.extend(eul)

    return dofs

def get_relative_pose(pose1, pose2):
    #obj2 w.r.t. obj1
    transform1 = transform_from_pose(pose1)
    transform2 = transform_from_pose(pose2)
    return pose_from_transform((np.linalg.pinv(transform1).dot(transform2)))

def get_relative_transform(transform1, transform2):
    #obj2 w.r.t. obj1
    return (np.linalg.pinv(transform1).dot(transform2))

def get_object_limits(obj):
    """Returns the bounding box of an object.
    Returns: min_x, max_x, min_y, max_y, z
    """

    ab = obj.ComputeAABB()
    max_x = ab.pos()[0] + ab.extents()[0]
    min_x = ab.pos()[0] - ab.extents()[0]

    max_y = ab.pos()[1] + ab.extents()[1]
    min_y = ab.pos()[1] - ab.extents()[1]

    max_z = ab.pos()[2] + ab.extents()[2]
    min_z = ab.pos()[2] - ab.extents()[2]

    return min_x, max_x, min_y, max_y, max_z

def get_object_height(obj):
    ab = obj.ComputeAABB()
    max_z = ab.pos()[2] + ab.extents()[2]
    min_z = ab.pos()[2] - ab.extents()[2]
    return max_z - min_z

def plot_transform(env, T, s=0.1):
    """
    Plots transform T in openrave environment.
    S is the length of the axis markers.
    """
    h = []
    x = T[0:3,0]
    y = T[0:3,1]
    z = T[0:3,2]
    o = T[0:3,3]
    h.append(env.drawlinestrip(points=np.array([o, o+s*x]), linewidth=3.0, colors=np.array([(1,0,0),(1,0,0)])))
    h.append(env.drawlinestrip(points=np.array([o, o+s*y]), linewidth=3.0, colors=np.array(((0,1,0),(0,1,0)))))
    h.append(env.drawlinestrip(points=np.array([o, o+s*z]), linewidth=3.0, colors=np.array(((0,0,1),(0,0,1)))))
    return h