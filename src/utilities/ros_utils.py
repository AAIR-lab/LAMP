import os
import sys
import argparse
import pickle
import cPickle
import numpy as np
import tqdm
import json
from tf.transformations import *
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point

x_pose = []
y_pose = []
z_pose = []

norm = []

def get_parent_with_file(file_name):
    current_dir = os.getcwd()
    while True:
        if file_name in os.listdir(current_dir):
            return current_dir
        else:
            current_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
    
    raise Exception("File Not Found in any Parent")

ROOT_DIR = get_parent_with_file("Config.py")
if ROOT_DIR is not None and ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

from Config import Config
from src.data_structures.EnvState import EnvState
import useful_functions
import rospy
from src.data_structures.ROSAction import ROSAction

Config.set_ros_paths()
sys.path.append(Config.ROS_WS_DIR)

from rcr_integration.msg import EnvState as EnvStateMsg
from rcr_integration.msg import ObjectPose
# from geometry_msgs.msg import PoseStamped, Pose
from common_utils import utils

def get_pose_stamped(header_frame_id, transformation):
    pose_stamped = PoseStamped()
    quat = quaternion_from_matrix(transformation)
    pos = transformation[:3,3]

    pose_stamped.header.frame_id = header_frame_id
    pose_stamped.pose.position.x= pos[0]
    pose_stamped.pose.position.y= pos[1]
    pose_stamped.pose.position.z= pos[2]
    pose_stamped.pose.orientation.x = quat[0]
    pose_stamped.pose.orientation.y = quat[1]
    pose_stamped.pose.orientation.z = quat[2]
    pose_stamped.pose.orientation.w = quat[3]
    return pose_stamped

def get_ros_plan(ll_plan,sim_object,test_structure):
    from rcr_ros.msg import Action, Plan

    ROSPlan = []
    for i, ll_transition in enumerate(ll_plan):
        action_id, robot, target_pose, grab_bool, _, req_grounded_pose,env_state, grabbed_object,static_object, rel,_ = ll_transition
        if target_pose is not None:
            if len(target_pose) == 4:
                robot_name = "freight"
            else:
                robot_name = "gripper"
        
        elif grab_bool is not None:
            robot_name = "gripper"

        Type = None
        SubType = ""
        if robot_name == "freight":
            Type = "BaseNavigation"
            if rel.cr == 0:
                SubType = "skip"
        else:
            if grab_bool is not None:
                if grab_bool:
                    Type = "GripperClose"
                else:
                    Type = "GripperOpen"
            else:
                Type = "Manipulation" 
        
        traj = None
        if target_pose is not None:
            traj = target_pose
        else:
            traj = grab_bool
    
        if traj is not None:
            # env_state = sim_object.execute_refinement(traj=traj,robot=robot,obj_name=grabbed_object,lock=False,move_gripper=True)
            sim_object.set_env_state(env_state)
        
        gripper_poses = []
        base_poses = []
        if static_object.split("_")[Config.OBJ_TYPE_IND] in Config.ROBOT_TYPES:
            static_object_pose = env_state.object_dict[static_object][1]
        else:
            static_object_pose = env_state.object_dict[static_object]
        
        is_ik = False
        desired_ik = []
        if (robot_name == "gripper" and rel.cr == 0):
            is_ik = True
            # is_ik = False
            # desired_ik = env_state.object_dict["gripper_{}".format(n+1)][0]
            desired_ik = sim_object.grabbed_armTuckDOFs
            # sim_object.robot.SetActiveDOFValues(sim_object.grabbed_armTuckDOFs)
            # static_object = "yumi_body"
            # static_object_pose = sim_object.get_pose_from_transform(sim_object.robot.GetLink("yumi_body").GetTransform())
        
        elif (rel.parameter1_type in Config.ROBOT_TYPES and rel.parameter2_type in Config.ROBOT_TYPES and rel.cr == 1):
            is_ik = True
            # is_ik = False
            desired_ik = sim_object.grabbed_armTuckDOFs
            # sim_object.robot.SetActiveDOFValues(sim_object.grabbed_armTuckDOFs)
            # static_object = "yumi_body"
            # static_object_pose = sim_object.get_pose_from_transform(sim_object.robot.GetLink("yumi_body").GetTransform())

        for n in range(env_state.num_robots):
            # ik_link_pose = sim_object.get_pose_from_transform(sim_object.ik_link.GetTransform())
            # gripper_poses.append(sim_object.get_relative_pose(static_object_pose,ik_link_pose))
            # gripper_poses.append(env_state.object_dict["gripper_{}".format(n+1)][1])
            gripper_poses.append(sim_object.get_relative_pose(static_object_pose,env_state.object_dict["gripper_{}".format(n+1)][1]))
            
            if Config.DOMAIN_NAME == "CafeWorld" or Config.DOMAIN_NAME == "DinnerTable":
                # base_poses.append(env_state.object_dict["freight_{}".format(n+1)][1])
                base_poses.append(sim_object.get_relative_pose(static_object_pose,env_state.object_dict["freight_{}".format(n+1)][1]))
        
        act = ROSAction(id=action_id,
                        robot=robot_name,
                        gripper_pose=gripper_poses,
                        base_pose=base_poses,
                        Type=Type,
                        SubType=SubType,
                        grabbed_object = grabbed_object,
                        base_frame=static_object,
                        is_ik=is_ik,
                        desired_ik=desired_ik)

        ROSPlan.append(act)

    # for i,current_action in enumerate(ROSPlan):
    #     prev_action = None
    #     next_action = None

    #     if i > 0:
    #         prev_action = ROSPlan[i-1]
    #     if i < len(ROSPlan)-1:
    #         next_action = ROSPlan[i+1]
        
    #     if next_action is not None:
    #         if next_action.Type == "GripperClose":
    #             current_action.SubType = "PreGrasp"
    #         elif next_action.Type == "GripperOpen":
    #             current_action.SubType = "PreDrop"

    #     if prev_action is not None:
    #         if prev_action.Type == "GripperOpen":
    #             current_action.SubType = "PostDrop"
    #         elif prev_action.Type == "GripperClose":
    #             current_action.SubType = "PostGrasp"               

        # current_action.SubType = SubType

    ROSMsgPlan = []
    for i,act in enumerate(ROSPlan):
        action_msg_object = Action()
        action_msg_object.id = act.id
        action_msg_object.type = act.Type
        action_msg_object.subtype = act.SubType
        action_msg_object.grasped_object = act.grabbed_object
        action_msg_object.base_frame = act.base_frame
        action_msg_object.desired_ik = act.desired_ik
        action_msg_object.is_ik = act.is_ik
        
        gripper_pose_t = transform_from_pose(act.gripper_pose[0])
        if len(act.base_pose)>0:
            base_pose_t = transform_from_pose(act.base_pose[0])
        else:
            base_pose_t = np.ones((4,4))

        action_msg_object.target_gripper_pose = get_pose_stamped(act.base_frame,gripper_pose_t)
        action_msg_object.target_base_pose = get_pose_stamped(act.base_frame,base_pose_t)
        
        ROSMsgPlan.append(action_msg_object)

        # (or (pg p3 goalConst) (not (= ?plank  plank4 )))        
    plan = Plan()
    plan.plan = ROSMsgPlan 
    
    get_pose_stamped_for_goals(test_structure+"_structure",sim_object)
    with open(Config.ROS_WS_DIR+"ROSPlan_{}_{}.p".format(Config.DOMAIN_NAME,test_structure),"wb") as f:
        cPickle.dump(plan,f,protocol=cPickle.HIGHEST_PROTOCOL)
        f.close()          

    print("ROSPlan saved")

def get_three_dof_pose(transform):
    p = useful_functions.pose_from_transform(transform)
    return [p[0],p[1],p[-1]]

def get_env_state_from_msg(env_msg):
    obj_dict = {}
    for obj in env_msg.poses:
        obj_name = obj.name
        obj_type = obj_name.split("_")[Config.OBJ_TYPE_IND]

        t = utils.get_transform_from_pose(obj.pose.pose)
        if obj_type not in Config.ROBOT_TYPES.keys() + ["human"]:
            req_val = useful_functions.pose_from_transform(t)
        else:
            req_val = [useful_functions.pose_from_transform(t),useful_functions.pose_from_transform(t)]
            if obj_type == "freight":
                dof_values = get_three_dof_pose(t)
                t = np.eye(4)
                t[:3,3] = [dof_values[0],dof_values[1],0]

                rot_z = np.eye(4)
                rot_z[:3,:3] = rotationMatrixFromAxisAngle([0,0,dof_values[-1]])
                
                req_val = [dof_values,useful_functions.pose_from_transform(t.dot(rot_z))]
        
        obj_dict[obj_name] = req_val

    grab_arguments = {
        "grabbed_flag_1": env_msg.grabbed_flag,
        "grabbed_object_1": env_msg.grabbed_object,
    }
    
    env_state = EnvState(
        obj_dic=obj_dict,
        keyword_arguments=grab_arguments
    )

    return env_state, obj_dict.keys()

def get_pose_stamped_for_goals(structure_name,sim_object):
    goal_dict = {}
    drop_t = sim_object.env.GetKinBody("droparea").GetTransform()
    for goal in sim_object.goalLoc_list:
        t = goal.GetTransform()
        relative_t = get_relative_transform(drop_t,t)
        pose_stamped = get_pose_stamped("droparea",relative_t)
        goal_dict[str(goal.GetName())] = pose_stamped

    with open(Config.ROS_WS_DIR+"{}.p".format(structure_name),"wb") as f:
        cPickle.dump(goal_dict,f,protocol=cPickle.HIGHEST_PROTOCOL)
        f.close()

def post_process_traj(traj,obj_list):
    new_traj = []
    grab_check_dict = dict.fromkeys([obj for obj in obj_list if obj.split("_")[Config.OBJ_TYPE_IND] in Config.OBJECT_NAME], False)
    loc_dict = dict.fromkeys([obj for obj in obj_list if obj.split("_")[Config.OBJ_TYPE_IND] in Config.OBJECT_NAME], [])
    
    first_grasp = dict.fromkeys([obj for obj in obj_list if obj.split("_")[Config.OBJ_TYPE_IND] in Config.OBJECT_NAME])
    last_grasp = dict.fromkeys([obj for obj in obj_list if obj.split("_")[Config.OBJ_TYPE_IND] in Config.OBJECT_NAME])
    grasp_state_count = 0

    for i in range(len(traj)-1):
        if i == 0:
            for obj in loc_dict.keys():
                loc_dict[obj].append(traj[i].object_dict[obj])
            
        cur_state = traj[i]
        next_state = traj[i+1]

        cur_state.grabbed_object_1, valid = get_grabbed_object_in_transition(cur_state,next_state,grab_check_dict)
        if not valid:
            return None, None, False
        # if cur_state.grabbed_flag_1:
        #     assert(cur_state.grabbed_object_1 is not None)
            
        new_traj.append(cur_state)
        
        if cur_state.grabbed_flag_1:
            grasp_state_count += 1
            if cur_state.grabbed_object_1 is not None and first_grasp[cur_state.grabbed_object_1] is None:
                first_grasp[cur_state.grabbed_object_1] = i

            last_grasp[cur_state.grabbed_object_1] = i
    
    if next_state.grabbed_flag_1:
        grasp_state_count += 1
        next_state.grabbed_object_1 = cur_state.grabbed_object_1
        # assert(cur_state.grabbed_object_1 is not None)
        last_grasp[next_state.grabbed_object_1] = len(traj) - 1
    
    new_traj.append(next_state)

    for obj in loc_dict.keys():
        loc_dict[obj].append(traj[last_grasp[obj]].object_dict[obj])
    
    new_traj = add_location_poses_to_traj(loc_dict,new_traj)
    new_object_list = new_traj[0].object_dict.keys()
    
    #assuming only one object can be grabbed at the same time
    clean_bool = sum([lg - fg + 1 for fg,lg in [(first_grasp[obj],last_grasp[obj]) for obj in grab_check_dict.keys()]]) == grasp_state_count 
    if clean_bool:
        for t in new_traj:
            if t.grabbed_flag_1:
                obj_pose = t.object_dict[t.grabbed_object_1]
                gripper_pose = t.object_dict["gripper_1"][1]
                x,y,z,_,_,_ = useful_functions.get_relative_pose(gripper_pose,obj_pose)
                x_pose.append(abs(x))
                y_pose.append(abs(y))
                z_pose.append(abs(z))
        # base_pose = new_traj[last_grasp["glass_1"]].object_dict["freight_1"][1]
        # obj_pose = new_traj[last_grasp["glass_1"]].object_dict["glass_1"]
        
        # print(useful_functions.get_relative_pose(base_pose,obj_pose))

    return new_traj, new_object_list, clean_bool

def get_grabbed_object_in_transition(prev_state, next_state,grab_check_dict={}):
    if not prev_state.grabbed_flag_1:
        return None, True
    
    if len(grab_check_dict.values()) == 0:
        grab_check_dict = dict.fromkeys([obj for obj in prev_state.object_dict.keys() if obj.split("_")[Config.OBJ_TYPE_IND] in Config.OBJECT_NAME], False)

    for obj in grab_check_dict.keys():
        grab_check_dict[obj] = np.allclose(
            useful_functions.get_relative_pose(prev_state.object_dict["gripper_1"][1],prev_state.object_dict[obj])[:3],
            useful_functions.get_relative_pose(next_state.object_dict["gripper_1"][1],next_state.object_dict[obj])[:3],
            atol=1e-2
        )
    
    # assert(any(grab_check_dict.values()))
    if not any(grab_check_dict.values()):
        return None, False        

    for obj in grab_check_dict.keys():
        if grab_check_dict[obj]:
            return obj, True

def add_location_poses_to_traj(loc_dict, traj):
    for s in traj:
        for name in loc_dict.keys():
            initLoc = "loc_init_{}".format(name.split("_")[Config.OBJ_ID_IND])
            targetLoc = "loc_target_{}".format(name.split("_")[Config.OBJ_ID_IND])

            s.object_dict[initLoc] = loc_dict[name][0]
            s.object_dict[targetLoc] = loc_dict[name][1]

    return traj

def check_continuous_grasp(traj):
    first_grasp = -1
    last_grasp = -1

    for i,t in enumerate(traj):
        if t.grabbed_flag_1:
            if first_grasp < 0:
                first_grasp = i
            last_grasp = i
    
    return (last_grasp - first_grasp + 1) == len([t for t in traj if t.grabbed_flag_1])

def to_json(ros_traj, traj_name):
    json_traj = []
    for wp in ros_traj:
        if wp is None:
            continue
        state,obj_list = get_env_state_from_msg(wp.values()[0])

        for obj in obj_list:
            if obj.split("_")[Config.OBJ_TYPE_IND] in Config.ROBOT_TYPES.keys() + ["human"]: 
                state.object_dict[obj] = state.object_dict[obj][1]
        
        json_traj.append(state.object_dict)

    dump_object = json.dumps(json_traj,indent=2)
    with open(Config.TEST_DIR + traj_name + ".json","w") as f:
        f.write(dump_object)
        f.close()

def get_env_state_trajectory_from_msg(ros_traj,object_list=[]):
    traj = []
    for wp in ros_traj:
        if wp is None:
            continue
        
        env_state,obj_list = get_env_state_from_msg(wp.values()[0])
        if object_list == []:
            object_list = obj_list
            
        traj.append(env_state)
    
    return traj,obj_list

if __name__ == "__main__":
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n","--name",  help = "name of env", nargs="*")
    argParser.add_argument("-r","--robot",  help = "name of robot", )
    argParser.add_argument("-d","--domain", help = "name of domain",)

    args, unknown_args = argParser.parse_known_args()
    domain = args.domain
    robot = args.robot
    env_list = args.name
    Config.declare_config(domain,robot)

    for env_name in env_list:
        x_pose = []
        y_pose = []
        z_pose = []
        norm = []

        object_list = []
        data = []
        
        new_object_list = None
        discontinuous_grab_count = 0
        sensing_gap_count = 0

        PATH = Config.ROS_TRAJ_PATHS+Config.DOMAIN_NAME+"/{}_saved_trajectories/".format(env_name)
        trajectories = [ f for f in os.listdir(PATH) if f[0] != "."]
        for traj_file in tqdm.tqdm(trajectories):
            traj_points = pickle.load(open(PATH+traj_file))
            traj,object_list = get_env_state_trajectory_from_msg(traj_points,object_list)
            
            new_traj, latest_object_list, clean_bool = post_process_traj(traj,object_list)

            if latest_object_list is not None:
                new_object_list = latest_object_list

            if clean_bool:
                data.append(new_traj)
            elif new_traj is None:
                sensing_gap_count += 1
            else:
                discontinuous_grab_count += 1
        
        print("clean_ratio: {}/{}".format(len(data),len(trajectories)))
        print("discontinous_grab: {}\nsensing_gap: {}".format(discontinuous_grab_count,sensing_gap_count))
        print("max x: {}".format(max(x_pose)))
        print("max y: {}".format(max(y_pose)))
        print("max z: {}".format(max(z_pose)))
        
        final_data = {"env_states": data,"object_list": list(new_object_list)}
        
        if not os.path.exists(Config.DATA_MISC_DIR+env_name):
            os.makedirs(Config.DATA_MISC_DIR+env_name)

        cPickle.dump(final_data,open(Config.DATA_MISC_DIR+env_name+ "/{}_data.p".format(env_name),"wb"),protocol=cPickle.HIGHEST_PROTOCOL)
        print("File Saved")

        # for traj_file in tqdm.tqdm(trajectories):
        #     traj_points = pickle.load(open(PATH+traj_file))
        #     to_json(traj_points,traj_file.split(".")[0])