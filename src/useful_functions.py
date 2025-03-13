from src.data_structures.PDDLState import PDDLState
import sys
import os
import Config
import openravepy
import networkx as nx
import numpy as np
import heapq
import pickle as cPickle
from openravepy import *
from tf.transformations import *
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
import subprocess

def start_process(command_list,file_name): 
    p = subprocess.Popen(command_list, stdout = open("{}.log".format(file_name),"w"), stderr = open("{}.err".format(file_name),"w"))
    return p 

def blockPrinting(func):
    def func_wrapper(*args, **kwargs):
        # block all printing to the console
        sys.stdout = open(os.devnull, 'w')
        # call the method in question
        value = func(*args, **kwargs)
        # enable all printing to the console
        sys.stdout = sys.__stdout__
        # pass the return value of the method back
        return value

    return func_wrapper

def get_abstract_state(env_state,object_dictionary, lifted_relations_dictionary, aux_list):
    return PDDLState.get_from_ll(lifted_relations_dict = lifted_relations_dictionary,
                                 object_dict = object_dictionary,
                                 ll_state = env_state,
                                 aux_list = aux_list)

def print_set(set):
    for i in sorted(list(set)):
        print(i)

def get_euclidean_distance(transform1, transform2):
    pos1 = np.array([transform1[:3,3]])
    pos2 = np.array([transform2[:3,3]])

    return np.linalg.norm(pos1-pos2)

def transform_from_pose(dof_vals):
    quat = quatFromAxisAngle(dof_vals[3:])
    pose = []
    pose.extend(quat)
    pose.extend(dof_vals[:3])
    transform = matrixFromPose(pose)
    return transform

def pose_from_transform(transform):
    # print('last@get_pose_from_transform')
    pose = poseFromMatrix(transform)
    quat = pose[:4]
    eul = axisAngleFromQuat(quat)
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

def create_dag(structure,cr_list,discretizer):
    env = openravepy.Environment()
    env.Load(Config.REFERENCE_DIR+structure+".dae")
    unknown_plank_relations = {}

    plank_num = len(env.GetBodies())
    dag = nx.DiGraph()

    root_node = "{}_1".format(Config.OBJECT_NAME[0])
    for p_num in range(1,plank_num+1):
        dag.add_node("{}_{}".format(Config.OBJECT_NAME[0],p_num))

    for p_num in range(1,plank_num+1):
        for p2_num in range(p_num+1,plank_num+1):
            if p_num != p2_num:
                transform1 = env.GetKinBody("{}{}".format(Config.OBJECT_NAME[0],p_num)).GetTransform()
                transform2 = env.GetKinBody("{}{}".format(Config.OBJECT_NAME[0],p2_num)).GetTransform()
                relative_transform = get_relative_transform(transform1,
                                                            transform2)
                relative_pose = pose_from_transform(relative_transform)

                discretized_pose = discretizer.get_discretized_pose(input_pose=relative_pose,
                                                                    is_relative=True)
                euc_dist = get_euclidean_distance(transform1,transform2)
                discretized_pose.append(0)
                known_flag = False
                for region in cr_list:
                    if discretized_pose in region:
                        cost = Config.KNOWN_COST
                        known_flag = True
                        dag.add_edge("{}_{}".format(Config.OBJECT_NAME[0],p_num),"{}_{}".format(Config.OBJECT_NAME[0],p2_num),region=region,cost=cost)
                        edge = ["{}_{}".format(Config.OBJECT_NAME[0],p_num),"{}_{}".format(Config.OBJECT_NAME[0],p2_num),region,cost]
                        break                                                                                                                  

                if not known_flag and p_num in Config.PLANKS_PROBLEM_ORDER[Config.DOMAIN_NAME][structure][p2_num]:
                    cost = euc_dist*Config.UNKNOWN_COST
                    region = [discretized_pose]                
                    edge = ["{}_{}".format(Config.OBJECT_NAME[0],p_num),"{}_{}".format(Config.OBJECT_NAME[0],p2_num),region,cost]
                    if "{}_{}".format(Config.OBJECT_NAME[0],p2_num) not in unknown_plank_relations.keys():
                        unknown_plank_relations["{}_{}".format(Config.OBJECT_NAME[0],p2_num)] = []

                    unknown_plank_relations["{}_{}".format(Config.OBJECT_NAME[0],p2_num)].append((cost,edge))

    for plank in unknown_plank_relations.keys():
        heapq.heapify(unknown_plank_relations[plank])         

    name=structure
    data_dict = save_dag(dag=dag,unknown_plank_relations=unknown_plank_relations,name=name)
    return data_dict
    
def save_dag(dag,unknown_plank_relations,name):
    data_dict = {"dag":dag,
                 "unknown_plank_relations":unknown_plank_relations}
    with open(Config.DAG_DIR+name+"_dag.p","wb") as f:
        nx.nx_pydot.write_dot(dag,f)
        cPickle.dump(data_dict,f,protocol=cPickle.HIGHEST_PROTOCOL)
    
    return data_dict

def check_cycles(graph,intended_edge,root_name):
    search_stack = Stack()
    visited_set = set()
    start_node = Node(name=root_name,parent=None)
    search_stack.push(start_node)
    goal_name = intended_edge[1]
    current_goal_neighbours = get_neighbours(graph = graph, node=intended_edge[1])
    goal_node = None
    p,c,reg,cost = intended_edge
    graph.add_edge(p,c,region=reg,cost=cost)

    while not(search_stack.isEmpty()):
        node = search_stack.pop()

        if node.name==goal_name:
            goal_node = node
            break
        
        if node not in visited_set:
            visited_set.add(node)
            successors = graph.edges(node.name)
            parent=node
            for _,successor_name in successors:
                succesor_node = Node(name=successor_name,parent=parent)
                search_stack.push(succesor_node)

    if goal_node is None:
        path=[]

    else:
        path = []
        current_node = goal_node
        while current_node.parent is not None:
            path.append(current_node.name)
            current_node = current_node.parent
            
        path.reverse()
    
    intersection = visited_set.intersection(current_goal_neighbours)
    
    flag = False
    if len(intersection) > 0 or len(path)==0:
        flag = True
    
    graph.remove_edge(p,c)
    return flag

def get_neighbours(graph,node):
    neighbours = set([])
    for edge in graph.edges:
        if node == edge[0]:
            neighbours.add(edge[1])
        elif node == edge[1]:
            neighbours.add(edge[0])
    
    return neighbours

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

class Node(object):
    def __init__(self,name,parent):
        self.name = name
        self.parent = parent
    
class Stack:
    "A container with a last-in-first-out (LIFO) queuing policy."
    def __init__(self):
        self.list = []

    def push(self,item):
        "Push 'item' onto the stack"
        self.list.append(item)

    def pop(self):
        "Pop the most recently pushed item from the stack"
        return self.list.pop()

    def isEmpty(self):
        "Returns true if the stack is empty"
        return len(self.list) == 0