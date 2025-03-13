import os
import numpy as np
from src.data_structures.discretizer import Discretizer
import math

grab_bins = 3
def get_relative_bin_arr_size(discretizer):
    relative_bin_arr_size = discretizer.relational_n_bins[:3]
    # relative_bin_arr_size.append(sum(np.asarray(discretizer.relational_n_bins[3:])-2)+1)
    relative_bin_arr_size.append(sum(np.asarray(discretizer.relational_n_bins[3:]))+1)
    relative_bin_arr_size.append(grab_bins)

    return relative_bin_arr_size

def get_non_aux_object_set():
    return DOMAIN_NON_AUX_OBJECTS_SET[DOMAIN_NAME]

def get_thresholding_value(obj1,obj2):
    global DOMAIN_NAME
    key_set = frozenset([obj1,obj2])
    return DOMAIN_THRESHOLDING_VALUES[DOMAIN_NAME][key_set]

def declare_config(domain_name,robot_name=None):
    global DOMAIN_NAME
    global BIN_COUNT
    global ROOT_DIR
    global LOCATION_NAME
    global K_PLANS
    global SURFACE_NAME
    global PRE_GRAB_CR
    global OBJECT_NAME
    global ROBOT_NAME
    global GRIPPER_NAME
    global BASE_NAME
    global ROBOT_TYPES
    global DUMMY_TYPES
    global MAX_POSES
    global NUM_GRASPS
    global TABLE_SAMPLE_COUNT
    global ROS_WS_DIR
    global ALT_PLAN_DIR
    global ROS_WS_DIR
    global COP_DIR
    global COP_DOMAIN_DIR
    global REAL_EXPERIMENT_FILE
    global URDF_DIR
    global ROB_DIR
    global BASE_ENV_DIR
    global ENV_DIR
    global CAMERA_DIR
    global DATA_DIR
    global IMAGE_DIR
    global MODEL_DIR
    global OBJECTS_DIR
    global PLANK_PAIRS_DIR
    global REFERENCE_DIR
    global DAG_DIR
    global PDDL_DIR
    global PLANNERS_DIR
    global FD_FILE
    global FF_FILE
    global KP_FILE
    global FD_SOLUTION_NAME
    global PLANNER_SOLUTION_DIR
    global FD_SOLUTION_DIR
    global FD_SOLUTION_FILE
    global KP_SOLUTION_NAME
    global KP_SOLUTION_DIR
    global KP_SOLUTION_FILE
    global FF_SOLUTION_DIR
    global FF_SOLUTION_FILE
    global STD_OUT_DIR
    global SOLVED_PLAN_FILES
    global PLANNER_TIME_OUT
    global DOMAIN_THRESHOLDING_VALUES
    global DOMAIN_BOUND_OBJECT_NAME
    global DOMAIN_NON_AUX_OBJECTS_SET
    global BOUND_OBJECT_NAME
    global SURFACES
    global DOMAIN_TRAJ_RELATION_PAIRS
    global PER_TRAJECTORY_RELATION_PAIR
    global DOMAIN_DISCRETIZER_PARAMS
    global DOMAIN_PROBLEM_AXIS_MAPPING
    global PLANKS_PROBLEM_ORDER
    global PROBLEM_STATES_DIR
    global DATA_MISC_DIR
    global OBJECT_PAIR_ROMS_DIR
    global DATA_TEST_TRAJS
    global DOMAIN_FILE
    global IMMOVABLE_OBJECTS
    global NON_RELATION_OBJECTS
    global AXIS_MAP
    global CONST_TYPES
    global KNOWN_COST
    global UNKNOWN_COST
    global MAX_IK_ATTEMPTS
    global MAX_CALL_COUNTS
    global SAMPLE_COUNT
    global MP_MAX_COUNT
    global MAX_REGION_SAMPLING_COUNT
    global TIMEOUT_THRESHOLD
    global DIFFERENT_ENV_FILES_USED
    global MOVABLE_OBJECTS_DOMAINS
    global MOVABLE_OBJECTS
    global CAMERA_TRANSFORM_FILE
    global DOMAIN_NAME

    DOMAIN_NAME = domain_name

    BIN_COUNT = np.array([64,64,64,60,60,60])
    discretizer = Discretizer(world_n_bins=BIN_COUNT)

    # env_range = discretizer.env_ranges
    # # angle_bins = BIN_COUNT[3]

    # bin_resolutions = []
    # for i in range(len(env_range)):
    #     bin_resolutions.append(env_range[i]/BIN_COUNT[i])

    ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + "/"

    LOCATION_NAME = ["glasstargetLoc","bowltargetLoc","goalLoc","targetLoc","initLoc","glassinitLoc","bowlinitLoc"]
    K_PLANS = 50

    OBJECT_NAME_DICT = {
        "Keva": ["plank"],
        "CafeWorld": ["can"],
        "Packing": ["can"],
        "Jenga": ["jenga"],
        "DinnerTable": ["bowl", "can"]
    }

    ROBOT_DICT = {
        "Keva": "yumi",
        "CafeWorld": "Fetch",
        "Packing": "MagneticGripper",
        "Jenga": "Fetch",
        "DinnerTable": "Fetch",
    }

    SURFACE_NAME = "surface"
    PRE_GRAB_CR = 2
    OBJECT_NAME = OBJECT_NAME_DICT[DOMAIN_NAME]
    if robot_name is None:
        ROBOT_NAME = ROBOT_DICT[DOMAIN_NAME]
    else:
        ROBOT_NAME = robot_name

    GRIPPER_NAME = "gripper"
    BASE_NAME = "freight"

    ROBOT_TYPES = {
        "gripper":"manip_joints", 
        "freight":"base_joints"
    }
    DUMMY_TYPES = ["goalLoc","droparea"]

    MAX_POSES = 5
    NUM_GRASPS = 4
    TABLE_SAMPLE_COUNT = 4

    ROS_WS_DIR = "/workspaces/catkin_ws/src/rcr_ros/plan_files/"
    ALT_PLAN_DIR = ROOT_DIR + "plan_files/"
    #ROS_WS_DIR = ROOT_DIR
    COP_DIR = ROOT_DIR + "base_lines/code_as_policies/"
    COP_DOMAIN_DIR = COP_DIR + DOMAIN_NAME + "/"
    REAL_EXPERIMENT_FILE = ROS_WS_DIR + "objects/env.dae"
    URDF_DIR = ROOT_DIR + "src/robot_files/" + ROBOT_NAME + "/URDF/"
    ROB_DIR = ROOT_DIR + "src/robot_files/"
    BASE_ENV_DIR = ROOT_DIR + "Environments/" + DOMAIN_NAME + "/"
    ENV_DIR = BASE_ENV_DIR + "env/"
    CAMERA_DIR = BASE_ENV_DIR + "camera_angles/"
    DATA_DIR = ROOT_DIR + "Data/" + DOMAIN_NAME + "/"
    IMAGE_DIR = ROOT_DIR + "Images/" + DOMAIN_NAME + "/"
    MODEL_DIR = ROOT_DIR + "Model/" + DOMAIN_NAME + "/"
    OBJECTS_DIR = ROOT_DIR + "Environments/" + DOMAIN_NAME + "/objects/"
    PLANK_PAIRS_DIR = BASE_ENV_DIR + "plank_relation_structures/"
    REFERENCE_DIR = BASE_ENV_DIR + "reference_structure/"
    DAG_DIR = BASE_ENV_DIR + "structure_dags/"
    PDDL_DIR = ROOT_DIR + "Domains/" + DOMAIN_NAME + "/"
    PLANNERS_DIR = ROOT_DIR + "Planners/"
    FD_FILE = PLANNERS_DIR + "FD/fast-downward.py"
    FF_FILE = PLANNERS_DIR + "FF/ff"
    # KP_FILE = PLANNERS_DIR + "symk/fast-downward.py"
    KP_FILE = PLANNERS_DIR + "kstar/fast-downward.py"
    FD_SOLUTION_NAME = "sas_plan"
    PLANNER_SOLUTION_DIR = ROOT_DIR + "plan_solutions/"
    FD_SOLUTION_DIR = PLANNER_SOLUTION_DIR + "fd_planner/"
    FD_SOLUTION_FILE = ROOT_DIR + FD_SOLUTION_DIR
    KP_SOLUTION_NAME = "sas_plan"
    KP_SOLUTION_DIR = ROOT_DIR + "plan_solutions/k_planner/"
    KP_SOLUTION_FILE = ROOT_DIR + KP_SOLUTION_NAME #PLANNERS_DIR + "symk/"
    FF_SOLUTION_DIR = PDDL_DIR
    FF_SOLUTION_FILE = PDDL_DIR
    STD_OUT_DIR = ROOT_DIR + "Data/" + DOMAIN_NAME + "/std_out_logs/"
    SOLVED_PLAN_FILES = ROOT_DIR + "solved_plans/"

    PLANNER_TIME_OUT = 3*3600

    DOMAIN_THRESHOLDING_VALUES = {
        "CafeWorld" : {
                        frozenset(["gripper","freight"])    : 0.2,#1.0,
                        frozenset(["gripper","can"])        : 0.95,#0.0195,
                        frozenset(["freight","surface"])    : 0.22,#0.248
                        frozenset(["can","goalLoc"])        : 0.0011,#0.04,
                        frozenset(["surface","can"])        : 0.4,
                        frozenset(["can"])                  : 1.0,
                        frozenset(["freight","can"])        : 1.0 
                    },
        
        "DinnerTable" : {
                        frozenset(["gripper","freight"])    : 1.0,#1.0,
                        frozenset(["gripper","glass"])        : 1.0,#0.0195,
                        frozenset(["gripper","bowl"])       : 1.0,#0.0195,
                        #   frozenset(["freight","surface"])    : 0.2,#0.248,
                        frozenset(["glass","glasstargetLoc"])        : 1.0,#0.04,
                        frozenset(["glass","bowltargetLoc"])        : 1.0,#0.04,
                        frozenset(["glass","bowlinitLoc"])        : 1.0,#0.04,
                        frozenset(["glass","glassinitLoc"])        : 1.0,#0.04,
                        frozenset(["bowl","bowltargetLoc"])       : 1.0,#0.04,
                        frozenset(["bowl","bowlinitLoc"])       : 1.0,#0.04,
                        frozenset(["bowl","glasstargetLoc"])       : 1.0,#0.04,
                        frozenset(["bowl","glassinitLoc"])        : 1.0,
                        frozenset(["freight","glasstargetLoc"])       : 0.55,#0.04,
                        frozenset(["freight","glassinitLoc"])       : 0.55,#0.04,
                        frozenset(["freight","bowltargetLoc"])       : 0.55,#0.04,
                        frozenset(["freight","bowlinitLoc"])       : 0.550,#0.04,
                        #   frozenset(["surface","glass"])        : 0.2,
                        frozenset(["glass"])                  : 1.0,
                        frozenset(["bowl"])                 : 1.0,
                        frozenset(["glass","bowl"])           : 1.0,
                        frozenset(["freight","glass"])        : 0.5,
                        frozenset(["freight","bowl"])       : 0.5
                    },

        "Keva" : {
                    frozenset(["gripper","plank"]) : 1.0,
                    frozenset(["plank","goalLoc"]) : 1.0,
                    frozenset(["plank","plank"])   : 1.0,
                    },
        
        "Jenga" : {
                    frozenset(["gripper","jenga"]) : 1.0,
                    frozenset(["jenga","goalLoc"]) : 1.0,
                    frozenset(["jenga","jenga"])   : 1.0,
                    },

        "MultiKeva" : {
                        frozenset(["gripper","plank"]) : 1.0,
                        frozenset(["plank","goalLoc"]) : 1.0,
                        frozenset(["plank","plank"])   : 1.0,
                        frozenset(["gripper"])         : 1.0,
                        },

        "Packing"   : {
                        frozenset(["gripper","can"])  : 1.0,
                        frozenset(["can","surface"])  : 0.2,
                        frozenset(["can","can"])      : 1.0,
                    }  
    }

    DOMAIN_BOUND_OBJECT_NAME = {
        "CafeWorld"  : "world_final",
        "DinnerTable": "world_final",
        "Keva"       : "table6",
        "MultiKeva"  : "table6",
        "Packing"    : "table6",
        "Jenga"      : "small_table"
    }

    DOMAIN_NON_AUX_OBJECTS_SET = {
        "CafeWorld"  : frozenset(["surface","goalLoc"]),
        "DinnerTable": frozenset(["surface","glasstargetLoc","bowltargetLoc"]),
        "Keva"       : frozenset([]),
        "MultiKeva"  : frozenset([]),
        "Packing"    : frozenset(["can","surface"]),
        "Jenga"      : frozenset([])
    }

    BOUND_OBJECT_NAME = DOMAIN_BOUND_OBJECT_NAME[DOMAIN_NAME]

    SURFACES = ["countertop","table"]
    DOMAIN_TRAJ_RELATION_PAIRS = {
        "DinnerTable": [set([BASE_NAME,SURFACE_NAME]),set([GRIPPER_NAME,OBJECT_NAME[0]]),set([SURFACE_NAME,OBJECT_NAME[0]]),set([GRIPPER_NAME,OBJECT_NAME[-1]]),set([SURFACE_NAME,OBJECT_NAME[-1]]),set([BASE_NAME,SURFACE_NAME]),set(["bowl","bowltargetLoc"]),set(["bowl","bowlinitLoc"]),set(["glass","glasstargetLoc"]),set(["glass","glassinitLoc"]),set(["freight","bowltargetLoc"]),set(["freight","bowlinitLoc"]),set(["freight","glasstargetLoc"]),set(["freight","glassinitLoc"])],
        "CafeWorld"  : [set([BASE_NAME,SURFACE_NAME]),set([GRIPPER_NAME,OBJECT_NAME[0]]),set([SURFACE_NAME,OBJECT_NAME[0]]),set([BASE_NAME,SURFACE_NAME])],
        "Keva"       : [set([OBJECT_NAME[0],OBJECT_NAME[0]])],
        "MultiKeva"  : [set([OBJECT_NAME[0],OBJECT_NAME[0]]),set([GRIPPER_NAME])],
        "Packing"    : [set()],
        "Jenga"       : [set([OBJECT_NAME[0],OBJECT_NAME[0]])],
    }

    PER_TRAJECTORY_RELATION_PAIR = DOMAIN_TRAJ_RELATION_PAIRS[DOMAIN_NAME]

    DOMAIN_DISCRETIZER_PARAMS = {
        "CafeWorld" : {
                        frozenset(["gripper","freight"])    : (np.array([-0.04,-0.018,-0.21,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                           np.array([0.04,0.018,0.21,math.radians(180),math.radians(180),math.radians(180)]),
                                                           np.array([15,15,15,61,61,10]),
                                                           ),
    
                        frozenset(["gripper","can"])        : (np.array([-0.0825,-0.0825,-0.0825,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.0825,0.0825,0.0825,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([8,8,8,61,61,61]),
                                                            ),

                        frozenset(["freight","can"])        : (np.array([-0.018,-0.046,-0.29,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                           np.array([0.018,0.046,0.29,math.radians(180),math.radians(180),math.radians(180)]),
                                                           np.array([15,15,15,61,61,10]),
                                                           ),                                                           
    
                        frozenset(["freight","surface"])    : (np.array([-0.16,-0.16,-2.25,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.16,0.16,2.25,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([2,2,1,41,41,41]),
                                                            ),

                        frozenset(["can","goalLoc"])        : (np.array([-0.225,-0.225,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.225,0.225,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([4,4,2,11,11,11]),
                                                            ),

                        frozenset(["surface","can"])        : (np.array([-0.114,-0.114,-0.018,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.114,0.114,0.018,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([1,1,1,61,61,61]),
                                                            ),

                        frozenset(["can"])                  : (np.array([-0.5,-0.5,-0.5,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.5,0.5,0.5,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([64,64,64,61,61,61]),
                                                            )
                        },
        
        "DinnerTable" : {
                        frozenset(["gripper","freight"])    : (np.array([-0.2,-0.2,-1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.2,0.2,1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([64,64,64,61,61,61]),
                                                            ),
    
                        frozenset(["gripper","glass"])        : (np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([64,64,64,61,61,61]),
                                                            ),

                        frozenset(["gripper","bowl"])       : (np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([64,64,64,61,61,61]),
                                                            ),

                        frozenset(["freight","glass"])        : (np.array([-0.01,-0.01,-0.0225,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.01,0.01,0.025,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([70,70,2,41,41,41]),
                                                            ),    

                        frozenset(["freight","bowl"])       : (np.array([-0.01,-0.01,-0.0225,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.01,0.01,0.0225,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([70,70,2,41,41,41]),
                                                            ),                                                       
    
                        frozenset(["freight","bowltargetLoc"])    :(np.array([-0.16,-0.16,-0.63,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.16,0.16,0.63,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([2,2,10,41,41,41]),
                                                            ),

                        frozenset(["freight","bowlinitLoc"])    : (np.array([-0.16,-0.16,-0.63,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.16,0.16,0.63,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([2,2,10,41,41,41]),
                                                            ),
                                                            
                        frozenset(["glass","glasstargetLoc"])        : (np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([32,32,32,61,61,61]),
                                                            ),

                        frozenset(["glass","glassinitLoc"])        : (np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([32,32,32,61,61,61]),
                                                            ),

                        frozenset(["glass","bowltargetLoc"])        : (np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([32,32,32,61,61,61]),
                                                            ),

                        frozenset(["glass","bowlinitLoc"])        : (np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([32,32,32,61,61,61]),
                                                            ),

                        frozenset(["bowl","bowltargetLoc"])       :(np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([32,32,32,61,61,61]),
                                                            ),

                        frozenset(["bowl","bowlinitLoc"])       :(np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([32,32,32,61,61,61]),
                                                            ),

                        frozenset(["bowl","glasstargetLoc"])       :(np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([32,32,32,61,61,61]),
                                                            ),

                        frozenset(["bowl","glassinitLoc"])       :(np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([32,32,32,61,61,61]),
                                                            ),

                        frozenset(["freight","glasstargetLoc"])    : (np.array([-0.16,-0.16,-0.63,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.16,0.16,0.63,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([2,2,10,41,41,41]),
                                                            ),

                        frozenset(["freight","glassinitLoc"])    : (np.array([-0.16,-0.16,-0.63,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.16,0.16,0.63,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([2,2,10,41,41,41]),
                                                            ),

                        # frozenset(["surface","can"])        : (np.array([-0.23,-0.23,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                        #                                        np.array([0.23,0.23,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                        #                                        np.array([1,1,2,11,11,11]),
                        #                                        ),

                        frozenset(["glass"])                  : (np.array([-0.5,-0.5,-0.5,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.5,0.5,0.5,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([64,64,64,61,61,61]),
                                                            ),

                        frozenset(["bowl"])                 : (np.array([-0.5,-0.5,-0.5,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.5,0.5,0.5,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([64,64,64,61,61,61]),
                                                            ),

                        frozenset(["glass","bowl"])           : (np.array([-0.5,-0.5,-0.5,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.5,0.5,0.5,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([64,64,64,61,61,61]),
                                                            )

                        },

        "Keva" : {
                    frozenset(["gripper","plank"]) : (np.array([-0.0015,-0.0015,-0.048,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                    np.array([0.0015,0.0015,0.048,math.radians(180),math.radians(180),math.radians(180)]),
                                                    np.array([1,1,10,61,61,61]),
                                                    ),

                    frozenset(["plank","goalLoc"]) : (np.array([-0.005,-0.005,-0.005,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                    np.array([0.005,0.005,0.005,math.radians(180),math.radians(180),math.radians(180)]),
                                                    np.array([6,6,6,61,61,61]),
                                                    ),

                    frozenset(["plank","plank"])   : (np.array([-0.07,-0.07,-0.07,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                    np.array([0.07,0.07,0.07,math.radians(180),math.radians(180),math.radians(180)]),
                                                    np.array([51,51,51,61,61,61]),
                                                    ),

                    },

        "Packing" : {
                        frozenset(["gripper","can"])  : (np.array([-0.15,-0.15,-0.15,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                        np.array([0.15,0.15,0.15,math.radians(180),math.radians(180),math.radians(180)]),
                                                        np.array([35,35,35,61,61,61]),
                                                        ),

                        frozenset(["can","surface"]) :  (np.array([-0.057,-0.057,-0.01,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                        np.array([0.057,0.057,0.01,math.radians(180),math.radians(180),math.radians(180)]),
                                                        np.array([3,3,2,61,61,61]),
                                                        ),
                                                        
                        frozenset(["can","can"])      : (np.array([-0.01,-0.01,-0.01,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                        np.array([0.01,0.01,0.01,math.radians(180),math.radians(180),math.radians(180)]),
                                                        np.array([3,3,3,61,61,61]),
                                                        ),
                        },

        "Jenga" : {
                    frozenset(["gripper","jenga"]) : (np.array([-0.15,-0.15,-0.15,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                    np.array([0.15,0.15,0.15,math.radians(180),math.radians(180),math.radians(180)]),
                                                    np.array([35,35,35,61,61,61]),
                                                    ),

                    frozenset(["jenga","goalLoc"]) : (np.array([-0.01,-0.01,-0.01,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                    np.array([0.01,0.01,0.01,math.radians(180),math.radians(180),math.radians(180)]),
                                                    np.array([3,3,3,61,61,61]),
                                                    ),
                                                    
                    frozenset(["jenga","jenga"])   : (np.array([-0.035,-0.02,-0.035,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                    np.array([0.035,0.02,0.035,math.radians(180),math.radians(180),math.radians(180)]),
                                                    np.array([3,3,3,61,61,61]),
                                                    ),
                    },


    }

    DOMAIN_PROBLEM_AXIS_MAPPING = {
        "Keva": {
            "pi_tower": "x",
            "tripple_pi": "x",
            "flat_tower": "y",
            "flat_3_tower": "y",
            "unseen_1": "x",
            # "unsolvable_1": {
            #     1 : set([]),
            #     2 : set([]),
            #     3 : set([1,2]),
            # },
            "random": "x",
            "2d_house": "y",
            "2_pi":"x",
            # "AAAI": {
            #     1: set([]),
            #     2: set([]),
            #     3: set([]),
            #     4: set([]),
            #     5: set([]),
            #     6: set([]),
            #     7: set([]),
            # }
        },
        "Jenga": {
            "pi_tower": "x",
            "tripple_pi": "x",
            "flat_tower": "y",
            "flat_3_tower": "y",
            "unseen_1": "x",
            # "unsolvable_1": {
            #     1 : set([]),
            #     2 : set([]),
            #     3 : set([1,2]),
            # },
            "random": "x",
            "2d_house": "y",
            "2_pi":"x",
            # "AAAI": {
            #     1: set([]),
            #     2: set([]),
            #     3: set([]),
            #     4: set([]),
            #     5: set([]),
            #     6: set([]),
            #     7: set([]),
            # }
        },
    }

    PLANKS_PROBLEM_ORDER = {
        "Keva": {
            "pi_tower": {
                1: set([]),
                2: set([]),
                3: set([1,2]),
                4: set([]),
                5: set([]),
                6: set([4,5]),
                7: set([3]),
                8: set([6]),
                9: set([7,8]),
            },
            "tripple_pi": {
                1: set([]),
                2: set([]),
                3: set([1,2]),
                4: set([3]),
                5: set([3]),
                6: set([4,5]),
                7: set([6]),
                8: set([6]),
                9: set([7,8]),
            },
            "double_pi": {
                1: set([]),
                2: set([]),
                3: set([1,2]),
                4: set([3]),
                5: set([3]),
                6: set([4,5]),
            },
            "flat_tower": {
                1: set([]),
                2: set([]),
                3: set([1,2]),
                4: set([1,2]),
                5: set([3,4]),
                6: set([3,4]),
                7: set([6,5]),
                8: set([6,5]),
                9 : set([7,8]),
                10: set([7,8]),
                11: set([9,10]),
                12: set([9,10]),
                13: set([11,12]),
                14: set([11,12]),
                15: set([13,14]),
                16: set([13,14]),
                17: set([15,16]),
                18: set([15,16]),
                19: set([17,18]),
                20: set([17,18]),
                21: set([19,20]),
                22: set([19,20]),
                23: set([21,22]),
                24: set([21,22])
            },
            "fort_curricullum": {
                1 : set([]),
                2 : set([]),
                3 : set([]),
                4 : set([]),
                5: set([1,2]),
                6: set([3,4]),
            },
            "flat_3_tower": {
                1 : set([]),
                2 : set([]),
                3 : set([1,2]),
                4 : set([1,2]),
                5 : set([]),
                6 : set([]),
                7 : set([6,5]),
                8 : set([6,5]),
                9 : set([7,8]),
                10: set([3,4]),
                11: set([9,10]),
                12: set([9,10]),
                13: set([11,12]),
                14: set([11,12]),
                15: set([13,14]),
                16: set([13,145]),
            },
            "keva_three_tower": {
                1: set([]),
                2: set([]),
                3: set([1,2]),
                4: set([1,2]),
                5: set([3,4]),
                6: set([3,4]),
                7: set([6,5]),
                8: set([6,5]),
                9 : set([]),
                10: set([]),
                11: set([9,10]),
                12: set([9,10]),
                13: set([11,12]),
                14: set([11,12]),
                15: set([13,14]),
                16: set([13,14]),
                17: set([7,8]),
                18: set([15,16]),
                19: set([17,18]),
                20: set([17,18]),
                21: set([19,20]),
                22: set([19,20]),
                23: set([21,22]),
                24: set([21,22])
            },
            "unseen_1": {
                1 : set([]),
                2 : set([]),
                3 : set([]),
                4 : set([1,2]),
                5 : set([2,3]),
            },
            # "unsolvable_1": {
            #     1 : set([]),
            #     2 : set([]),
            #     3 : set([1,2]),
            # },
            "random": {
                1 : set([]),
                2 : set([]),
                3 : set([]),
                4 : set([]),
                5 : set([]),
                6 : set([]),
                7 : set([]),
                8 : set([]),
                9 : set([]),
            },
            "2d_house": {
                1 : set([]),
                2 : set([]),
                3 : set([]),
                4 : set([]),
                5 : set([]),
                6 : set([])
            },
            "2_pi": {
                1 : set([]),
                2 : set([]),
                3 : set([1,2]),
                4 : set([]),
                5 : set([]),
                6 : set([4,5])
            },
            # "AAAI": {
            #     1: set([]),
            #     2: set([]),
            #     3: set([]),
            #     4: set([]),
            #     5: set([]),
            #     6: set([]),
            #     7: set([]),
            # }
            "stonehenge": {
                1: set([]),
                2: set([]),
                3: set([1,2]),
                4: set([]),
                5: set([]),
                6: set([4,5]),
                7: set([]),
                8: set([]),
                9 : set([7,8]),
                10: set([]),
                11: set([]),
                12: set([11,10]),
                13: set([]),
                14: set([]),
                15: set([13,14]),
                16: set([]),
                17: set([]),
                18: set([17,16]),
            },
            "twin_stacked_pi":{
                1: set([]),
                2: set([]),
                3: set([1,2]),
                4: set([]),
                5: set([]),
                6: set([4,5]),
                7: set([3]),
                8: set([3]),
                9: set([7,8]),
                10: set([6]),
                11: set([6]),
                12: set([10,11]),
                13: set([9]),
                14: set([9]),
                15: set([13,14]),
                16: set([12]),
                17: set([12]),
                18: set([16,17]),
                19: set([15]),
                20: set([15]),
                21: set([19,20]),
                22: set([18]),
                23: set([18]),
                24: set([12,13]),
                25: set([21]),
                26: set([21]),
                27: set([25,26]),
                28: set([24]),
                29: set([24]),
                30: set([28,29]),
                31: set([27]),
                32: set([27]),
                33: set([31,32]),
                34: set([30]),
                35: set([30]),
                36: set([34,35]),
            }
        }, 

        "Jenga": {
            "pi_tower": {
                1: set([]),
                2: set([]),
                3: set([1,2]),
                4: set([]),
                5: set([]),
                6: set([4,5]),
                7: set([3]),
                8: set([6]),
                9: set([7,8]),
            },
            "tripple_pi": {
                1: set([]),
                2: set([]),
                3: set([1,2]),
                4: set([3]),
                5: set([3]),
                6: set([4,5]),
                7: set([6]),
                8: set([6]),
                9: set([7,8]),
            },
            "flat_tower": {
                1: set([]),
                2: set([]),
                3: set([1,2]),
                4: set([1,2]),
                5: set([3,4]),
                6: set([3,4]),
                7: set([6,5]),
                8: set([6,5]),
                9 : set([7,8]),
                10: set([7,8]),
                11: set([9,10]),
                12: set([9,10]),
            },
            "flat_3_tower": {
                1 : set([]),
                2 : set([]),
                3 : set([1,2]),
                4 : set([1,2]),
                5 : set([]),
                6 : set([]),
                7 : set([6,5]),
                8 : set([6,5]),
                9 : set([3,4,7,8]),
                10: set([3,4,7,8]),
                11: set([9,10]),
                12: set([9,10]),
                13: set([11,12]),
                14: set([11,12]),
                15: set([13,14]),
                16: set([13,145]),
            },

            "three_tower_dummy": {
                1 : set([]),
                2 : set([]),
                3 : set([]),
                4 : set([]),
                5 : set([3,4]),
                6 : set([1,2]),
            },

            "three_tower_12": {
                1 : set([]),
                2 : set([]),
                3 : set([]),
                4 : set([]),
                5 : set([1,2]),
                6 : set([1,2]),
                7 : set([3,4]),
                8 : set([3,4]),
                9 : set([7,8]),
                10: set([5,6]),
                11: set([9,10]),
                12: set([9,10]),
            },

            "three_tower": {
                1 : set([]),
                2 : set([]),
                3 : set([]),
                4 : set([]),
                5 : set([1,2]),
                6 : set([1,2]),
                7 : set([3,4]),
                8 : set([3,4]),
                9 : set([5,6]),
                10: set([5,6]),
                11: set([7,8]),
                12: set([7,8]),
                13: set([9,10]),
                14: set([9,10]),
                15: set([11,12]),
                16: set([11,12]),
                17: set([15,16]),
                18: set([13,14]),
                19: set([17,18]),
                20: set([17,18]),
                21: set([19,20]),
                22: set([19,20]),
                23: set([21,22]),
                24: set([21,22])
            },
            "keva_24_plank_tower_structure": {
                1: set([]),
                2: set([]),
                3: set([1,2]),
                4: set([1,2]),
                5: set([3,4]),
                6: set([3,4]),
                7: set([6,5]),
                8: set([6,5]),
                9 : set([7,8]),
                10: set([7,8]),
                11: set([9,10]),
                12: set([9,10]),
                13: set([11,12]),
                14: set([11,12]),
                15: set([13,14]),
                16: set([13,14]),
                17: set([15,16]),
                18: set([15,16]),
                19: set([17,18]),
                20: set([17,18]),
                21: set([19,20]),
                22: set([19,20]),
                23: set([21,22]),
                24: set([21,22])
            },
            "unseen_1": {
                1 : set([]),
                2 : set([]),
                3 : set([]),
                4 : set([1,2]),
                5 : set([2,3]),
            },
            # "unsolvable_1": {
            #     1 : set([]),
            #     2 : set([]),
            #     3 : set([1,2]),
            # },
            "random": {
                1 : set([]),
                2 : set([]),
                3 : set([]),
                4 : set([]),
                5 : set([]),
                6 : set([]),
                7 : set([]),
                8 : set([]),
                9 : set([]),
            },
            "2d_house": {
                1 : set([]),
                2 : set([]),
                3 : set([]),
                4 : set([]),
                5 : set([]),
                6 : set([])
            },
            "2_pi": {
                1 : set([]),
                2 : set([]),
                3 : set([1,2]),
                4 : set([]),
                5 : set([]),
                6 : set([4,5])
            },
            # "AAAI": {
            #     1: set([]),
            #     2: set([]),
            #     3: set([]),
            #     4: set([]),
            #     5: set([]),
            #     6: set([]),
            #     7: set([]),
            # }
            "stonehenge": {
                1: set([]),
                2: set([]),
                3: set([1,2]),
                4: set([]),
                5: set([]),
                6: set([4,5]),
                7: set([]),
                8: set([]),
                9 : set([7,8]),
                10: set([]),
                11: set([]),
                12: set([11,10]),
                13: set([]),
                14: set([]),
                15: set([13,14]),
                16: set([]),
                17: set([]),
                18: set([17,16]),
            },
        },
    }

    '''
    Data Dirs
    ''' 
    PROBLEM_STATES_DIR = ROOT_DIR + "problem_sets/" + DOMAIN_NAME + "/"
    DATA_MISC_DIR = DATA_DIR + "misc/"
    OBJECT_PAIR_ROMS_DIR = DATA_MISC_DIR + "object_pair_roms/"
    DATA_TEST_TRAJS = DATA_MISC_DIR + "test_trajs/"
    DOMAIN_FILE = PDDL_DIR + "domain.pddl"
    # PROBLEM_DIR = PDDL_DIR + DOMAIN_NAME + "/problem_files/"

    IMMOVABLE_OBJECTS = LOCATION_NAME + ["droparea","spawnarea","surface"]
    NON_RELATION_OBJECTS = ["droparea","spawnarea","table6","world_final"]
    AXIS_MAP = {
        "Keva": {'x':0.059, #0.05850000999999999
                'y':0.0115, #0.011499999859999999
                'z':0.006,
                "xd":0.057
                },

        "Jenga": {
            "x": 0.0762,
            'y':0.0254, #0.011499999859999999
            'z':0.01524,
        }
    }

    CONST_TYPES = {
        "CafeWorld": ["goalLoc"],
        "MultiKeva": ["goalLoc"],
        "Keva": ["goalLoc"],
        "Packing": ["goalLoc"],
        # "DinnerTable": ["bowltargetLoc","glasstargetLoc"]
        "DinnerTable": [],
        "Jenga": ["goalLoc"]
    }
    # CONST_TYPES = []

    KNOWN_COST = 1
    UNKNOWN_COST = 100

    MAX_IK_ATTEMPTS = 5
    MAX_CALL_COUNTS = 100
    SAMPLE_COUNT = 3
    MP_MAX_COUNT = 5
    MAX_REGION_SAMPLING_COUNT = 3

    TIMEOUT_THRESHOLD = {
        "CafeWorld": 1800,
        "Keva": -1,
        "Packing": 300,
        "MultiKeva": -1,
        "DinnerTable": -1,
        "Jenga": -1
    }

    DIFFERENT_ENV_FILES_USED = {
        "CafeWorld": 8,
        "DinnerTable": 1,
        "Packing": 1,
        "Keva": 3, #1 random, 1 pi and 1 3 planks parallel,
        "MultiKeva": 2, #1 random, 1 triangle,
        "Jenga": 3 #1 random, 1 pi and 1 3 planks parallel,
    }

    MOVABLE_OBJECTS_DOMAINS = {
        "Keva": ["plank"],
        "CafeWorld": ["can"],
        "Packing": ["can"],
        "Jenga": ["jenga"],
        "DinnerTable": ["bowl","glass"]
    }

    MOVABLE_OBJECTS = MOVABLE_OBJECTS_DOMAINS[DOMAIN_NAME]

    CAMERA_TRANSFORM_FILE = ROOT_DIR + "{}_camera_transform.npy".format(DOMAIN_NAME)

def get_discretizer(obj1,obj2):
    global DOMAIN_NAME
    key_set = frozenset([obj1,obj2])
    return Discretizer(*DOMAIN_DISCRETIZER_PARAMS[DOMAIN_NAME][key_set])
