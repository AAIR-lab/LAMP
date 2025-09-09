"""
This module contains the configuration parameters for the rest of the codebase.
Domain specific parameters are stored in their respective Config files named <Domain>Config.py in LAMP/domain_configs/
"""

import os
import numpy as np
from src.utilities.Discretizer import Discretizer
import math
import importlib

class Config():
    ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + "/"
    DOMAIN_CONFIG_DIR = ROOT_DIR + "domain_configs/"
    ALT_PLAN_DIR = ROOT_DIR + "plan_files/"
    COP_DIR = ROOT_DIR + "base_lines/code_as_policies/"
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
    KP_SOLUTION_FILE = ROOT_DIR + KP_SOLUTION_NAME
    SOLVED_PLAN_FILES = ROOT_DIR + "solved_plans/"
    TEST_DIR = ROOT_DIR + "test_trajectories/"
    KNOWN_COST = 1
    UNKNOWN_COST = 100
    MAX_IK_ATTEMPTS = 5
    MAX_CALL_COUNTS = 100
    SAMPLE_COUNT = 3
    MP_MAX_COUNT = 5
    MAX_REGION_SAMPLING_COUNT = 3
    PLANNER_TIME_OUT = 3*3600
    K_PLANS = 50
    SURFACE_NAME = ["surface"]
    PRE_GRAB_CR = 2
    BIN_COUNT = np.array([64,64,64,60,60,60])
    discretizer = Discretizer(world_n_bins=BIN_COUNT)
    ROBOT_TYPES = {
        "gripper":"manip_joints", 
        "freight":"base_joints"
    }
    DUMMY_TYPES = ["goalLoc","droparea"]
    GRIPPER_NAME = "gripper"
    BASE_NAME = "freight"
    MAX_POSES = 5
    NUM_GRASPS = 4
    GRAB_BINS = 3
    SENSOR_BIN_LIST = [GRAB_BINS]
    GRAB_INDEX = -1
    SENSOR_COUNT = len(SENSOR_BIN_LIST)
    OBJ_ID_IND = -1
    CONST_TYPE_IND = -2
    OBJ_TYPE_IND = 0
    DEFAULT_NUM_COMPONENTS = 10
    SIMULATOR = "openrave"
    SIM_DIR = ROOT_DIR + "simulators/" + SIMULATOR + "/"

    @staticmethod
    def declare_config(domain_name,robot_name=None):
        Config.DOMAIN_NAME = domain_name
        Config.update_domain_specific_params()

        if robot_name is not None:
            Config.ROBOT_NAME = robot_name
        
        Config.COP_DOMAIN_DIR = Config.COP_DIR + Config.DOMAIN_NAME + "/"
        Config.DATA_DIR = Config.ROOT_DIR + "Data/" + Config.DOMAIN_NAME + "/"
        Config.MODEL_DIR = Config.ROOT_DIR + "Model/" + Config.DOMAIN_NAME + "/"
        Config.PDDL_DIR = Config.ROOT_DIR + "Domains/" + Config.DOMAIN_NAME + "/"
        
        Config.FF_SOLUTION_DIR = Config.PDDL_DIR
        Config.FF_SOLUTION_FILE = Config.PDDL_DIR
        Config.STD_OUT_DIR = Config.ROOT_DIR + "Data/" + Config.DOMAIN_NAME + "/std_out_logs/"

        Config.PROBLEM_STATES_DIR = Config.ROOT_DIR + "problem_sets/" + Config.DOMAIN_NAME + "/"
        Config.DATA_MISC_DIR = Config.DATA_DIR + "misc/"
        Config.DOMAIN_FILE = Config.PDDL_DIR + "domain.pddl"

        Config.set_sim_config()
        
    @staticmethod
    def set_sim_config():
        sim_config_object = Config.get_simulator_module("SimConfig").SimConfig
        sim_attributes = getattr(sim_config_object,"REQ_ATTR")
        for a in sim_attributes:
            setattr(sim_config_object,a,getattr(Config,a))
            
        sim_config_object.update_dependant_attributes()
        Config.set_sim_config()
        
    @staticmethod
    def set_sim_config():
        sim_config_object = Config.get_simulator_module("SimConfig").SimConfig
        sim_attributes = getattr(sim_config_object,"REQ_ATTR")
        for a in sim_attributes:
            setattr(sim_config_object,a,getattr(Config,a))
            
        sim_config_object.update_dependant_attributes()

    @staticmethod
    def get_discretizer(obj1,obj2):
        key_set = frozenset([obj1,obj2])
        return Discretizer(*(Config.DISCRETIZER_PARAMS)[key_set])

    @staticmethod
    def get_eval_thresholding_values(obj1,obj2):
        key_set = frozenset([obj1,obj2])
        return Config.EVAL_THRESHOLDING_VALUES[key_set]
    
    @staticmethod
    def update_domain_specific_params():
        domain_mod = importlib.import_module("domain_configs.{}Config".format(Config.DOMAIN_NAME))
        [ setattr(Config,attr,getattr(domain_mod,attr)) for attr in dir(domain_mod) if attr.isupper() ]       

    @staticmethod
    def set_ros_paths():
        current_dir = os.path.dirname(os.path.abspath(__file__))
        while True:
            if "ros_ws" in os.listdir(current_dir):
                break
            else:
                current_dir = os.path.abspath(os.path.join(current_dir, os.pardir))

        Config.ROS_WS_DIR = current_dir + "/ros_ws/"
        Config.REAL_EXPERIMENT_FILE = Config.ROS_WS_DIR + "objects/env.dae"
        Config.ROS_TRAJ_PATHS = Config.ROS_WS_DIR + "src/mocap/rcr_integration/saved_trajectories/"
    
    @staticmethod
    def get_simulator_module(mod_name):
        mod = importlib.import_module("simulators.{}.{}".format(Config.SIMULATOR,mod_name))
        return mod