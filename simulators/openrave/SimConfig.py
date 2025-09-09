import os
import sys

def get_parent_with_file(file_name):
    current_dir = os.getcwd()
    while True:
        if file_name in os.listdir(current_dir):
            return current_dir
        else:
            current_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
    
    raise Exception("File Not Found in any Parent")

class SimConfig():
    SIM_DIR = os.path.dirname(os.path.abspath(__file__)) + "/"
    SIM_ROOT_DIR = os.path.abspath(os.path.join(SIM_DIR, os.pardir)) + "/"
    REQ_ATTR = ["ROOT_DIR",
                "BASE_NAME",
                "OBJ_TYPE_IND",
                "OBJ_ID_IND",
                "ROBOT_NAME",
                "OBJECT_NAME",
                "SENSOR_COUNT",
                "DOMAIN_NAME",
                "ROBOT_TYPES",
                "BOUND_OBJECT_NAME",
                "MAX_IK_ATTEMPTS",
                "LOCATION_NAME"]

    ROB_DIR = SIM_ROOT_DIR + "robot_files/"
    CAMERA_ARRAYS = SIM_ROOT_DIR + "camera_arrays/"

    @staticmethod
    def update_dependant_attributes():
        SimConfig.REL_URDF_DIR = "robot_files/" + SimConfig.ROBOT_NAME + "/URDF/"
        SimConfig.URDF_DIR = SimConfig.ROB_DIR + SimConfig.ROBOT_NAME + "/URDF/"

        SimConfig.BASE_ENV_DIR = SimConfig.SIM_DIR + "Environments/" + SimConfig.DOMAIN_NAME + "/"
        SimConfig.ENV_DIR = SimConfig.BASE_ENV_DIR + "env/"
        SimConfig.PLANK_PAIRS_DIR = SimConfig.BASE_ENV_DIR + "plank_relation_structures/"
        SimConfig.REFERENCE_DIR = SimConfig.BASE_ENV_DIR + "reference_structure/"
        SimConfig.OBJECTS_DIR = SimConfig.BASE_ENV_DIR + "objects/"