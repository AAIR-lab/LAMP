import os
import numpy as np
import math
import sys

# DOMAIN_NAME = "CafeWorld"
DOMAIN_NAME = "Keva"
# DOMAIN_NAME = "Packing"

ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + "/"
DATA_DIR = ROOT_DIR+"data/"+DOMAIN_NAME+"/"
PARENT_DIR = os.path.abspath(os.path.join(ROOT_DIR, os.pardir))
sys.path.append(PARENT_DIR)


NON_RELATION_OBJECTS = ["droparea","spawnarea","table6","world_final"]
CONST_TYPES = {
    "CafeWorld": ["goalLoc"],
    "Keva": ["goalLoc"],
    "Packing": ["goalLoc"],
}

DOMAIN_DISCRETIZER_PARAMS = {
    "CafeWorld" : {
                    frozenset(["gripper","freight"])    : (np.array([-0.15,-0.15,-0.21,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                           np.array([0.15,0.15,0.21,math.radians(180),math.radians(180),math.radians(180)]),
                                                           np.array([15,15,15,61,61,61]),
                                                           ),
   
                    frozenset(["gripper","can"])        : (np.array([-0.0825,-0.0825,-0.0825,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                           np.array([0.0825,0.0825,0.0825,math.radians(180),math.radians(180),math.radians(180)]),
                                                           np.array([8,8,8,61,61,61]),
                                                           ),

                    frozenset(["freight","can"])        : (np.array([-0.31,-0.31,-0.31,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                           np.array([0.31,0.31,0.31,math.radians(180),math.radians(180),math.radians(180)]),
                                                           np.array([20,20,20,61,61,61]),
                                                           ),                                                           
  
                    frozenset(["freight","surface"])    : (np.array([-0.35,-0.35,-2.25,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                           np.array([0.35,0.35,2.25,math.radians(180),math.radians(180),math.radians(180)]),
                                                           np.array([2,2,1,41,41,41]),
                                                           ),

                    frozenset(["can","goalLoc"])        : (np.array([-0.225,-0.225,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                           np.array([0.225,0.225,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                           np.array([4,4,2,11,11,11]),
                                                           ),

                    frozenset(["surface","can"])        : (np.array([-0.215,-0.215,-0.015,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                           np.array([0.215,0.215,0.015,math.radians(180),math.radians(180),math.radians(180)]),
                                                           np.array([1,1,1,11,11,11]),
                                                           ),

                    frozenset(["can"])                  : (np.array([-0.5,-0.5,-0.5,-math.radians(180),-math.radians(180),-math.radians(180)]),
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
}

OBJECT_COUNT = 2
SURFACE_COUNT = 2
DOMAIN_OBJECTS = {
    "CafeWorld": ["freight_1","gripper_1"] + ["can_{}".format(i+1) for i in range(OBJECT_COUNT)] + ["surface_{}".format(i) for i in range(SURFACE_COUNT)],
    "Keva": ["gripper_1"] + ["plank_{}".format(i+1) for i in range(OBJECT_COUNT)],
    "Packing": ["gripper_1","surface_1"] + ["can_{}".format(i+1) for i in range(min(4,OBJECT_COUNT))]
}