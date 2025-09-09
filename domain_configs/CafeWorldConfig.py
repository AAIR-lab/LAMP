import numpy as np
import math

OBJECT_NAME = ["can"]
ROBOT_NAME = "Fetch"
SURFACES = ["countertop","table"]
MVG_THRESHOLD = 0.8

EVAL_THRESHOLDING_VALUES = {
                        frozenset(["gripper","freight"])    : 1.0,#1.0,
                        frozenset(["gripper","can"])        : 0.01,#0.0195,
                        frozenset(["freight","surface"])    : 0.05,#0.248,
                        frozenset(["can","loc"])            : 0.0011,#0.04,
                        frozenset(["surface","can"])        : 0.2,
                        frozenset(["can"])                  : 1.0,
                        frozenset(["freight","can"])        : 0.1
}

BOUND_OBJECT_NAME = "world_final"
NON_AUX_OBJECTS_SET =  frozenset(["surface","loc"])

"""
These params are used to discretize the relative poses of the objects w.r.t. each other.
The params are tuple of arrays representing the range of the relative pose which will be considered for discretization.
The values beyond these ranges, will be marked -1. 
The last element of the tuple represent the arrays consisting of number of bins to use for discretization between the given end points.
"""
DISCRETIZER_PARAMS = {
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
}

CONST_TYPES = ["loc"]
CONST_NAMES = []
NON_RELATION_OBJECTS = ["world_final"]
LOCATION_NAME = []
IMMOVABLE_OBJECTS = LOCATION_NAME + ["world_final"] + SURFACES