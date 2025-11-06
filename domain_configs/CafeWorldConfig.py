import numpy as np
import math

OBJECT_NAME = ["can"]
ROBOT_NAME = "Fetch"
SURFACES = ["countertop","table"]

MVG_SCORE_THRESHOLD = 3

EVAL_THRESHOLDING_VALUES = {
                        frozenset(["gripper","can"])        : 0.1,#0.0195,
                        frozenset(["freight","surface"])    : 0.001,#0.248,
                        frozenset(["surface","can"])        : 0.3,
                        frozenset(["gripper","freight"])    : 0.6,#1.0,
                        frozenset(["freight","can"])        : 0.9,
                        frozenset(["can"])                  : 1.0,
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
                        frozenset(["gripper","can"])        : (np.array([-0.064,-0.001,-0.026,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.064,0.001,0.026,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([2,1,1,21,21,4]),
                                                            ),

                        frozenset(["freight","surface"])    : (np.array([-0.26,-0.005,-0.28,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.26,0.005,0.28,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([2,1,1,21,21,4]),
                                                            ),

                        frozenset(["surface","can"])        : (np.array([-0.05,-0.05,-0.0165,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.05,0.05,0.0165,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([1,1,2,21,21,4]),
                                                            ),

                        frozenset(["gripper","freight"])    : (np.array([-0.002,-0.0064,-0.0021,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.002,0.0064,0.0021,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([1,5,10,4,4,4]),
                                                            ),                        

                        frozenset(["freight","can"])        : (np.array([-0.002,-0.0064,-0.0021,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.002,0.0064,0.0021,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([1,5,10,4,4,4]),
                                                            ),                                                             

                        frozenset(["can"])                  : (np.array([-0.001,-0.001,-0.001,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.001,0.001,0.001,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([1,1,1,6,6,6]),
                                                            )
}

CONST_TYPES = ["loc"]
CONST_NAMES = []
NON_RELATION_OBJECTS = ["world_final"]
LOCATION_NAME = []
IMMOVABLE_OBJECTS = LOCATION_NAME + ["world_final"] + SURFACES
