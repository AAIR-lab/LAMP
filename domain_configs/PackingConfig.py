import numpy as np
import math

OBJECT_NAME = ["can"]
ROBOT_NAME = "MagneticGripper"
MVG_THRESHOLD = 0.8

EVAL_THRESHOLDING_VALUES = {
                        frozenset(["gripper","can"])  : 1.0,
                        frozenset(["can","surface"])  : 0.032,
                        frozenset(["can","can"])      : 1.0,
}

BOUND_OBJECT_NAME = "table6"
NON_AUX_OBJECTS_SET = frozenset(["can","surface"])

"""
These params are used to discretize the relative poses of the objects w.r.t. each other.
The params are tuple of arrays representing the range of the relative pose which will be considered for discretization.
The values beyond these ranges, will be marked -1. 
The last element of the tuple represent the arrays consisting of number of bins to use for discretization between the given end points.
"""
DISCRETIZER_PARAMS = {
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
}

SURFACES = ["surface"]
CONST_TYPES = []
CONST_NAMES = []
NON_RELATION_OBJECTS = ["table6"]
LOCATION_NAME = []
IMMOVABLE_OBJECTS = LOCATION_NAME + ["table6"]