import numpy as np
import math

OBJECT_NAME = ["bowl", "glass"]
ROBOT_NAME = "Fetch"
SURFACES = ["countertop","table","surfaces"]
MVG_THRESHOLD = 0.8

EVAL_THRESHOLDING_VALUES = {
                        frozenset(["gripper","freight"])    : 1.0,#1.0,
                        frozenset(["gripper","glass"])        : 0.001,#0.0195,
                        frozenset(["gripper","bowl"])       : 0.001,#0.0195,
                        frozenset(["glass","glasstargetLoc"])        : 1.0,#0.04,
                        frozenset(["glass","bowltargetLoc"])        : 1.0,#0.04,
                        frozenset(["glass","bowlinitLoc"])        : 1.0,#0.04,
                        frozenset(["glass","glassinitLoc"])        : 1.0,#0.04,
                        frozenset(["bowl","bowltargetLoc"])       : 1.0,#0.04,
                        frozenset(["bowl","bowlinitLoc"])       : 1.0,#0.04,
                        frozenset(["bowl","glasstargetLoc"])       : 1.0,#0.04,
                        frozenset(["bowl","glassinitLoc"])        : 1.0,
                        frozenset(["freight","glasstargetLoc"])       : 0.04,#0.04,
                        frozenset(["freight","glassinitLoc"])       : 0.04,#0.04,
                        frozenset(["freight","bowltargetLoc"])       : 0.04,#0.04,
                        frozenset(["freight","bowlinitLoc"])       : 0.05,#0.04,
                        frozenset(["glass"])                  : 1.0,
                        frozenset(["bowl"])                 : 1.0,
                        frozenset(["glass","bowl"])           : 1.0,
                        frozenset(["freight","glass"])        : 0.5,
                        frozenset(["freight","bowl"])       : 0.5
}

BOUND_OBJECT_NAME = "world_final"
NON_AUX_OBJECTS_SET =  frozenset(["surface","glasstargetLoc","bowltargetLoc"])

"""
These params are used to discretize the relative poses of the objects w.r.t. each other.
The params are tuple of arrays representing the range of the relative pose which will be considered for discretization.
The values beyond these ranges, will be marked -1. 
The last element of the tuple represent the arrays consisting of number of bins to use for discretization between the given end points.
"""
DISCRETIZER_PARAMS = {
                        frozenset(["gripper","freight"])    : (np.array([-0.2,-0.2,-1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.2,0.2,1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([32,32,32,41,41,41]),
                                                            ),
    
                        frozenset(["gripper","glass"])        : (np.array([-0.082,-0.0072,-0.045,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.082,0.0072,0.045,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([16,1,4,41,41,4]),
                                                            ),

                        frozenset(["gripper","bowl"])       : (np.array([-0.11,-0.035,-0.0142,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.11,0.035,0.0142,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([14,4,2,41,41,4]),
                                                            ),

                        frozenset(["freight","glass"])        :  (np.array([-0.2,-0.2,-1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.2,0.2,1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([3,3,3,3,3,3]),
                                                            ),  

                        frozenset(["freight","bowl"])       : (np.array([-0.2,-0.2,-1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.2,0.2,1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([3,3,3,3,3,3]),
                                                            ),                                                    
    
                        frozenset(["freight","bowltargetLoc"])    : (np.array([-0.3,-0.3,-0.3,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.3,0.3,0.3,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([2,2,10,41,41,41]),
                                                            ),

                        frozenset(["freight","bowlinitLoc"])    : (np.array([-0.3,-0.3,-0.3,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.3,0.3,0.3,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([2,2,10,41,41,41]),
                                                            ),
                                                            
                        frozenset(["glass","glasstargetLoc"])        : (np.array([-0.004,-0.004,-0.004,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.004,0.004,0.004,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([1,1,1,21,21,5]),
                                                            ),

                        frozenset(["glass","glassinitLoc"])        : (np.array([-0.004,-0.004,-0.004,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.004,0.004,0.004,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([1,1,1,21,21,5]),
                                                            ),

                        frozenset(["glass","bowltargetLoc"])        : (np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([32,32,32,61,61,61]),
                                                            ),

                        frozenset(["glass","bowlinitLoc"])        : (np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([32,32,32,61,61,61]),
                                                            ),

                        frozenset(["bowl","bowltargetLoc"])       :(np.array([-0.004,-0.004,-0.004,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.004,0.004,0.004,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([1,1,1,21,21,5]),
                                                            ),

                        frozenset(["bowl","bowlinitLoc"])       :(np.array([-0.004,-0.004,-0.004,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.004,0.004,0.004,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([1,1,1,21,21,5]),
                                                            ),

                        frozenset(["bowl","glasstargetLoc"])       :(np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([32,32,32,61,61,61]),
                                                            ),

                        frozenset(["bowl","glassinitLoc"])       :(np.array([-0.1,-0.1,-0.1,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.1,0.1,0.1,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([32,32,32,61,61,61]),
                                                            ),

                        frozenset(["freight","glasstargetLoc"])    : (np.array([-0.3,-0.3,-0.3,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.3,0.3,0.3,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([2,2,10,41,41,41]),
                                                            ),

                        frozenset(["freight","glassinitLoc"])    : (np.array([-0.3,-0.3,-0.3,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                            np.array([0.3,0.3,0.3,math.radians(180),math.radians(180),math.radians(180)]),
                                                            np.array([2,2,10,41,41,41]),
                                                            ),

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

}

CONST_TYPES = []
CONST_NAMES = []
NON_RELATION_OBJECTS = ["world_final"] + SURFACES
LOCATION_NAME = ["glasstargetLoc","bowltargetLoc","targetLoc","initLoc","glassinitLoc","bowlinitLoc"]
IMMOVABLE_OBJECTS = LOCATION_NAME + ["world_final"] + SURFACES