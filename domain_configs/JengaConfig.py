import numpy as np
import math

OBJECT_NAME = ["jenga"]
ROBOT_NAME = "Fetch"
SURFACES = []
MVG_THRESHOLD = 0.9

EVAL_THRESHOLDING_VALUES = {
                        frozenset(["gripper","jenga"]) : 1.0,
                        frozenset(["jenga","loc"]) : 1.0,
                        frozenset(["jenga","jenga"])   : 1.0,
}

BOUND_OBJECT_NAME = "smalltable"
NON_AUX_OBJECTS_SET = frozenset([])

DISCRETIZER_PARAMS = {
                    frozenset(["gripper","jenga"]) : (np.array([-0.0825,-0.01,-0.01,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                    np.array([0.0825,0.01,0.01,math.radians(180),math.radians(180),math.radians(180)]),
                                                    np.array([35,5,5,61,61,61]),
                                                    ),

                    frozenset(["jenga","loc"]) : (np.array([-0.01,-0.01,-0.01,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                    np.array([0.01,0.01,0.01,math.radians(180),math.radians(180),math.radians(180)]),
                                                    np.array([3,3,3,61,61,61]),
                                                    ),
                                                    
                    frozenset(["jenga","jenga"])   : (np.array([-0.035,-0.02,-0.035,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                    np.array([0.035,0.02,0.035,math.radians(180),math.radians(180),math.radians(180)]),
                                                    np.array([3,3,3,61,61,61]),
                                                    ),
}

"""
These params are used to discretize the relative poses of the objects w.r.t. each other.
The params are tuple of arrays representing the range of the relative pose which will be considered for discretization.
The values beyond these ranges, will be marked -1. 
The last element of the tuple represent the arrays consisting of number of bins to use for discretization between the given end points.
"""
PLANKS_PROBLEM_ORDER = {
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
}

AXIS_MAP = {
            "x": 0.0762,
            'y':0.0254, #0.011499999859999999
            'z':0.01524,
}

CONST_TYPES = ["loc"]
CONST_NAMES = ["{}Target".format(o) for o in OBJECT_NAME]
NON_RELATION_OBJECTS = ["droparea","spawnarea","smalltable","tabletop","freight"]#TODO: fix this
LOCATION_NAME = ["loc"]
IMMOVABLE_OBJECTS = LOCATION_NAME + ["droparea","spawnarea","tabletop"] + SURFACES