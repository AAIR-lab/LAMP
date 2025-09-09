import numpy as np
import math

OBJECT_NAME = ["plank"]
ROBOT_NAME = "yumi"
SURFACES = []

EVAL_THRESHOLDING_VALUES = {
                        frozenset(["gripper","plank"]) : 1.0,
                        frozenset(["plank","loc"]) : 1.0,
                        frozenset(["plank","plank"])   : 1.0,
}

BOUND_OBJECT_NAME = "table6"
NON_AUX_OBJECTS_SET = frozenset([])
MVG_THRESHOLD = 0.9

"""
These params are used to discretize the relative poses of the objects w.r.t. each other.
The params are tuple of arrays representing the range of the relative pose which will be considered for discretization.
The values beyond these ranges, will be marked -1. 
The last element of the tuple represent the arrays consisting of number of bins to use for discretization between the given end points.
"""
DISCRETIZER_PARAMS = {
                        frozenset(["gripper","plank"]) : (np.array([-0.0015,-0.0015,-0.048,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                        np.array([0.0015,0.0015,0.048,math.radians(180),math.radians(180),math.radians(180)]),
                                                        np.array([1,1,10,61,61,61]),
                                                        ),

                        frozenset(["plank","loc"]) : (np.array([-0.005,-0.005,-0.005,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                        np.array([0.005,0.005,0.005,math.radians(180),math.radians(180),math.radians(180)]),
                                                        np.array([1,1,1,31,31,31]),
                                                        ),

                        frozenset(["plank","plank"])   : (np.array([-0.07,-0.07,-0.07,-math.radians(180),-math.radians(180),-math.radians(180)]),
                                                        np.array([0.07,0.07,0.07,math.radians(180),math.radians(180),math.radians(180)]),
                                                        np.array([26,26,26,51,51,51]),
                                                        ),
}

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
}

AXIS_MAP = {'x':0.059, #0.05850000999999999
            'y':0.0115, #0.011499999859999999
            'z':0.006,
            "xd":0.057
}

CONST_TYPES = ["loc"]
CONST_NAMES = ["{}Target".format(o) for o in OBJECT_NAME]
NON_RELATION_OBJECTS = ["droparea","table6"]
LOCATION_NAME = ["loc"]
IMMOVABLE_OBJECTS = LOCATION_NAME + ["droparea","spawnarea"] + SURFACES
