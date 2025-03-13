import importlib
import numpy as np
import argparse
import sys
import os
import json
import Config

def get_argument_dict(problem_name=""):
    module_name = os.path.basename(__file__)[:-3]
    file_name = Config.ROOT_DIR + "argument_files/" + module_name + ".json"

    with open(file_name,"r") as f:
        data_dict = json.load(f)
        f.close()
    
    domain_dict = data_dict[Config.DOMAIN_NAME]["general"]
    if problem_name != "":
        problems_dict = data_dict[Config.DOMAIN_NAME]["problems"][problem_name]      
        domain_dict.update(problems_dict)
    
    return domain_dict

if __name__ == "__main__":

    argParser = argparse.ArgumentParser()
    argParser.add_argument("-v","--visualize",                help = "visualize the sim",                    action="store_true")
    argParser.add_argument("-o","--order_plank_placement",    help = "set to remove random order of planks", action="store_false")
    argParser.add_argument("-R","--random_env_setup",         help = "set a random goal state",              nargs="?",default=-1)
    argParser.add_argument(     "--no_motion_plan",           help = "not to compute motionplan",            action="store_false")
    argParser.add_argument("-a","--axis_for_offset",          help = "axis for offset",                      default='x')
    argParser.add_argument("-s","--surface_as_goal",          help = "surface as goal",                      nargs='*',default="")
    argParser.add_argument("-n","--name",                     help = "name of env",                          nargs='*')
    argParser.add_argument(     "--object_names",             help = "name of objects to manipulate",        nargs='*')
    argParser.add_argument("-p","--plank_count",              help = "plank count for exp",                  nargs="?",default=1)
    argParser.add_argument("-t","--reference_structure_name", help = "name of test structure",               nargs='*')
    argParser.add_argument("-c","--planks_in_init_state",     help = "planks_in_init_state",                 nargs="?",default=0)
    argParser.add_argument("-q","--quadrant_number",          help = "quadrant to use",                      nargs="?")
    argParser.add_argument("-m","--minimum_plank_count",      help = "minimum number of objects to use",     nargs="?")
    argParser.add_argument("-g","--grasp_number",             help = "grasp to use",                         nargs="?")
    argParser.add_argument(     "--set_y_flag",               help = "flag for setting objects on y axis",   action="store_true")
    argParser.add_argument(     "--complete_random",          help = "flag for collecting random trajs",     action="store_true")
    argParser.add_argument(     "--false_load_arguments",     help = "don't load arguments",                 action="store_false")
    argParser.add_argument("-r","--robot",                    help = "name of robot",                        )
    argParser.add_argument("-d","--domain",                    help = "name of domain",                        )
    
    args = argParser.parse_args()
    domain = args.domain
    robot = args.robot
    Config.declare_config(domain,robot)

    env_list = args.name
    object_list = args.object_names
    visualize = args.visualize
    plank_count = args.plank_count
    axis = args.axis_for_offset
    reference_structure = args.reference_structure_name
    R = args.random_env_setup
    o = args.order_plank_placement
    planks_in_init_state = args.planks_in_init_state
    surface = args.surface_as_goal    
    quadrant = args.quadrant_number
    grasp_num = args.grasp_number
    min_planks = args.minimum_plank_count
    mp_flag = args.no_motion_plan
    set_y = args.set_y_flag
    complete_random = args.complete_random
    load_arguments = args.false_load_arguments

    mod = importlib.import_module("src.data_gen.{}".format(Config.DOMAIN_NAME))
    ClassFunction = getattr(mod,"GetDomain")

    if R is not None:
        if plank_count is None:
            raise Exception("No Plank Count Given")
        R = int(R)
    else:
        R = 0
    
    if plank_count is not None:
        plank_count = int(plank_count)

    if min_planks is not None:
        min_planks = int(min_planks)

    if quadrant is not None:
        quadrant = int(quadrant)

    if grasp_num is not None:
        grasp_num = int(grasp_num)

    # if reference_structure is not None:
    #     reference_structure = reference_structure[0]

    if planks_in_init_state is None:
        planks_in_init_state = 0
    else:
        planks_in_init_state = int(planks_in_init_state)
    
    arguments_set = set([])
    if load_arguments:
        thismodule = sys.modules[__name__]
        argument_dict = get_argument_dict()
        for key in argument_dict.keys():
            setattr(thismodule, key, argument_dict[key])
            arguments_set.add(key)
    Config.declare_config(domain,robot)

    for env_num,name in enumerate(env_list):
        if load_arguments:
            thismodule = sys.modules[__name__]
            argument_dict = get_argument_dict(problem_name=name)
            for key in argument_dict.keys():
                if key not in arguments_set:
                    setattr(thismodule, key, argument_dict[key])
        
        calls_to_mp = 1
        if "Keva" in Config.DOMAIN_NAME or "Jenga" in Config.DOMAIN_NAME:
            random_count = 10
        elif "Packing" in Config.DOMAIN_NAME:
            random_count = 250
        else:
            random_count = 200
        
        ref_env = reference_structure
        if reference_structure is not None:
            ref_env = reference_structure[env_num]

        itter_count = random_count*calls_to_mp
        
        dg_class = ClassFunction(robot_name=robot)
        dg = dg_class(env_name=name,
                      number_of_configs=random_count,
                      number_of_mp=calls_to_mp,
                      file_name=name+"_data.p",
                      axis_for_offset=axis,
                      visualize=visualize,
                      plank_count=plank_count,
                      reference_structure_name=ref_env,
                      random=R,
                      order=o,
                      planks_in_init_state=planks_in_init_state,
                      surface=surface,
                      quadrant=quadrant,
                      grasp_num=grasp_num,
                      minimum_plank_count=min_planks,
                      mp=mp_flag,
                      object_list=object_list,
                      set_y=set_y,
                      complete_random=complete_random)
        
        if complete_random:
            dg.random_start()
        else:
            dg.start()            
        dg.env.Destroy()
