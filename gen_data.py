import importlib
import numpy as np
import argparse
import sys
import os
import json
import Config

def get_argument_dict(problem_name="",only_problems=False):
    module_name = os.path.basename(__file__)[:-3]
    file_name = Config.ROOT_DIR + "argument_files/" + module_name + ".json"

    with open(file_name,"r") as f:
        data_dict = json.load(f)
        f.close()
    
    domain_dict = data_dict[Config.DOMAIN_NAME]["general"]
    if problem_name != "":
        problems_dict = data_dict[Config.DOMAIN_NAME]["problems"][problem_name]      
        if not only_problems:
            domain_dict.update(problems_dict)
        else:
            domain_dict = problems_dict

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
    argParser.add_argument("-p","--plank_count",              help = "plank count for exp",                  nargs="?")
    argParser.add_argument("-t","--reference_structure_name", help = "name of test structure",               )
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

    object_names = args.object_names
    visualize = args.visualize
    axis_for_offset = args.axis_for_offset
    reference_structure_name = args.reference_structure_name
    random_env_setup = args.random_env_setup
    order_plank_placement = args.order_plank_placement
    planks_in_init_state = args.planks_in_init_state
    surface_as_goal = args.surface_as_goal    
    quadrant_number = args.quadrant_number
    grasp_number = args.grasp_number
    minimum_plank_count = args.minimum_plank_count
    no_motion_plan = args.no_motion_plan
    set_y_flag = args.set_y_flag
    complete_random = args.complete_random
    false_load_arguments = args.false_load_arguments

    mod = importlib.import_module("src.data_gen.{}".format(Config.DOMAIN_NAME))
    ClassFunction = getattr(mod,"GetDomain")

    if random_env_setup is not None:
        random_env_setup = int(args.random_env_setup)
    else:
        random_env_setup = 0

    if minimum_plank_count is not None:
        minimum_plank_count = int(minimum_plank_count)

    if quadrant_number is not None:
        quadrant_number = int(quadrant_number)

    if grasp_number is not None:
        grasp_number = int(grasp_number)

    if planks_in_init_state is None:
        planks_in_init_state = 0
    else:
        planks_in_init_state = int(planks_in_init_state)
    
    arguments_set = set([])
    if false_load_arguments:
        thismodule = sys.modules[__name__]
        argument_dict = get_argument_dict()
        for key in argument_dict.keys():
            setattr(thismodule, key, argument_dict[key])
            arguments_set.add(key)
    Config.declare_config(domain,robot)

    if args.name is not None:
        env_list = args.name

    for env_num,name in enumerate(env_list):
        problem_arguments = set([])
        if false_load_arguments:
            thismodule = sys.modules[__name__]
            argument_dict = get_argument_dict(problem_name=name)
            for key in argument_dict.keys():
                if key not in arguments_set:
                    setattr(thismodule, key, argument_dict[key])
                    problem_arguments.add(key)
                
        if args.plank_count is not None:
            plank_count = args.plank_count
        
        if plank_count is not None:
            plank_count = int(plank_count)

        calls_to_mp = 1
        if "Keva" in Config.DOMAIN_NAME or "Jenga" in Config.DOMAIN_NAME:
            random_count = 20
        elif "Packing" in Config.DOMAIN_NAME:
            random_count = 250
        else:
            random_count = 200
        
        ref_env = reference_structure_name
        # if reference_structure_name is not None:
        #     ref_env = reference_structure_name[env_num]

        itter_count = random_count*calls_to_mp
        
        dg_class = ClassFunction(robot_name=robot)
        dg = dg_class(env_name=name,
                      number_of_configs=random_count,
                      number_of_mp=calls_to_mp,
                      file_name=name+"_data.p",
                      axis_for_offset=axis_for_offset,
                      visualize=visualize,
                      plank_count=plank_count,
                      reference_structure_name=ref_env,
                      random=random_env_setup,
                      order=order_plank_placement,
                      planks_in_init_state=planks_in_init_state,
                      surface=surface_as_goal,
                      quadrant=quadrant_number,
                      grasp_num=grasp_number,
                      minimum_plank_count=minimum_plank_count,
                      mp=no_motion_plan,
                      object_list=object_names,
                      set_y=set_y_flag,
                      complete_random=complete_random)
    
        dg.start(complete_random)            
        dg.env.Destroy()
        
        for arg in problem_arguments:
            setattr(thismodule, arg, getattr(args,arg))