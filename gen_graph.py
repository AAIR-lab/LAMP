import numpy as np
import argparse
import sys
import json
import os
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

if __name__ == '__main__':

    argParser = argparse.ArgumentParser() 
    argParser.add_argument("-n","--name",                     help = "name of env",                          nargs='*')
    argParser.add_argument("-t","--test_structure_name",      help = "name of test structure",               nargs='*',default="")
    argParser.add_argument("-v","--visualize",                help = "visualize the sim",                    action="store_true")
    argParser.add_argument("-p","--plank_count",              help = "plank count for exp",                  nargs="?",default=1)
    argParser.add_argument(     "--planks_in_init_state",     help = "planks_in_init_state",                 nargs="?",default=0)
    argParser.add_argument("-g","--get_graph",                help = "get pruned or complete graph",         action="store_true")
    argParser.add_argument("-e","--execute",                  help = "execute the plan after search",        action="store_true")
    argParser.add_argument("-G","--goal_config_pairs",        help = "goal config pairs",                    nargs='*')
    argParser.add_argument("-a","--axis_for_offset",          help = "axis for offset",                      default="x")
    argParser.add_argument("-R","--random_env_setup",         help = "set a random goal state",              nargs="?",default=-1)
    argParser.add_argument("-F","--ff",                       help = "use FF planner",                       action="store_true")
    argParser.add_argument("-D","--fd",                       help = "use FD planner",                       action="store_true")
    argParser.add_argument("-K","--kp",                       help = "use top_K planner",                    nargs="?",default=None)
    argParser.add_argument("-P","--po",                       help = "use partial_ordering",                 nargs="?",default=None)
    argParser.add_argument("-u","--use_problem_file",         help = "use_problem_file",                     action="store_true")
    argParser.add_argument("-U","--use_plan_file",            help = "use_plan_file",                        action="store_true")
    argParser.add_argument(     "--keep_plans",               help = "keep_plans files",                     action="store_true")
    argParser.add_argument(     "--num_robots",               help = "number of robots to use",              nargs="?",default=1)
    argParser.add_argument(     "--false_create_domain_file", help = "don't create domain file",             action="store_false")
    argParser.add_argument(     "--motion_plan",              help = "compute motionplan",                   action="store_true")
    argParser.add_argument(     "--no_sim",                   help = "not to use simulator",                 action="store_false")
    argParser.add_argument(     "--no_replan",                help = "not to replan after learning",         action="store_false")
    argParser.add_argument("-o","--order_plank_placement",    help = "set to remove random order of planks", action="store_false")
    argParser.add_argument(     "--process_count",            help = "trajectories to process per env",      nargs='?',default=1000000000)
    argParser.add_argument(     "--model_num",                help = "model number to try first",            nargs='?',default=0)
    argParser.add_argument(     "--problem_num",              help = "problem number to try first",          nargs='?',default=0)
    argParser.add_argument("-m","--model_update_flag",        help = "update model as it goes",              action="store_false")
    argParser.add_argument(     "--object_names",             help = "name of objects to manipulate",        nargs='*')
    argParser.add_argument(     "--experiment_flag",          help = "flag for experiments",                 action="store_true")
    argParser.add_argument(     "--real_world_flag",          help = "flag for real_world experiments",      action="store_true")
    argParser.add_argument("-c","--total_demo_count",         help = "total num of demonstrations",          nargs="?")
    argParser.add_argument(     "--false_load_arguments",     help = "don't load arguments",                 action="store_false")
    argParser.add_argument(     "--seed",                     help = "seed to fix",                          nargs='?',default=0)
    argParser.add_argument("-d","--domain",                    help = "name of domain",                        )
    
    args = argParser.parse_args()
    domain = args.domain
    Config.declare_config(domain)
    from src.transition_graph.LAMP import LAMP
    
    env_list = args.name
    test_structure = args.test_structure_name
    plank_count = args.plank_count
    visualize = args.visualize
    get_graph = args.get_graph
    execute = args.execute
    object_list = Config.OBJECT_NAME
    axis = args.axis_for_offset
    if axis is not None:
        axis = axis[0]
    R = args.random_env_setup
    FD = args.fd
    FF = args.ff
    KP = args.kp
    PO = args.po
    u = args.use_problem_file
    planks_in_init_state = args.planks_in_init_state
    goal_config_pairs = args.goal_config_pairs
    get_domain_flag = args.false_create_domain_file
    num_robots = (int(args.num_robots))
    use_plan_file = args.use_plan_file
    keep_plans = args.keep_plans
    mp_flag = args.motion_plan
    sim_use = args.no_sim
    replan = args.no_replan
    order = args.order_plank_placement
    process_count = int(args.process_count)
    model_num = int(args.model_num)
    model_update_flag = args.model_update_flag
    problem_num = int(args.problem_num)
    experiment_flag = args.experiment_flag
    real_world_flag = args.real_world_flag
    seed = int(args.seed)
    
    load_arguments = args.false_load_arguments

    np.random.seed(seed)
    # np.random.seed(11311)

    if R is not None:
        if plank_count is None:
            raise Exception("No Plank Count Given")
        R = int(R)
    else:
        R = 0

    if planks_in_init_state is None:
        planks_in_init_state = 0
    else:
        planks_in_init_state = int(planks_in_init_state)

    k_plans = None
    planner = None
    if FF:
        planner="FF"
    elif FD:
        planner="FD"
    elif KP is not None:
        planner = "KP"
        k_plans = int(KP)
    elif PO is not None:
        planner = "PO"
        k_plans = int(PO)

    if test_structure is not None and test_structure != "":
        test_structure = test_structure[0]
    
    if plank_count is not None:
        plank_count = int(plank_count)
    
    robot = Config.ROBOT_NAME
    
    goal_pairs = None
    if goal_config_pairs is not None:
        goal_pairs = []
        for i in list(range(len(goal_config_pairs)))[::2]:
            object_name = goal_config_pairs[i]
            surface_name = goal_config_pairs[i+1]
            goal_pairs.append([object_name,surface_name])

    if load_arguments:
        thismodule = sys.modules[__name__]
        argument_dict = get_argument_dict(problem_name=test_structure)
        for key in argument_dict.keys():
            setattr(thismodule, key, argument_dict[key])

    if args.total_demo_count is not None:
        total_demo_count = int(args.total_demo_count.split(".")[0])

    if planner is None:
        raise ValueError("No Planner Specified")
                
    tg = LAMP(robot_name=robot,
               env_list=env_list,
               test_structure=test_structure,
               test_env_name=env_list[0],
               visualize=visualize,
               axis_for_offset=axis,
               plank_count=plank_count,
               random=R,
               planner=planner,
               use_problem_file=u,
               planks_in_init_state=planks_in_init_state,
               k_plans=k_plans,
               num_robots=num_robots,
               use_plan_file=use_plan_file,
               keep_plans=keep_plans,
               mp=mp_flag,
               sim_use=sim_use,
               replan=replan,
               order=order,
               process_count=process_count,
               object_list=object_list,
               experiment_flag=experiment_flag,
               real_world_experiment=real_world_flag,
               total_demo_count=total_demo_count,
               seed=seed)
    
    tg.testing(execute=execute,
               generate=get_graph,
               config=goal_pairs,
               get_domain_flag=get_domain_flag,
               model_num=model_num,
               model_update_flag=model_update_flag,
               seed_prefix=problem_num,
               seed=seed)