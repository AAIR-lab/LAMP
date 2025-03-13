from src.transition_graph.debug_LAMP import LAMP
import numpy as np
import argparse
import random

random.seed(10)

if __name__ == '__main__':

    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n","--name",                help = "name of env",                  nargs='*')
    argParser.add_argument("-t","--test_structure_name", help = "name of test structure",       nargs='*')
    argParser.add_argument("-r","--robot",               help = "name of robot",                nargs='*')
    argParser.add_argument("-v","--visualize",           help = "visualize the sim",            action="store_true")
    argParser.add_argument("-p","--plank_count",         help = "plank count for exp",          nargs="?",default=1)
    argParser.add_argument("-c","--planks_in_init_state",help = "planks_in_init_state",         nargs="?",default=0)
    argParser.add_argument("-g","--get_graph",           help = "get pruned or complete graph", action="store_true")
    argParser.add_argument("-e","--execute",             help = "execute the plan after search",action="store_true")
    argParser.add_argument("-G","--goal_config_pairs",   help = "axis for offset",              nargs='*')
    argParser.add_argument("-a","--axis_for_offset",     help = "goal config pairs",            nargs='*')
    argParser.add_argument("-R","--random_env_setup",    help = "set a random goal state",      action="store_true")
    argParser.add_argument("-F","--ff",                  help = "use FF planner",               action="store_true")
    argParser.add_argument("-D","--fd",                  help = "use FD planner",               action="store_true")
    argParser.add_argument("-K","--kp",                  help = "use top_K planner",            nargs="?",default=None)
    argParser.add_argument("-u","--use_problem_file",    help = "use_problem_file",             action="store_true")
    argParser.add_argument("-U","--use_plan_file",       help = "use_plan_file",                action="store_true")
    argParser.add_argument(     "--keep_plans",          help = "keep_plans files",             action="store_true")
    argParser.add_argument(     "--num_robots",          help = "number of robots to use",      nargs="?",default=1)
    argParser.add_argument(     "--create_domain_file",  help = "create domain file",           action="store_true")
    argParser.add_argument(     "--no_motion_plan",      help = "not to compute motionplan",    action="store_false")
    argParser.add_argument(     "--no_sim",              help = "not to use simulator",         action="store_false")

    args = argParser.parse_args()
    env_list = args.name
    test_structure = args.test_structure_name
    plank_count = args.plank_count
    visualize = args.visualize
    get_graph = args.get_graph
    execute = args.execute
    axis = args.axis_for_offset[0]
    R = args.random_env_setup
    FD = args.fd
    FF = args.ff
    KP = args.kp
    u = args.use_problem_file
    planks_in_init_state = args.planks_in_init_state
    goal_config_pairs = args.goal_config_pairs
    get_domain_flag = args.create_domain_file
    num_robots = (int(args.num_robots))
    use_plan_file = args.use_plan_file
    keep_plans = args.keep_plans
    mp_flag = args.no_motion_plan
    sim_use = args.no_sim

    if planks_in_init_state is None:
        planks_in_init_state = 0
    else:
        planks_in_init_state = int(planks_in_init_state)

    k_plans = None
    if FF:
        planner="FF"
    elif FD:
        planner="FD"
    elif KP is not None:
        planner = "KP"
        k_plans = int(KP)
    else:
        raise ValueError("No Planner Specified")

    if test_structure is not None:
        test_structure = test_structure[0]
    
    if plank_count is not None:
        plank_count = int(plank_count)
    
    if args.robot is not None:
        robot = args.robot[0]
    
    goal_pairs = []
    if goal_config_pairs is not None:
        for i in list(range(len(goal_config_pairs)))[::2]:
            object_name = goal_config_pairs[i]
            surface_name = goal_config_pairs[i+1]
            goal_pairs.append([object_name,surface_name])
    
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
               sim_use=sim_use
               )
    
    tg.testing(execute=execute,generate=get_graph,config=goal_pairs,get_domain_flag=get_domain_flag)