import numpy as np
import argparse
import Config
import csv
import json
import os 
import time

def get_log_path():
    path = os.path.dirname(os.path.abspath(__file__)) 
    parent = os.path.split(path)[0] + "/"

    return parent

def log(log_dict,log_num,seed,demonstration_count,key=None,seed2=0):
    # print(log_dict)
    log_file_path = get_log_path()
    file_name = "{}_logs/seed_{}_{}_order_{}_log_{}.json".format(Config.DOMAIN_NAME,seed,demonstration_count,seed2,log_num)

    try:
        with open(log_file_path+file_name,"r") as f:
            data = json.load(f)
            f.close()
        if key is None:
            data[str(len(data.keys())+1)] = log_dict      
        else:
            data[str(key)] = log_dict
    except:
        if key is None:
            data = {str(1): log_dict}
        else:
            data = {str(key): log_dict}
    
    dump_json_object = json.dumps(data)
    with open(log_file_path+file_name,"w") as f:
        f.write(dump_json_object)
        f.close()
    
if __name__ == '__main__':

    argParser = argparse.ArgumentParser()        
    argParser.add_argument("-n","--name",                 help = "name of env",                          nargs='*')
    argParser.add_argument("-t","--test_structure_name",  help = "name of test structure",               nargs='*')
    argParser.add_argument("-r","--robot",                help = "name of robot",                        nargs='*')
    argParser.add_argument("-v","--visualize",            help = "visualize the sim",                    action="store_true")
    argParser.add_argument("-p","--plank_count",          help = "plank count for exp",                  nargs="?",default=1)
    argParser.add_argument("-c","--planks_in_init_state", help = "planks_in_init_state",                 nargs="?",default=0)
    argParser.add_argument("-g","--get_graph",            help = "get pruned or complete graph",         action="store_true")
    argParser.add_argument("-e","--execute",              help = "execute the plan after search",        action="store_true")
    argParser.add_argument("-G","--goal_config_pairs",    help = "axis for offset",                      nargs='*')
    argParser.add_argument("-a","--axis_for_offset",      help = "goal config pairs",                    nargs='*')
    argParser.add_argument("-R","--random_env_setup",     help = "set a random goal state",              nargs="?",default=-1)
    argParser.add_argument("-F","--ff",                   help = "use FF planner",                       action="store_true")
    argParser.add_argument("-D","--fd",                   help = "use FD planner",                       action="store_true")
    argParser.add_argument("-K","--kp",                   help = "use top_K planner",                    nargs="?",default=None)
    argParser.add_argument("-P","--po",                   help = "use partial_ordering",                 nargs="?",default=None)
    argParser.add_argument("-u","--use_problem_file",     help = "use_problem_file",                     action="store_true")
    argParser.add_argument("-U","--use_plan_file",        help = "use_plan_file",                        action="store_true")
    argParser.add_argument(     "--keep_plans",           help = "keep_plans files",                     action="store_true")
    argParser.add_argument(     "--num_robots",           help = "number of robots to use",              nargs="?",default=1)
    argParser.add_argument(     "--create_domain_file",   help = "create domain file",                   action="store_true")
    argParser.add_argument(     "--no_motion_plan",       help = "not to compute motionplan",            action="store_false")
    argParser.add_argument(     "--no_sim",               help = "not to use simulator",                 action="store_false")
    argParser.add_argument(     "--no_replan",            help = "not to replan after learning",         action="store_false")
    argParser.add_argument(     "--num_episodes",         help = "no. of episodes to run",               nargs="?",default=1)
    argParser.add_argument("-o","--order_plank_placement",help = "set to remove random order of planks", action="store_true")
    argParser.add_argument(     "--process_count",        help = "trajectories to process per env",      nargs='?')
    argParser.add_argument(     "--model_num",            help = "model number to try first",            nargs='?',default=0)
    argParser.add_argument("-m","--model_update_flag",    help = "update model as it goes",              action="store_true")
    argParser.add_argument(     "--experiment_flag",      help = "flag for experiments",                 action="store_true")
    argParser.add_argument(     "--problem_num",          help = "problem number to try first",          nargs='?',default=0)
    argParser.add_argument(     "--seed2",                help = "seed to fix random problem ordering",  nargs='?',default=0)
    argParser.add_argument(     "--seed",                 help = "seed to fix",                          nargs='?',default=0)
    argParser.add_argument(     "--total_demo_count",     help = "total num of demonstrations",          nargs="?")
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
    axis = args.axis_for_offset[0]
    R = args.random_env_setup
    FD = args.fd
    FF = args.ff
    KP = args.kp
    PO = args.po
    u = args.use_problem_file
    planks_in_init_state = args.planks_in_init_state
    goal_config_pairs = args.goal_config_pairs
    get_domain_flag = args.create_domain_file
    num_robots = (int(args.num_robots))
    use_plan_file = args.use_plan_file
    keep_plans = args.keep_plans
    mp_flag = args.no_motion_plan
    sim_use = args.no_sim
    replan = args.no_replan
    order = args.order_plank_placement
    num_episodes = int(args.num_episodes)
    process_count = args.process_count
    model_num = int(args.model_num)
    model_update_flag = args.model_update_flag
    problem_num = int(args.problem_num)
    experiment_flag = args.experiment_flag
    seed = int(args.seed)
    seed2 = int(args.seed2)
    total_demo_count = int(args.total_demo_count)

    np.random.seed(seed)
    print("SEED ---> {}".format(seed))
    
    if R is not None:
        if plank_count is None:
            raise Exception("No Plank Count Given")
        R = int(R)
    else:
        R = 0

    if process_count is None:
        raise ValueError("NO PROCESS COUNT GIVEN")
    else:
        process_count = int(process_count)

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
    elif PO is not None:
        planner = "PO"
        k_plans = int(PO)
    else:
        raise ValueError("No Planner Specified")

    if test_structure is not None:
        test_structure = test_structure[0]
    
    if plank_count is not None:
        plank_count = int(plank_count)
    
    if args.robot is not None:
        robot = args.robot[0]
    
    goal_pairs = None
    if goal_config_pairs is not None:
        goal_pairs = []
        for i in list(range(len(goal_config_pairs)))[::2]:
            object_name = goal_config_pairs[i]
            surface_name = goal_config_pairs[i+1]
            goal_pairs.append([object_name,surface_name])
    
    log_num = int(env_list[0][3:])
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
            experiment_flag=experiment_flag,
            total_demo_count=total_demo_count,
            seed=seed)
    
    success_bool, kplans, plan_lengths, learnt_actions, learnt_relations, traj_config, time_dict, latest_model_num = tg.testing(execute=execute,
                                                                                                                                generate=get_graph,
                                                                                                                                config=goal_pairs,
                                                                                                                                get_domain_flag=get_domain_flag,
                                                                                                                                model_num=model_num,
                                                                                                                                model_update_flag=model_update_flag,
                                                                                                                                seed_prefix=problem_num,
                                                                                                                                experiment_flag=experiment_flag,
                                                                                                                                seed=seed)
    log_dict = {
        "successful_run":success_bool,
        "kth_successful_plan": kplans,
        "plan_lengths": plan_lengths,
        "num_learnt_actions": learnt_actions,
        "learnt_relations": learnt_relations,
        "object_count": plank_count, 
        "object_in_init_state": planks_in_init_state,
        "robot_used": robot,
        "end_time": time.time(),
    }
            
    if planner == "KP" or planner == "PO":
        log_dict["k_used in k planning"] = k_plans
    else:
        log_dict["k_used in k planning"] = 1
    
    if "Keva" in Config.DOMAIN_NAME:
        log_dict["goal_config"] = test_structure
    elif "CafeWorld" == Config.DOMAIN_NAME:
        structure_string = ""
        for count,config in enumerate(traj_config):
            can,surface1 = config[0]
            _,surface2 = config[1]
            structure_string += "{}-{}-{}".format(can,surface1,surface2)
            if count < len(traj_config)-1:
                structure_string+= ", "

        log_dict["goal_config"] = structure_string
    else:
        log_dict["goal_config"] = "Cans in Box"
    
    log_dict["num demonstrations"] = total_demo_count

    for key in time_dict:
        log_dict[key] = time_dict[key]
    
    log(log_dict,log_num,seed=seed,key=problem_num,demonstration_count=total_demo_count,seed2=seed2)