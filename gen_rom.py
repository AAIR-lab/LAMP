from src.data_gen.RelativeOccupancyMapping import RelativeOccupancyMap
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

if __name__ == '__main__':

    argParser = argparse.ArgumentParser()
    argParser.add_argument("-c","--traj_count", help = "number of trajectories per env", nargs='?')
    argParser.add_argument("-n","--name", help = "name of env", nargs='*')
    argParser.add_argument("-k","--object_pair_key", help = "object pairs needed for data", nargs='*')
    argParser.add_argument("-o","--false_overwrite_data", help = "overwrite stored data", action="store_false")
    argParser.add_argument("-p","--print_analysis", help = "whether to print the max and len(argwhere)", action="store_true")
    argParser.add_argument("-P","--plot_test", help = "plot the indices before processing", action="store_true")
    argParser.add_argument("--debug", help = "debugging flag", action="store_true")
    argParser.add_argument("-l","--false_load_all", help = "don't load all data for processing all relations together", action="store_false")
    argParser.add_argument("--num_robots",help = "number of robots",nargs='?',default=1)
    argParser.add_argument("--process_count",help = "trajectories to process per env",nargs='?',default=1000000000)
    argParser.add_argument("--total_demo_count",     help = "total num of demonstrations",          nargs="?")
    argParser.add_argument("--seed",                 help = "seed to fix",                          nargs='?',default=0)
    argParser.add_argument(     "--false_load_arguments",     help = "don't load arguments",                 action="store_false")
    argParser.add_argument("-d","--domain",                    help = "name of domain",                        )

    args = argParser.parse_args()
    domain = args.domain
    Config.declare_config(domain)
    
    seed = int(args.seed)

    np.random.seed(seed)

    env_list = args.name
    overwrite = args.false_overwrite_data
    key_string_set = args.object_pair_key
    print_analysis = args.print_analysis
    plot = args.plot_test
    debug = args.debug
    load_all = args.false_load_all
    num_robots = int(args.num_robots)
    load_arguments = args.false_load_arguments
    if args.process_count is not None:
        process_count = int(args.process_count)
    else:
        process_count = 0

    if args.total_demo_count is not None:
        total_demo_count = float(args.total_demo_count)
    else:
        total_demo_count = 0

    if key_string_set is not None:
        obj1 = key_string_set[0]
        obj2 = key_string_set[1]

        key_string_set = set([obj1,obj2])

    if load_arguments:
        thismodule = sys.modules[__name__]
        argument_dict = get_argument_dict()
        for key in argument_dict.keys():
            setattr(thismodule, key, argument_dict[key])
    
    if args.traj_count is not None:
        traj_per_env = float(args.traj_count)

    rom = RelativeOccupancyMap(env_list,
                               traj_per_env = traj_per_env,
                               overwrite=overwrite,
                               print_analysis=print_analysis,
                               plot=plot,
                               debug=debug,
                               load_all=load_all,
                               num_robots=num_robots,
                               process_count=process_count,
                               total_demo_count=total_demo_count,
                               seed=seed)
    # print("at start")
    if debug:
        import IPython;IPython.embed()
    rom.start(key_string_set=key_string_set)
    del rom
