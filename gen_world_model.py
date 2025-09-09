import os
import sys
import argparse
from Config import Config
from useful_functions import get_argument_dict

MOD_NAME = os.path.basename(__file__)[:-3]

if __name__ == '__main__':
    argParser = argparse.ArgumentParser()

    argParser.add_argument("-n","--name",                     help = "name of env",                          nargs='*')            	
    argParser.add_argument("-d","--domain",                    help = "name of domain",                        )
    argParser.add_argument(     "--prefix",                     help = "prefix to use",                          nargs='?',default=0)
    argParser.add_argument("--total_traj_count",         help = "total num of demonstrations",          nargs="?")
    argParser.add_argument("-f","--force",                     help = "force_generation of model",           action="store_false")
    argParser.add_argument("-v","--visualize",                help = "visualize the learnt pddl domain file",                    action="store_true")
                                                                                                                                    
    args, unknown_args = argParser.parse_known_args()

    domain = args.domain
    Config.declare_config(domain)

    from src.data_structures.WorldModel import WorldModel

    force = not (args.force)
    prefix = int(args.prefix)
    env_list = args.name
    visualize = args.visualize
    total_traj_count = args.total_traj_count
                                                                    
    thismodule = sys.modules[__name__]                      
    argument_dict = get_argument_dict(module_name=MOD_NAME)
    for key in argument_dict.keys():
        setattr(thismodule, key, argument_dict[key])        

    if args.total_traj_count is not None:
        total_traj_count = int(args.total_traj_count.split(".")[0])

    file_prefix = "{}_{}_".format(total_traj_count,prefix)                                                                    

    WorldModel.set_params(file_prefix,force)
    model = WorldModel.get_model()
    pddl = model.get_domain_pddl() 

    if visualize:
        print(pddl)

    print("Saving Model")
    WorldModel.save_model(model,0)
