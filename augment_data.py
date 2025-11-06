import argparse
from Config import Config
import sys
import json
import os
from useful_functions import get_argument_dict

from src.utilities.DataAugmentation import DataAugmenter

MOD_NAME = os.path.basename(__file__)[:-3]

if __name__ == "__main__":

    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n","--name",help = "name of env",nargs='*')
    argParser.add_argument("-k","--object_pair_key", help = "object pairs needed for data", nargs='*')
    argParser.add_argument(     "--false_load_arguments",     help = "don't load arguments",                 action="store_false")
    argParser.add_argument("-d","--domain",                    help = "name of domain",                        )

    args, unknown_args = argParser.parse_known_args()
    domain = args.domain
    Config.declare_config(domain)
    
    env_list = args.name
    key_string_set = args.object_pair_key
    load_arguments = args.false_load_arguments

    if load_arguments:
        thismodule = sys.modules[__name__]
        argument_dict = get_argument_dict(module_name=MOD_NAME)
        for key in argument_dict.keys():
            setattr(thismodule, key, argument_dict[key])
    
    if args.name is not None:
        env_list = args.name

    if key_string_set is not None:
        key_string_set = set(key_string_set)
    else:
        key_string_set = set([])
        
    for name in env_list:         
        da = DataAugmenter(name,key_string_set=key_string_set)
        da.augment_data(da.data)