from src.data_gen.DataAugmentation import DataAugmenter
import argparse
import Config
import sys
import json
import os

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
    argParser.add_argument("-n","--name",help = "name of env",nargs='*')
    argParser.add_argument("-k","--object_pair_key", help = "object pairs needed for data", nargs='*')
    argParser.add_argument(     "--false_load_arguments",     help = "don't load arguments",                 action="store_false")
    argParser.add_argument("-d","--domain",                    help = "name of domain",                        )

    args = argParser.parse_args()
    domain = args.domain
    Config.declare_config(domain)
    
    env_list = args.name
    key_string_set = args.object_pair_key
    load_arguments = args.false_load_arguments
    
    if load_arguments:
        thismodule = sys.modules[__name__]
        argument_dict = get_argument_dict()
        for key in argument_dict.keys():
            setattr(thismodule, key, argument_dict[key])

    if key_string_set is not None:
        obj1 = key_string_set[0]
        obj2 = key_string_set[1]

        key_string_set = set([obj1,obj2])
        
    for name in env_list:         
        da = DataAugmenter(name)
        da.augment_data(da.data,key_string_set)