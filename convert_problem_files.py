import os
from Config import ROOT_DIR
import argparse
import cPickle
import pickle
from src.data_structures import EnvState

PROB_PATH = ROOT_DIR+"problem_sets/"
NEW_PROB_PATH = ROOT_DIR+"converted_problem_sets/"

if __name__=="__main__":
    argParser = argparse.ArgumentParser() 
    argParser.add_argument("-d","--domain_name",help = "name of domain",nargs='*')

    args = argParser.parse_args()
    domain_names = args.domain_name
    if type(domain_names) != list:
        domain_names = [domain_names]

    for domain in domain_names:
        domain_path = PROB_PATH+domain+"/"
        files = os.listdir(domain_path)
        
        for f_name in files:
            with open(domain_path+f_name,"r") as f:
                d = cPickle.load(f)
                f.close()

            new_d = {}
            if "Keva" not in domain:
                new_d["num_objects"] = d["objects_to_move"]
            else:
                new_d["num_objects"] = len([o for o in d["init_state"].object_dict.keys() if "plank" in o])

            print(f_name, new_d["num_objects"])

            if not os.path.exists(NEW_PROB_PATH+domain):
                os.makedirs(NEW_PROB_PATH+domain)
            
            with open(NEW_PROB_PATH+domain+"/"+f_name,"w") as f:
                pickle.dump(new_d,f)
                f.close()