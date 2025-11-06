import os
from Config import Config
import argparse
# import cPickle
import pickle

PICKLE_FILE_DIRECTORIES = [
    # Config.ROOT_DIR + "Data",
    Config.ROOT_DIR + "problem_sets"
]

CPICKLE_SUFFIX = ".p"
PICKLE_SUFFIX = ".pkl"
PICKLE_PROTOCOL = 2

if __name__=="__main__":
    argParser = argparse.ArgumentParser()
    argParser.add_argument("--filter", help = "name of test structure", default="")
    argParser.add_argument("--reverse",help = "from pickle to cPickle",action="store_true")

    args, unknown_args = argParser.parse_known_args()
    filt = args.filter
    reverse = args.reverse

    if reverse:
        PRE_SUFFIX = PICKLE_SUFFIX
        POST_SUFFIX = CPICKLE_SUFFIX
        PRE_PICKLE = pickle
        POST_PICKLE = pickle

    else:
        PRE_SUFFIX = CPICKLE_SUFFIX
        POST_SUFFIX = PICKLE_SUFFIX
        PRE_PICKLE = pickle
        POST_PICKLE = pickle

    pickle_files = []
    for path in PICKLE_FILE_DIRECTORIES:
        for root, _, f in os.walk(path):
            pickle_files.extend([root+ "/" + _ for _ in f if _.endswith(PRE_SUFFIX) and filt in "{}/{}".format(root,_)])

    for pf in pickle_files:
        with open(pf,"rb") as f:
            if reverse:
                data = PRE_PICKLE.load(f,encoding="latin1")
            else:
                data = PRE_PICKLE.load(f)
            f.close()

        with open(pf[:-len(PRE_SUFFIX)]+POST_SUFFIX,"wb") as f:
            POST_PICKLE.dump(data,f,protocol=Config.PICKLE_PROTOCOL)
            f.close()

        print(pf.split("/")[-1] + " converted")