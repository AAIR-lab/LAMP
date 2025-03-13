import sys
import cPickle
from params import DOMAIN_OBJECTS, DOMAIN_NAME
from utils import *

env_object_list = DOMAIN_OBJECTS[DOMAIN_NAME]
object_dictionary = get_object_dictionary(env_object_list)

def sample_from_grounded_relation(grounded_relation):
    if len(grounded_relation.region) == 1:
        region_to_sample_from = grounded_relation.region[0][:-1]
    else:
        indice = np.random.choice(len(grounded_relation.region))
        region_to_sample_from = grounded_relation.region[indice][:-1]

    return grounded_relation.discretizer.convert_sample(region_to_sample_from,is_relative=True)

def save_lifted_relations(lifted_relations_dict):
    with open(DATA_DIR+"lifted_relations.p","w") as f:
        cPickle.dump(lifted_relations_dict,f,protocol=cPickle.HIGHEST_PROTOCOL)
        f.close()

def load_lifted_relations():
    try:
        with open(DATA_DIR+"lifted_relations.p","r") as f:
            data_dict = cPickle.load(f)
            f.close()
    except:
        rcr_data = load_rcrs()
        data_dict = get_lifted_relations_dict(env_object_list,rcr_data)

    return data_dict

if __name__ == "__main__":
    lifted_relations_dict = load_lifted_relations()
    grounded_relations_dict = get_grounded_relations_dict(lifted_relations_dict,object_dictionary)

    #for Keva domain, sample from plank_plank_1 grounded_relation
    cr = 1 #id for the critical region.
    object_pair = "plank_plank" #object type pair for which the critical region was identified and learned.
    grounded_relation = grounded_relations_dict[object_pair][cr][0] #first grounded relation in list

    print(sample_from_grounded_relation(grounded_relation))