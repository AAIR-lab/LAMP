import Config
import os

domains = ["Keva", "CafeWorld", "Packing"]

for d in domains:
    Config.declare_config(d)

    seed_set = set([])
    files = os.listdir(Config.DATA_MISC_DIR)
    for f in [ _ for _ in files if os.path.isfile(_)]:
        n = f.split("_")
        if [n[2], n[3]] == [d,"1"]:
            seed_set.add(int(n[1]))
        
    print("{} : NumSeeds: {} and Seeds: {}".format(d,len(seed_set),seed_set))
    