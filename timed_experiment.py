import subprocess
import Config
import time 
import argparse
import json
import os
import numpy as np
import copy

def get_log_path():
    path = os.path.dirname(os.path.abspath(__file__)) 
    parent = os.path.split(path)[0] + "/"

    return parent

def log(log_dict,log_num,seed,demonstration_count,key=None,seed2=0):
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

def start_process(command_list,file_suffix): 
    if not os.path.exists(Config.STD_OUT_DIR):
        os.makedirs(Config.STD_OUT_DIR)
    p = subprocess.Popen(command_list, stdout = open(Config.STD_OUT_DIR + "log{}.log".format(file_suffix),"w"), stderr = open(Config.STD_OUT_DIR + "err{}.err".format(file_suffix),"w"))
    return p 

def get_latest_model_num(prefix,domain):
    file_name_template = "{}_{}_".format(prefix,domain)

    # path = Config.DATA_MISC_DIR
    path = Config.DATA_MISC_DIR
    files = os.listdir(path)

    model_files_count = 0
    for f in files:
        if file_name_template in f:
            model_files_count += 1
    
    return model_files_count - 1

argParser = argparse.ArgumentParser()        
    
argParser.add_argument("-n","--name",                 help = "name of env",                          nargs='*')
argParser.add_argument("-r","--robot",                help = "name of robot",                        nargs='*')
argParser.add_argument(     "--total_demo_count",     help = "total num of demonstrations",          nargs="?")
argParser.add_argument(     "--seed",                 help = "seed to fix",                          nargs='?',default=0)
argParser.add_argument(     "--seed2",                help = "seed to fix random problem ordering",  nargs='?',default=0)
argParser.add_argument(     "--problem_num",          help = "problem number to try first",          nargs='?',default=0)
argParser.add_argument("-d","--domain",                    help = "name of domain",                        )

args = argParser.parse_args()
domain = args.domain
Config.declare_config(domain)

name = "env{}".format(args.name[0])
robot = Config.ROBOT_NAME
log_num = int(args.name[0])
num_robots = 1
total_demo_count = int(args.total_demo_count)
seed = int(args.seed)
seed2 = int(args.seed2)
robot = args.robot[0]
axis = "x"
process_count = 0
problems_completed = 0
test_structure = "test_structure_placeholder"

latest_model_num = 0
problem_dict = {
    "Keva": ["pi_tower;9;0;x","tripple_pi;6;0;x","tripple_pi;9;0;x","flat_tower;24;0;y","keva_three_tower;24;0;y","flat_3_tower;12;0;y","2d_house;6;0;y","twin_stacked_pi;30;0;y","random;5;0;x","stonehenge;18;0;x","flat_tower;6;0;y","fort_curricullum;6;0;y","2_pi;6;0;x"],
    "CafeWorld": ["can_surface;8;0;x" for _ in range(10)] + ["can_surface;16;0;x","can_surface;18;0;x","can_surface;20;0;x"],
    "Packing": ["can_surface;4;0;x" for _ in range(13)]
} 

problem_set = problem_dict[Config.DOMAIN_NAME]

object_count_list = []
for prob in problem_set:
    _,obj_count,_,_ = prob.split(";")
    object_count_list.append(int(obj_count))

if args.problem_num is not None:
    problem_num = args.problem_num
else:
    problem_num = 0

problem_list = list(range(1,len(problem_set)+1))
if problem_num != 0:
    problem_list = [int(p) for p in problem_num.split(",")]
    if Config.DOMAIN_NAME != "Packing":
        object_count_list = [int(o) for _,o,_,_ in [s.split(";") for i,s in enumerate(problem_set) if int(i+1) in problem_list ]]

problem_list.sort()
if "Keva" in Config.DOMAIN_NAME:
    problem_list_temp = [x for _, x in sorted(zip(object_count_list, problem_list))]
    problem_list = copy.deepcopy(problem_list_temp)
    
while problems_completed < len(problem_list):
    i = problem_list[problems_completed]
    
    # if i == 5:
    #     break

    if i in []:
        problems_completed+=1
        print("problem {} skipped".format(i))
        continue        

    if "Keva" not in Config.DOMAIN_NAME:
        test_structure, plank_count, planks_in_init_state,axis = problem_set[i-1].split(";")
        argument_list = [
            name,
            axis,
            robot,
            plank_count,
            num_robots,
            test_structure,
            process_count,
            i,
            seed,
            total_demo_count,
            domain
        ]   
        command = "python experiments.py -n {0} -a {1} -r {2} -o -p {3} --num_robots {4} -t {5} --create_domain_file --process_count {6} --problem_num {7} --model_num 1 --no_motion_plan --experiment_flag --seed {8} --total_demo_count {9} --ff -d {10}".format(*argument_list)
    
    else:
        prefix = "{}_{}".format(total_demo_count,seed)
        latest_model_num = get_latest_model_num(prefix,domain=Config.DOMAIN_NAME)
        test_structure, plank_count, planks_in_init_state,axis = problem_set[i-1].split(";")
        if test_structure == "random":
            order_string = "-R"
        else:
            order_string = "--order_plank_placement"
        argument_list = [name,None,None,axis,robot,num_robots,None,None,test_structure,plank_count,planks_in_init_state,latest_model_num,order_string,process_count,i,seed,total_demo_count,seed2,domain]
        command = "python experiments.py -n {0} -a {3} -r {4} --num_robots {5} --kp 2000 -t {8} -p {9} -c {10} --create_domain_file -m --model_num {11} {12} --process_count {13} --problem_num {14} --experiment_flag --no_motion_plan --seed {15} --total_demo_count {16} --seed2 {17} -d {18}".format(*argument_list) 
    
    file_suffix = "order_{}_seed_{}_demo_count_{}_problem_{}".format(seed2,seed,total_demo_count,i)
    p = start_process(command.split(" "),file_suffix) 
    start_time = time.time()
    killed = False
    
    print("starting problem {} with id: {}".format(problems_completed,i))
    while p.poll() is None:
        if time.time() - start_time > 3600*3: 
            p.kill()
            killed = True

    if killed: 
        print("problem {} with id: killed".format(problems_completed,i))
        log_dict = {
            "successful_run": False,
            "kth_successful_plan": -1,
            "plan_lengths": [],
            "num_learnt_actions": [],
            "learnt_relations": [],
            "object_count": plank_count, 
            "object_in_init_state": -1,
            "robot_used": robot,
            "k_used in k planning": -1,
            "goal_config": "Cans in Box",
            "num demonstrations": total_demo_count,
            "end_time": time.time(),
        }
        
        log(log_dict,log_num,seed=seed,key=i,demonstration_count=total_demo_count,seed2=seed2)
    
    else: 
        print("exit with code {} for problem_num: {} with id: {}".format(p.poll(),problems_completed,i))

    problems_completed += 1
