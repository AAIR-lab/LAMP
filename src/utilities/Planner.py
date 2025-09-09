import os
import sys
import time
from subprocess import Popen

def get_parent_with_file(file_name):
    current_dir = os.getcwd()
    while True:
        if file_name in os.listdir(current_dir):
            return current_dir
        else:
            current_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
    
    raise Exception("File Not Found in any Parent")

ROOT_DIR = get_parent_with_file("Config.py")
if ROOT_DIR is not None and ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

from Config import Config
from src.data_structures.PDDLTrajectory import PDDLTrajectory
from src.utilities.TrajAbstracter import TrajAbstracter
import useful_functions

class Planner():
    id = "1"
    file_prefix = ""
    planner = ""
    solution_dir = ""
    file_template = Config.KP_SOLUTION_NAME
    start_i = 0
    problem_name = ""
    prefix = 0
    success_flag = False
    generator = None
    
    ff_start_time = None
    process = None
    
    @staticmethod
    def init_planner(id,file_prefix,problem_name,prefix):
        Planner.id = id
        Planner.file_prefix = file_prefix
        Planner.problem_name = problem_name
        Planner.prefix = prefix
    
    @staticmethod
    def get_num_plans_found(solution_dir):
        files = os.listdir(solution_dir)
        plan_count = 0

        for f in files:
            if Planner.file_template in f:
                plan_count += 1
        
        return plan_count

    @staticmethod
    def init_planning(start_i,end_i,planner,num_plans,test_structure,object_count):
        print("planning")
        Planner.planner = planner

        domain_file_dir = Config.PDDL_DIR+Planner.id+Planner.file_prefix+"{}_{}_domain.pddl".format(test_structure,object_count)
        problem_file = Config.PDDL_DIR+Planner.id+Planner.file_prefix+Planner.problem_name+".pddl"
        std_out_file = Config.PDDL_DIR+Planner.id+Planner.file_prefix+"prefix_{}_".format(Planner.prefix)+Planner.problem_name+"_planner_out.txt"

        solution_dir = getattr(Config,"{}_SOLUTION_DIR".format(planner)) + "{}traj_count/".format(Planner.file_prefix)
        if not os.path.exists(solution_dir):
            os.makedirs(solution_dir)
            
        cd_flag = False
        curdir = None
        
        if planner=="FD":
            success_string = "Solution found."
            cd_flag = True
            curdir = os.getcwd()
            os.chdir(solution_dir)
            # command = "python {} {} {} --search \"lazy_greedy([ff()], preferred=[ff()])\"".format(Config.FD_FILE, domain_file_dir, problem_file)
            command = "python {} {} {} --evaluator \"hff=ff()\" --search \"lazy_greedy([hff], preferred=[hff])\"".format(Config.FD_FILE, domain_file_dir, problem_file)
           # command = "python {} --alias seq-sat-fd-autotune-2 {} {}".format(Config.FD_FILE, domain_file_dir, problem_file)
            #command = "python {} --alias seq-sat-lama-2011 {} {}".format(Config.FD_FILE, domain_file_dir, problem_file)

        elif planner=="FF":
            success_string = "found legal plan as follows"
            command = "{} -o {} -f {}".format(Config.FF_FILE, domain_file_dir, problem_file)
        
        elif planner=="KP":
            success_string = "Solutions found." 
            cd_flag = True
            curdir = os.getcwd()
            os.chdir(solution_dir)
            # command = "{} {} {} --search \"symq-bd(simple=true,plan_selection=top_k(Planner.num_plans={},dump_plans=false),quality=1.0)\"".format(Config.KP_FILE,domain_file_dir,problem_file,Planner.num_plans)
            command = "{} {} {} --search \'kstar(celmcut(),k={},q=1.0)\'".format(Config.KP_FILE,domain_file_dir,problem_file,num_plans)

        Planner.process = Popen(command,shell=True,stdout=open(std_out_file,"w"))
            
        if cd_flag: 
            os.chdir(curdir)

        Planner.generator = Planner.plan_generator(planner=planner,
                                                   start_i=start_i,
                                                   end_i=end_i,
                                                   std_out_file=std_out_file)

        return True

    @staticmethod
    def plan_generator(planner,start_i,end_i,std_out_file,use_plan_file=False,keep_plans=False):
        i = start_i 
        done_flag = False
        if use_plan_file:
            done_flag = True
        while True: 
            time.sleep(2)
            if i > end_i:
                raise StopIteration

            if not use_plan_file:
                poll_code = Planner.process.poll()
            else:
                poll_code = 0
            
            if planner == "FF" and time.time()-Planner.ff_start_time >= Config.PLANNER_TIME_OUT and poll_code is None:
                Planner.process.kill()
                print("FF Planner Killed")
                yield [[]]

            if poll_code is None or done_flag: 
                print(poll_code,done_flag)
                while True and (poll_code is None or done_flag):
                    if planner == "KP":
                        solution_dir=Config.KP_SOLUTION_DIR+"{}traj_count/found_plans/".format(Planner.file_prefix)
                        plan_file = solution_dir+Config.KP_SOLUTION_NAME+".{}".format(i)
                    
                    elif planner == "FF":
                        plan_file = Config.FF_SOLUTION_DIR+Planner.id+Planner.file_prefix+Planner.problem_name+".pddl.soln"

                    time.sleep(1)
                    if os.path.isfile(plan_file): 
                        plan = []
                        if planner == "KP":
                            Planner.success_flag = True
                            print("plan {} found".format(i))
                            with open(plan_file,"r") as f:
                                solution_file = f.readlines()
                                f.close()
                            for l in solution_file[:-1]:
                                plan.append(l.strip("\n"))
                            end_i = Planner.get_num_plans_found(solution_dir=solution_dir)
                        
                        elif planner == "FF":
                            success_string = "found legal plan as follows"
                            if not use_plan_file:
                                if os.path.isfile(std_out_file):
                                    with open(std_out_file,"r") as f:
                                        std_str = f.read()
                                        f.close()                            
                                    os.remove(std_out_file)
                                    if success_string in std_str:
                                        Planner.success_flag = True
                                        print("plan {} found".format(i))
                                        with open(plan_file,"r") as f: 
                                            solution_file = f.readlines()
                                            f.close()
                                        for l in solution_file[1:]:
                                            plan.append(l.strip("\n"))
                                    else:
                                        Planner.success_flag = False
                            else:
                                print("plan {} found".format(i))
                                with open(plan_file,"r") as f: 
                                    solution_file = f.readlines()
                                    f.close()
                                for l in solution_file[1:]:
                                    plan.append(l.strip("\n"))
                        
                        if not keep_plans:
                            os.remove(plan_file)
                        i += 1
                        break
                    
                    elif (poll_code is not None and poll_code < 0) or done_flag:
                        print(poll_code)
                        raise StopIteration

                    elif poll_code == 0:
                        done_flag = True
                        time.sleep(1)
                    
                    elif poll_code >= 0:
                        done_flag = True
                        time.sleep(1)

                    else:
                        time.sleep(10)
                        if not use_plan_file:
                            poll_code = Planner.process.poll()
                        else:
                            poll_code = 0
                        if poll_code is not None:
                            done_flag = True
                            time.sleep(1)
                        
                        if planner == "FF" and time.time()-Planner.ff_start_time >= Config.PLANNER_TIME_OUT and poll_code is None:
                            Planner.process.kill()
                            print("FF Planner Killed")
                            yield [[]]
                
                yield plan
            
            elif poll_code is not None: 
                if poll_code < 0: 
                    raise StopIteration
                else:
                    done_flag = True
                    time.sleep(0.1)  