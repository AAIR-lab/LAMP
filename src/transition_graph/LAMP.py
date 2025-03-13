from openravepy import *
from src.data_structures.discretizer import Discretizer
from Config import *
from src.data_structures.TransitionGraph import TransitionGraph 
from src.data_structures.RaveExecution import GetRaveExecution
from src.data_structures.PDDLAction import LiftedPDDLAction
from src.data_structures.PDDLPrecondition import LiftedPDDLPrecondition
from src.data_structures.Relation import Relation
from src.data_structures.ChangedRelations import ChangedRelations
from src.data_structures.ParameterizedLiftedRelation import ParameterizedLiftedRelation
from src.data_structures.Const import Const
from src.transition_graph.model_updater import ModelUpdater
from src.data_structures.Link import Link
from src.data_structures.ROSAction import ROSAction
import rospy
import shlex
# from rcr_ros.msg import Action, Plan
from time import sleep
from src.useful_functions import *
import Config
import cPickle 
import os
import tqdm
import copy
import importlib
import time
import shutil
from subprocess import Popen
import json
import heapq

class LAMP(object): 
    def __init__(self,robot_name,env_list,test_structure,test_env_name,axis_for_offset,total_demo_count,visualize=False,plank_count=1,random=False,planner="FF",use_problem_file=False,planks_in_init_state=0,k_plans=None,num_robots=1,use_plan_file=False,keep_plans=False,mp=True,sim_use=True,replan=True,order=False,process_count=100000000000,object_list=["bowl","glass"],experiment_flag=False,real_world_experiment=False,seed=0):
        self.env_list = env_list
        self.test_env_name = test_env_name

        self.total_demo_count = total_demo_count
        self.seed = seed
        self.env_prefix = "{}_".format(env_list[0][3:])
        self.file_prefix = "{}_{}_".format(total_demo_count,seed)
        self.file_name = "rcr_indices.p"
        # if "Keva" in Config.DOMAIN_NAME or "CafeWorld" in Config.DOMAIN_NAME or "Jenga" in Config.DOMAIN_NAME:
        #     self.data,self.env_traj_dict = self.load_data()
        # else:
        #     self.data = self.load_data()
        self.data,self.env_traj_dict = self.load_data()
        self.plank_count = plank_count
        self.actions = []
        self.aux_list = None
        self.bound_object_name = Config.BOUND_OBJECT_NAME
        self.transition_graph = TransitionGraph()
        self.graph_loaded = False
        self.object_dictionary = None
        self.domain = Config.DOMAIN_NAME
        self.num_robots = num_robots
        self.compute_motion_plan = mp
        self.sim_use = sim_use
        self.replan = replan
        self.process_count = process_count
        self.learnt_action_count = 0
        self.added_relations = dict()
        self.new_relations = set([])
        self.learnt_relations = set([])
        self.edited_actions_dict = {}

        self.new_goal_relations = {}
        self.problem_object_pairs = []
        self.considered_relations = {}
        self.init_env_state = None
        self.goal_env_state = None

        self.transitions_used = []
        self.goal_relations = {}
        self.goal_aux_list = []
        self.relations_from_goal = set([])
        self.relations_to_remove = []

        self.ff_start_time = 0

        if self.sim_use:
            sim_class = GetRaveExecution(domain_name=self.domain,
                                         robot_name=robot_name)
            self.sim_object = sim_class(env_name=test_env_name,
                                              visualize=visualize,
                                              plank_count=plank_count,
                                              axis_for_offset=axis_for_offset,
                                              test_structure_name=test_structure,
                                              random=random,
                                              planks_in_init_state=planks_in_init_state,
                                              num_robots=self.num_robots,
                                              order=order,
                                              object_list=object_list,
                                              experiment_flag=experiment_flag,
                                              prefixes = [self.env_prefix,self.file_prefix])
        else:
            self.sim_object = None
        
        self.best_state = None
        self.non_applicable_dictionary = {}
        self.action_refinement_dictionary = {}
        self.test_structure = test_structure
        # if "Keva" in Config.DOMAIN_NAME or "Jenga" in Config.DOMAIN_NAME:
        #     self.test_structure_dag,self.unknown_plank_relations = self.load_dag(test_structure)
        self.new_lifted_relations_dict = {}
        self.planner = planner
        self.use_problem_file = use_problem_file
        self.total_plans = k_plans
        self.partial_order_plans = k_plans
        self.failed_planks = {}
        self.use_plan_file = use_plan_file
        self.keep_plans = keep_plans
        self.relations_to_learn = set([])
        self.p = None

        self.new_ll_transitions = {}
        self.og_relations = {}

    def load_data(self):
        print("loading {} data".format(self.env_list))
        data_load = cPickle.load(open(Config.DATA_MISC_DIR+self.file_prefix+self.file_name))
        # if Config.DOMAIN_NAME in ["CafeWorld","Keva","Jenga"]:
        #     return data_load["rcr_dict"],data_load["env_traj_dict"]
        # else:
        #     return data_load
        return data_load["rcr_dict"],data_load["env_traj_dict"]
    
    def load_dag(self,structure_name,create=False,rcr_dict={}):
        try:
            with open(Config.DAG_DIR + structure_name + "_dag.p","rb") as f:
                data = cPickle.load(f)
                f.close()
        except:
            create = True
        
        if create:
            if len(rcr_dict.values()) == 0:
                with open(Config.DATA_MISC_DIR+self.file_prefix+"og_"+self.file_name,"rb") as f:
                    data = cPickle.load(f)
                    if Config.DOMAIN_NAME in ["Keva","CafeWorld","Jenga"]:
                        rcr_dict = data["rcr_dict"]
                    else:
                        rcr_dict = data
                    f.close()

            obj_name = Config.OBJECT_NAME[0]
            if obj_name == "can":
                obj_name = "plank"
            discretizer = Config.get_discretizer(obj1=obj_name,obj2=obj_name)
            data=create_dag(structure=structure_name,
                            cr_list=rcr_dict[obj_name][obj_name],
                            discretizer=discretizer)
        
        return data["dag"],data["unknown_plank_relations"]
        
    def get_lifted_relations_dict(self,env_object_list,rcrs=None):
        if rcrs is None:
            rcrs = self.data
        
        lifted_relations_dict = {}
        for link1 in rcrs.keys(): 
            for link2 in rcrs[link1].keys(): 
                link1_list = []
                link2_list = []
                for obj in env_object_list:
                    if obj.split("_")[0] == link1:
                        link1_list.append(obj)
                    if obj.split("_")[0] == link2:
                        link2_list.append(obj)
                        
                lifted_regions = rcrs[link1][link2]
                for obj1 in link1_list:
                    for obj2 in link2_list:
                        if obj2 != obj1:
                            obj1_type = obj1.split("_")[0]
                            obj2_type = obj2.split("_")[0]

                            key_string = "{}_{}".format(obj1_type,obj2_type)
                            if key_string not in lifted_relations_dict.keys():
                                lifted_relations_dict[key_string] = {}                            
                                for cr in range(len(lifted_regions)+1):
                                    obj1_type, obj2_type = key_string.split("_")
                                    if cr == 0:
                                        region = []
                                        for r in lifted_regions:
                                            region.extend(r)
                                    else:
                                        region = lifted_regions[cr-1]
                                    
                                    discretizer = Config.get_discretizer(obj1=obj1_type,obj2=obj2_type)

                                    relation = Relation(parameter1_type=obj1_type,
                                                        parameter2_type=obj2_type,
                                                        cr = cr,
                                                        region = region,
                                                        discretizer=discretizer)
                                    
                                    lifted_relations_dict[key_string][cr] = relation

        return lifted_relations_dict
    
    def get_object_dictionary(self,object_list):
        object_dictionary = {}
        for obj in object_list:
            if obj not in Config.NON_RELATION_OBJECTS:
                obj_type = obj.split("_")[0]
                if obj_type not in object_dictionary.keys():
                    object_dictionary[obj_type] = set()

                object_name = obj
                if obj_type in Config.CONST_TYPES[Config.DOMAIN_NAME]:
                    object_name = obj_type+"_Const"

                link = Link(link_name=object_name,
                            link_type=obj_type)
                object_dictionary[obj_type].add(link)
    
        return object_dictionary

    def get_graph(self,generate=False):
        try:
            print("loading graph")                
            transition_graph = TransitionGraph()
            with open(Config.DATA_MISC_DIR+self.file_prefix+"transition_graph.p") as f:
                graph_data = cPickle.load(f)
            transition_graph._g = graph_data["graph"]
            self.added_relations = graph_data["added_relations"]
            self.transitions_used = graph_data["transitions_used"]
        except:
            generate = True
        
        # demo_state = None
        env_transitions_used = []
        if generate:
            self.added_relations = {}
            transition_graph = TransitionGraph()
            print("generating graph")
            for i,env in enumerate(self.env_list):
                print("loading {} trajectory data".format(env))
                with open(Config.DATA_MISC_DIR+env+"/"+env+"_data.p") as f:
                    traj_data = cPickle.load(f)                    

                print("using {} trajectories".format(env))
                # traj_count = min(len(traj_data["env_states"]),self.process_count)
                env_num = int(env[3:])
                if env_num not in self.env_traj_dict:
                    continue
                traj_count = len(self.env_traj_dict[env_num])
                for traj_num in tqdm.tqdm(self.env_traj_dict[int(env[3:])]):
                # for traj in tqdm.tqdm(traj_data['env_states'][:traj_count]):
                    traj = traj_data['env_states'][traj_num]
                    if len(traj) == 0:
                        continue
                    self.og_lifted_relations_dict = self.get_lifted_relations_dict(traj[0].object_dict.keys()) 
                    self.new_lifted_relations_dict = copy.deepcopy(self.og_lifted_relations_dict)
                    object_dictionary = self.get_object_dictionary(traj[0].object_dict.keys())
                    prev_env_state = traj[0]
                    prev_state = get_abstract_state(env_state=traj[0],
                                                    object_dictionary=object_dictionary,
                                                    lifted_relations_dictionary=self.new_lifted_relations_dict,
                                                    aux_list = self.aux_list)
                    if prev_state not in transition_graph._g.nodes:
                        transition_graph.add_state(prev_state)
                        if prev_state not in self.added_relations.keys():
                            self.added_relations[prev_state] = []

                    local_added_relations = set()

                    for state_count,env_state in enumerate(traj[1:]):
                        next_state = get_abstract_state(env_state=env_state,
                                                        object_dictionary=object_dictionary,
                                                        lifted_relations_dictionary=self.new_lifted_relations_dict,
                                                        aux_list = self.aux_list)
                        
                        local_added_relations = self.get_local_added_relations(local_added_relations, prev_state, next_state)

                        next_env_state = env_state
                        env_transitions_used.append((prev_env_state,next_env_state))
                        if ((prev_state,next_state) not in transition_graph._g.edges) and (prev_state != next_state):
                            if next_state not in transition_graph._g.nodes:
                                transition_graph.add_state(next_state)
                                self.added_relations[next_state] = local_added_relations
                            if len(self.added_relations[next_state]) == 0:
                                self.added_relations[next_state] = local_added_relations
                            transition_graph.add_transition(prev_state,next_state)
    
                        if prev_state != next_state:
                            added_relations_to_remove = set([])
                            for re in self.added_relations[next_state]: 
                                if re not in local_added_relations: 
                                    added_relations_to_remove.add(re)
                            for re in added_relations_to_remove:
                                self.added_relations[next_state].remove(re)           

                        prev_state = next_state
                        prev_env_state = env_state
                        # prev_env_state_count = state_count+1
                                                
            print("num nodes {}".format(len(list(transition_graph._g.nodes))))
            print("num edges {}".format(len(transition_graph._g.edges)))
            transition_graph.save(name=self.file_prefix+"transition_graph",added_relations=self.added_relations,transitions=env_transitions_used)

        self.transition_graph = transition_graph
        self.graph_loaded = True
        return env_transitions_used
    
    def get_local_added_relations(self, local_added_relations, prev_state, next_state):
        new_local_added_relations = copy.deepcopy(local_added_relations)
        for re in next_state.true_set: 
            if re not in prev_state.true_set: 
                new_local_added_relations.add(re)
        return new_local_added_relations

    def get_num_plans_found(self,solution_dir,file_template):
        files = os.listdir(solution_dir)
        plan_count = 0

        for f in files:
            if file_template in f:
                plan_count += 1
        
        return plan_count

    def plan_generator(self,planner,start_i,end_i,problem_name,std_out_file): 
        i = start_i 
        done_flag = False
        if self.use_plan_file:
            done_flag = True
        while True: 
            time.sleep(2)
            if i > end_i:
                raise StopIteration

            if not self.use_plan_file:
                poll_code = self.p.poll()
            else:
                poll_code = 0
            
            if planner == "FF" and time.time()-self.ff_start_time >= Config.PLANNER_TIME_OUT and poll_code is None:
                self.p.kill()
                print("FF Planner Killed")
                yield [[]]

            if poll_code is None or done_flag: 
                print(poll_code,done_flag)
                while True and (poll_code is None or done_flag):
                    if planner == "KP":
                        solution_dir=Config.KP_SOLUTION_DIR+"{}traj_count/found_plans/".format(self.file_prefix)
                        plan_file = solution_dir+Config.KP_SOLUTION_NAME+".{}".format(i)
                    
                    elif planner == "FF":
                        plan_file = Config.FF_SOLUTION_DIR+self.env_prefix+self.file_prefix+problem_name+".pddl.soln"

                    time.sleep(1)
                    if os.path.isfile(plan_file): 
                        plan = []
                        #TODO: add handlers for FD
                        if planner == "KP":
                            self.success_flag = True
                            print("plan {} found".format(i))
                            with open(plan_file,"r") as f:
                                solution_file = f.readlines()
                                f.close()
                            for l in solution_file[:-1]:
                                plan.append(l.strip("\n"))
                            end_i = self.get_num_plans_found(solution_dir=solution_dir,file_template=Config.KP_SOLUTION_NAME)
                        
                        elif planner == "FF":
                            success_string = "found legal plan as follows"
                            if not self.use_plan_file:
                                if os.path.isfile(std_out_file):
                                    with open(std_out_file,"r") as f:
                                        std_str = f.read()
                                        f.close()                            
                                    os.remove(std_out_file)
                                    if success_string in std_str:
                                        self.success_flag = True
                                        print("plan {} found".format(i))
                                        with open(plan_file,"r") as f: 
                                            solution_file = f.readlines()
                                            f.close()
                                        for l in solution_file[1:]:
                                            plan.append(l.strip("\n"))
                                    else:
                                        self.success_flag = False
                            else:
                                print("plan {} found".format(i))
                                with open(plan_file,"r") as f: 
                                    solution_file = f.readlines()
                                    f.close()
                                for l in solution_file[1:]:
                                    plan.append(l.strip("\n"))
                        
                        if not self.keep_plans:
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
                        if not self.use_plan_file:
                            poll_code = self.p.poll()
                        else:
                            poll_code = 0
                        if poll_code is not None:
                            done_flag = True
                            time.sleep(1)
                        
                        if planner == "FF" and time.time()-self.ff_start_time >= Config.PLANNER_TIME_OUT and poll_code is None:
                            self.p.kill()
                            print("FF Planner Killed")
                            yield [[]]
                
                yield self.get_action_list([plan],current_planner=planner)
            
            elif poll_code is not None: 
                if poll_code < 0: 
                    raise StopIteration
                else:
                    done_flag = True
                    time.sleep(0.1)
                
    # @blockPrinting
    def get_plan(self,init_pddl_state,goal_pddl_state,start_i, end_i, get_problem_flag=True,problem_name="",get_domain_flag=False,use_plan_file=False,planner=None,k_multiplier=1):
        additional_constants = []
        if get_domain_flag:
            additional_constants = self.get_additional_constants(self.edited_actions_dict)
        self.get_domain_pddl(domain_name=self.domain,edited_action_dict=self.edited_actions_dict,additional_constants=additional_constants,create_domain_flag=get_domain_flag)

        if get_problem_flag:
            problem_name = self.get_problem_pddl(init_state=init_pddl_state,
                                                goal_state=goal_pddl_state)

        std_out_file = ""
        if not use_plan_file:
            num_plans = end_i
            if planner == "FF":
                self.ff_start_time = time.time()
            if planner == "KP":
                num_plans = self.total_plans

            if planner == "FF" or k_multiplier == 1:
                self.p, std_out_file = self.call_planner(problem_name=problem_name,planner=planner,num_plans=num_plans)

        return self.plan_generator(planner,start_i,end_i,problem_name,std_out_file), problem_name
   
    def get_variable_string(self,variable_list):
        s = ""
        for var in variable_list:
            s+=var.__str__()
            s+=", "
        
        return s
    
    def gen_actions(self,clusters):
        actions = []       
        
        LiftedPDDLAction.action_id = len(self.actions)
        for cluster in clusters.keys():
            action = LiftedPDDLAction.get_action_from_cluster(clusters[cluster],self.added_relations)
            actions.append(action)
        
        return actions
    
    @blockPrinting
    def generate_transition_clusters(self,generate=False,trans=None):
        try:
            print("loading clusters")                
            with open(Config.DATA_MISC_DIR+self.file_prefix+"transition_clusters.p") as f:
                transition_clusters = cPickle.load(f)
            
        except:
            generate = True
            save_flag = True
        
        if generate:
            save_flag = False
            print("generating clusters")
            if trans is None:
                save_flag = True
                trans = self.transition_graph._g.edges

            transition_clusters = {}
            for transition in trans:
                state1,state2 = transition
                added_relations = []
                deleted_relations =[]
                added_auxilaries = []
                deleted_auxilaries = []

                for re2 in state2.true_set:
                    if re2 in state1.false_set:
                        added_relations.append(re2)
                
                for re2 in state2.false_set:
                    if re2 in state1.true_set:
                        deleted_relations.append(re2)

                for a_re in state2.aux_true_set:
                    if a_re not in state1.aux_true_set:
                        added_auxilaries.append(a_re)
                    
                for a_re in state1.aux_true_set:
                    if a_re not in state2.aux_true_set:
                        deleted_auxilaries.append(a_re)

                added_relations.sort()
                deleted_relations.sort()
                added_auxilaries.sort()
                deleted_auxilaries.sort()

                param_added_relations = []
                param_deleted_relations = []
                param_added_aux_relations = []
                param_deleted_aux_relations = []

                param_dict = {}
                for re in added_relations[::-1]:
                    if re.parameter1 not in param_dict.keys():
                        param_dict[re.parameter1] = "p{}".format(len(param_dict.values()))
                    if re.parameter2 not in param_dict.keys():
                        param_dict[re.parameter2] = "p{}".format(len(param_dict.values()))
                
                for re in deleted_relations[::-1]:
                    if re.parameter1 not in param_dict.keys():
                        param_dict[re.parameter1] = "p{}".format(len(param_dict.values()))
                    if re.parameter2 not in param_dict.keys():
                        param_dict[re.parameter2] = "p{}".format(len(param_dict.values()))
                
                for re in added_auxilaries[::-1]:
                    if re.parameter not in param_dict.keys():
                        param_dict[re.parameter] = "p{}".format(len(param_dict.values()))
                    
                    a = copy.deepcopy(re)
                    a.parameter = param_dict[re.parameter]
                    param_added_aux_relations.append(a)

                for re in deleted_auxilaries[::-1]:
                    if re.parameter not in param_dict.keys():
                        param_dict[re.parameter] = "p{}".format(len(param_dict.values()))
                    
                    a = copy.deepcopy(re)
                    a.parameter = param_dict[re.parameter]
                    param_deleted_aux_relations.append(a)
                
                for re in added_relations:
                    r = ParameterizedLiftedRelation(param_dict[re.parameter1],
                                                    param_dict[re.parameter2],
                                                    re.get_lifted_relation())
                    
                    param_added_relations.append(r)

                for re in deleted_relations:
                    r = ParameterizedLiftedRelation(param_dict[re.parameter1],
                                                    param_dict[re.parameter2],
                                                    re.get_lifted_relation())
                    
                    param_deleted_relations.append(r)
                
                param_added_relations.sort()
                param_deleted_relations.sort()
                param_added_aux_relations.sort()
                param_deleted_aux_relations.sort()

                changed_relations_key = ChangedRelations(changed_lifted_added_relations=list(set(param_added_relations)),
                                                         changed_lifted_deleted_relations=list(set(param_deleted_relations)),
                                                         added_auxilary_relations=list(set(param_added_aux_relations)),
                                                         deleted_auxilary_relations=list(set(param_deleted_aux_relations)))
                
                if changed_relations_key not in transition_clusters.keys():
                    transition_clusters[changed_relations_key] = []
                
                transition_clusters[changed_relations_key].append(transition)
            
            if save_flag:
                with open(Config.DATA_MISC_DIR+self.file_prefix+"transition_clusters.p","wb") as f:
                    cPickle.dump(transition_clusters,f,protocol=cPickle.HIGHEST_PROTOCOL)

        return transition_clusters
    
    def get_kept_planks_in_state(self,env_state):
        kept_planks_set = set()
        c_state = get_abstract_state(env_state=env_state,
                                     object_dictionary=self.object_dictionary,
                                     lifted_relations_dictionary=self.new_lifted_relations_dict,
                                     aux_list=self.aux_list)
        for re in c_state.true_set:
            if re.parameter1_type in Config.CONST_TYPES[Config.DOMAIN_NAME] and re.cr == 1:
                kept_planks_set.add(int(re.parameter2.split("_")[1]))
            elif re.parameter2_type in Config.CONST_TYPES[Config.DOMAIN_NAME] and re.cr == 1:
                kept_planks_set.add(int(re.parameter1.split("_")[1]))

        return kept_planks_set

    def get_action_info(self,plan):
        info_vector = []
        for action_num,action in enumerate(plan):
            action_type = None
            offset_axis = None
            offset_order = None
            for re in action.effect.add_set:
                if re.parameter1_type == "gripper" and re.parameter2_type == Config.OBJECT_NAME:
                    if re.cr == 2:
                        for re2 in action.precondition.true_set:
                            if re2.parameter1_type == "gripper" and re2.parameter2_type == Config.OBJECT_NAME:
                                if re2.cr == 0:
                                    action_type = "pre_grab"
                                    offset_order = "pre"
                                    offset_axis = "z"
                                elif re2.cr == 1:
                                    action_type = "release"
                                    offset_order = "post"
                                    if self.sim_object.axis_for_offset == "y":
                                        offset_axis = "z"
                                    elif self.sim_object.axis_for_offset == "x":
                                        offset_axis = "x"

                if "goalLoc" in re.parameter1_type and re.parameter2_type == Config.OBJECT_NAME:
                    if re.cr == 1:
                        action_type = "place"
                        offset_order = "pre"
                        offset_axis = "z"

            info_vector.append((action_type,offset_axis,offset_order))
        
        for i,info_tuple in enumerate(info_vector):
            current_type, current_axis, current_order = info_tuple

            if current_type == "pre_grab":
                next_type, _, _ = info_vector[i+1]
                if next_type is None:
                    next_type = "grab"
                    next_axis = current_axis
                    next_order = "post"
                    info_vector[i+1] = (next_type,next_axis,next_order)
            
        return info_vector

    # @blockPrinting
    def check_plan_refinement(self,init_env_state,plan,relations_to_learn,init_object_placement={},objects_to_move=-1):
        action_instance_list = {}
        refined_plan = []
        if len(plan) == 0:
            return False
        env_state = init_env_state
        env_state_list = [env_state]
        required_plank = ""
        self.sim_object.set_env_state(init_env_state)
        # plan_info = self.get_action_info(plan)

        kept_planks_set = self.get_kept_planks_in_state(env_state)

        validation_flag = True
        time_out_flag = False
        i = 0
        learnt_action = None
        refined_action_counts = []
        last_refined_state = None
        start_time = time.time()
        state_action_mapping = {}
        pddl_states_list = []
        continue_flag = True
        continue_flag = self.check_plan_order(plan,kept_planks=list(kept_planks_set))
        required_planks = []
        collision_planks = []
        
        if not continue_flag:
            validation_flag = False
        else:
            print("REFINING plan with len {}".format(len(plan)))
            print("Plan ->")
            for ac in plan:
                print(ac)

        while i < len(plan) and continue_flag:
            print(i+1,len(env_state_list))
            grounded_action = plan[i]
            # action_info = plan_info[i]
            action_info = [None,None,None]
            required_planks = []
            collision_planks = []
            plank_to_pick = None
            if Config.DOMAIN_NAME in ["Keva","Jenga"] and i%5 == 0:
                plank_to_pick = self.get_plank_to_pick(grounded_action,kept_planks_set)
                if plank_to_pick is not None:
                    self.sim_object.set_env_state(env_state)
                    env_state = self.sim_object.set_plank(plank_to_pick,real_world_flag=True) #TODO: Set to false for simulator experiments
                    sleep(0.0001)

            current_env_state = copy.deepcopy(env_state)
            current_state = get_abstract_state(env_state=env_state,
                                               object_dictionary=self.object_dictionary,
                                               lifted_relations_dictionary=self.new_lifted_relations_dict,
                                               aux_list = self.aux_list)

            if i == 0:
                pddl_states_list.append(current_state)
                state_action_mapping[plan[0]] = current_state
            if i not in action_instance_list:
                action_instance_list[i] = {}

            traj,grab_bool,(rel,instance,instance_count),rob,req_grounded_pose,static_object, eef_transform, objects_in_collision_list = grounded_action.compute_refinement(env_state=env_state,
                                                                                                                                                                            sim_object=self.sim_object,
                                                                                                                                                                            previous_instances=action_instance_list[i],
                                                                                                                                                                            compute_motion_plan = self.compute_motion_plan,
                                                                                                                                                                            action_info = action_info)
            current_time = time.time()
            # if Config.TIMEOUT_THRESHOLD[Config.DOMAIN_NAME] != -1:
            #     if (current_time-start_time)>=Config.TIMEOUT_THRESHOLD[Config.DOMAIN_NAME]:
            #         validation_flag = False
            #         time_out_flag = True
            #         break

            if rel is None and instance is None:
                if i != 0:
                    action_instance_list[i] = {}
                    i -= 1 
                    validation_flag = True
                else:
                    validation_flag = False
                    break

                env_state = copy.deepcopy(env_state_list[i])
                kept_planks_set = self.get_kept_planks_in_state(env_state)
                env_state_list = copy.deepcopy(env_state_list[:-1])
                pddl_states_list = copy.deepcopy(pddl_states_list[:-1])
                # refined_plan = copy.deepcopy(refined_plan[:-1])
                refined_plan.pop(-1)
                continue
            
            if rel not in action_instance_list[i].keys():
                action_instance_list[i][rel] = set()
            
            found_flag = False
            found_intances = set([])
            to_be_added_instance = set([])
            for (inst, inst_count) in action_instance_list[i][rel]:
                found_flag = False
                if inst == tuple(instance):
                    found_flag = True
                    found_intances.add((tuple(inst),inst_count))
                    to_be_added_instance.add((inst,instance_count))

            for inst_tuple in found_intances:
                action_instance_list[i][rel].remove(inst_tuple)

            for inst_tuple in to_be_added_instance:
                action_instance_list[i][rel].add(inst_tuple)

            if found_flag is False:
                action_instance_list[i][rel].add((tuple(instance),instance_count))
            
            if traj is not None:
                if len(traj) == 2:
                    for t in traj:
                        if t is None:
                            continue
                        env_state = self.sim_object.execute_refinement(robot=rob,traj=t)
                else:
                    env_state = self.sim_object.execute_refinement(robot=rob,traj=traj)

            obj_name=None
            _,_,action_order = action_info
            if grab_bool is not None:
                for relation in grounded_action.changed_relations:
                    if relation.parameter1.split("_")[0] not in Config.IMMOVABLE_OBJECTS and relation.parameter1_type not in Config.ROBOT_TYPES.keys():
                        obj_name = relation.parameter1
                        break
                    elif relation.parameter2.split("_")[0] not in Config.IMMOVABLE_OBJECTS and relation.parameter2_type not in Config.ROBOT_TYPES.keys():
                        obj_name = relation.parameter2
                        break
                if type(grab_bool) == list:
                    env_state = self.sim_object.execute_refinement(robot=rob,traj=grab_bool[0],obj_name=obj_name)
                else:
                    env_state = self.sim_object.execute_refinement(robot=rob,traj=grab_bool,obj_name=obj_name)

            grabbed_object = obj_name
            
            next_state = get_abstract_state(env_state=env_state,
                                            object_dictionary=self.object_dictionary,
                                            lifted_relations_dictionary=self.new_lifted_relations_dict,
                                            aux_list = self.aux_list)

            if grab_bool is not None and type(grab_bool) == list:
                env_state = self.sim_object.execute_refinement(robot=rob,traj=grab_bool[1])

            # if i > 0:
            pddl_states_list.append(self.get_next_pddl_state(pddl_states_list[i],grounded_action))

            if i+1 < len(plan):
                state_action_mapping[plan[i+1]] = pddl_states_list[i+1]

            for re in grounded_action.effect.add_set:
                if re not in next_state.true_set:
                    validation_flag = False
                    print("{} not in add".format(re))
            
            # for re in grounded_action.effect.delete_set:
            #     if re not in next_state.false_set:
            #         validation_flag = False
            #         print("{} not in delete".format(re))
            if validation_flag:
                last_refined_state = next_state
                if Config.DOMAIN_NAME in ["Keva","Jenga"]:
                    newly_kept_plank = set()
                    for re in next_state.true_set:
                        if "goalLoc" in re.__str__() and re.cr != 0:
                            if re.parameter1_type == Config.OBJECT_NAME[0]:
                                plank_num = int(re.parameter1.split("_")[1])
                                if plank_num not in kept_planks_set:
                                    newly_kept_plank.add(plank_num)
                            if re.parameter2_type == Config.OBJECT_NAME[0]:
                                plank_num = int(re.parameter2.split("_")[1])
                                if plank_num not in kept_planks_set:
                                    newly_kept_plank.add(plank_num)
                    
                    if len(objects_in_collision_list) == 0:
                        for p in newly_kept_plank:
                            if not (Config.PLANKS_PROBLEM_ORDER[Config.DOMAIN_NAME][self.test_structure][p].issubset(kept_planks_set)):
                                validation_flag = False
                                # relations,required_planks = self.get_relation_from_DAG(p,relations_to_learn)
                                if self.planner == "PO":
                                    # relations,required_planks = self.get_info_to_learn(p,relations_to_learn,kept_planks=kept_planks_set)
                                    relations_to_learn = relations_to_learn.union(relations)
                                return validation_flag, -1, relations_to_learn, [], grounded_action.lifted_action_id, required_planks, state_action_mapping[grounded_action], i+1, env_state_list
                    else:
                        validation_flag = False
                        plank_to_keep = self.get_plank_to_be_kept(rob,env_state)
                        required_planks = self.get_collision_object_tuple(plank_to_keep,objects_in_collision_list)              
                        return validation_flag, -1, relations_to_learn, [], grounded_action.lifted_action_id, required_planks, state_action_mapping[grounded_action], i+1, env_state_list
            
                    kept_planks_set = copy.deepcopy(kept_planks_set.union(newly_kept_plank))
                    
                lifted_add_set = set([])
                for re in next_state.true_set:
                    if re not in current_state.true_set:
                        lifted_add_set.add(re.get_lifted_relation())
                            
                if len(relations_to_learn.intersection(lifted_add_set)) > 0: 
                    new_relations_present = relations_to_learn.intersection(lifted_add_set)
                    print("the transition to add -> {}".format(grounded_action))

                    for r in new_relations_present:
                        self.og_lifted_relations_dict["{}_{}".format(r.parameter1_type,r.parameter2_type)][r.cr] = r
                        self.og_lifted_relations_dict["{}_{}".format(r.parameter1_type,r.parameter2_type)][0].region.extend(r.region)
                    
                    self.new_relations = self.new_relations.union(new_relations_present)
                    self.new_ll_transitions[i+1] = (current_env_state,env_state)

            else:
                i -= 1
                env_state = copy.deepcopy(env_state_list[i])
                validation_flag = True
                kept_planks_set = self.get_kept_planks_in_state(env_state)
                env_state_list = copy.deepcopy(env_state_list[:-1])
                pddl_states_list = copy.deepcopy(pddl_states_list[:-1])
                refined_plan.pop(-1)
                continue
            
            i += 1
            if i not in refined_action_counts:
                refined_action_counts.append(i)

            exact_current_env_state = self.sim_object.get_current_state()
            refined_plan.append((grounded_action.lifted_action_id,rob,traj,grab_bool,obj_name,req_grounded_pose,exact_current_env_state,grabbed_object,static_object,rel,eef_transform))
            env_state_list.append(env_state)
        
        if validation_flag:
            if objects_to_move != -1:
                placed_objects = self.get_placed_objects(init_object_placement,current_state)
                if placed_objects != objects_to_move:
                    validation_flag = False
                    print("Validation Failed")
                    self.sim_object.set_env_state(init_env_state)
                    return validation_flag, 3, relations_to_learn, env_state_list, plan[refined_action_counts[-1] - 1].lifted_action_id, required_planks, state_action_mapping[plan[refined_action_counts[-1] - 1]], i+1, env_state_list

            print("Validation Completed")
            self.sim_object.set_env_state(init_env_state)
            return validation_flag,refined_plan, relations_to_learn, env_state_list, plan[refined_action_counts[-1] - 1].lifted_action_id, required_planks, state_action_mapping[plan[refined_action_counts[-1] - 1]], i+1, env_state_list

        else:
            print("Validation Failed")
            self.sim_object.set_env_state(init_env_state)
            if len(refined_action_counts) == 0:
                refined_action_count_index = 0
            else:
                refined_action_count_index = refined_action_counts[-1]
            
            if continue_flag:
                return validation_flag, 2, relations_to_learn, [], plan[refined_action_count_index].lifted_action_id, required_planks, state_action_mapping[plan[refined_action_count_index]], i+1, env_state_list
            else:
                return validation_flag, 2, relations_to_learn, [], plan[refined_action_count_index].lifted_action_id, required_planks, None, i+1, env_state_list

    def get_collision_object_tuple(self,plank_to_keep,objects_in_collision_list):
        collision_object_tuple_set = set([])
        for obj in objects_in_collision_list:
            collision_object_tuple_set.add(tuple([plank_to_keep,obj]))
        
        return collision_object_tuple_set
    
    def get_plank_to_be_kept(self,rob,env_state):
        rob_id = rob.id
        grabbed_flag = getattr(env_state,"grabbed_flag_{}".format(rob_id))
        grabbed_object = getattr(env_state,"grabbed_object_{}".format(rob_id))

        return grabbed_object

    def get_next_pddl_state(self,current_pddl_state,grounded_action):
        relations_to_delete = set([])
        next_pddl_state = copy.deepcopy(current_pddl_state)

        for re in grounded_action.effect.delete_set:
            if re in next_pddl_state.true_set:
                next_pddl_state.true_set.remove(re)
                next_pddl_state.false_set.add(re)

        for re in grounded_action.effect.add_set:
            if re not in next_pddl_state.true_set:
                next_pddl_state.true_set.add(re)
                next_pddl_state.false_set.remove(re)

        return next_pddl_state

    def get_candidate_relation(self):
        freq_keys = self.new_goal_relations.keys()
        freq_keys.sort()
        freq_keys.reverse()
        for freq in freq_keys:
            if freq not in self.considered_relations.keys():
                self.considered_relations[freq] = {}
            for relation_type in self.new_goal_relations[freq].keys():
                if relation_type not in self.considered_relations[freq].keys():
                    self.considered_relations[freq][relation_type] = {}
                for cr_num in self.new_goal_relations[freq][relation_type]:
                    re = self.new_goal_relations[freq][relation_type][cr_num]
                    if cr_num not in self.considered_relations[freq][relation_type].keys():
                        self.considered_relations[freq][relation_type][cr_num] = re
                        if cr_num not in self.new_lifted_relations_dict["{}_{}".format(relation_type[0],relation_type[1])].keys():
                            return re,[freq,relation_type,cr_num]
        
        return None,[None,None,None]

    def get_req_object_pairs(self,plank_num):
        req_obj_pairs = []
        for pair in self.problem_object_pairs:
            _,obj2 = pair
            if int(obj2.split("_")[-1]) == plank_num:
                req_obj_pairs.append(pair)
        
        return req_obj_pairs

    def get_discretized_pose_for_pair(self,obj_pair):
        obj1,obj2 = obj_pair

        obj1_type = obj1.split("_")[0]
        obj2_type = obj2.split("_")[0]
        discretizer = Config.get_discretizer(obj1_type,obj2_type)

        obj1_pose = self.goal_env_state.object_dict[obj1]
        obj2_pose = self.goal_env_state.object_dict[obj2]

        rel_pose = self.sim_object.get_relative_pose(obj1_pose,obj2_pose)
        discretized_pose = discretizer.get_discretized_pose(rel_pose,is_relative=True)
        discretized_pose.append(0) 

        return [discretized_pose]

    def add_relations_to_dict(self,relations_dict,relations):
        add_flag = False
        for re in relations:
            if "{}_{}".format(re.parameter1_type,re.parameter2_type) not in relations_dict.keys():
                relations_dict["{}_{}".format(re.parameter1_type,re.parameter2_type)] = {}
                relations_dict["{}_{}".format(re.parameter1_type,re.parameter2_type)][0] = Relation(parameter1_type=re.parameter1_type,
                                                                                                    parameter2_type=re.parameter2_type,
                                                                                                    cr=0,
                                                                                                    region = [],
                                                                                                    discretizer=Config.get_discretizer(re.parameter1_type,re.parameter2_type))

            if re not in relations_dict["{}_{}".format(re.parameter1_type,re.parameter2_type)].values():
                add_flag = True
                if re.cr in relations_dict["{}_{}".format(re.parameter1_type,re.parameter2_type)].keys():
                    cr = re.cr*100
                else:
                    cr = re.cr
                relations_dict["{}_{}".format(re.parameter1_type,re.parameter2_type)][cr] = re
                relations_dict["{}_{}".format(re.parameter1_type,re.parameter2_type)][0].region.extend(re.region)
        
        return add_flag, relations_dict

    def get_info_to_learn(self,failed_plank_num,relations_to_learn=set([]),kept_planks=set([])):
        req_obj_pairs = self.get_req_object_pairs(failed_plank_num)
        required_planks = []
        relations = set([])
        candidate_found = False

        if len(relations_to_learn) > 0:
            for obj_pair in req_obj_pairs:
                obj1,obj2 = obj_pair
                region = self.get_discretized_pose_for_pair(obj_pair)
                for re in relations_to_learn:
                    if region == re.region:
                        required_planks.append(obj_pair)
                        break
        
        not_used_considered_relations = []
        while not candidate_found:
            candidate_relation,considered_relation_track = self.get_candidate_relation()  
            if candidate_relation is not None and candidate_relation not in relations_to_learn:      
                for obj_pair in req_obj_pairs:
                    obj1,obj2 = obj_pair
                    region = self.get_discretized_pose_for_pair(obj_pair)
                    if region == candidate_relation.region and int(obj1.split("_")[1]) not in kept_planks:
                        required_planks.append(obj_pair)
                        relations.add(candidate_relation)
                        self.new_lifted_relations_dict["{}_{}".format(obj1.split("_")[0],obj2.split("_")[0])][candidate_relation.cr] = candidate_relation
                        self.new_lifted_relations_dict["{}_{}".format(obj1.split("_")[0],obj2.split("_")[0])][0].region.extend(region)
                        self.data[obj1.split("_")[0]][obj2.split("_")[0]].append(region)
                        candidate_found = True

                if not candidate_found:
                    not_used_considered_relations.append(considered_relation_track)

            else:
                break
        
        for considered_relation_track in not_used_considered_relations:
            del self.considered_relations[considered_relation_track[0]][considered_relation_track[1]][considered_relation_track[2]]

        return relations, required_planks

    def get_changed_relations(self,transition):
        changed_relations = set([])
        state1, state2 = transition

        for r1 in state1.true_set: 
            if r1 not in state2.true_set:
                changed_relations.add(r1)
        for r1 in state2.true_set: 
            if r1 not in state1.true_set: 
                changed_relations.add(r1)

        return changed_relations

    def get_relation_to_add(self,transitions):
        if Config.DOMAIN_NAME not in ["Keva","Jenga"]:
            return None
        
        goal_relations = []
        freq_keys = self.new_goal_relations.keys()
        freq_keys.sort()
        freq_keys.reverse()
        for freq in freq_keys:
            for rel_type in self.new_goal_relations[freq]:
                goal_relations.extend(self.new_goal_relations[freq][rel_type].values())
        
        relations_to_use = copy.deepcopy(self.goal_relations)
        _,relations_to_use = self.add_relations_to_dict(relations_to_use,goal_relations)

        aux_to_use = self.get_auxilary_preds(lifted_relations_dict=relations_to_use)
        for prev_env_state, next_env_state in transitions:
            prev_state = get_abstract_state(env_state=prev_env_state,
                                            object_dictionary=self.object_dictionary,
                                            lifted_relations_dictionary=relations_to_use,
                                            aux_list = aux_to_use)

            next_state = get_abstract_state(env_state=next_env_state,
                                            object_dictionary=self.object_dictionary,
                                            lifted_relations_dictionary=relations_to_use,
                                            aux_list = aux_to_use)

            changed_relations = self.get_changed_relations((prev_state,next_state))
            lifted_changed_relations = set([re.get_lifted_relation() for re in changed_relations])
            
            for r in goal_relations:
                if r in lifted_changed_relations:
                    return r

        return None        

    def get_freq_relations(self,least_freq_desired):
        relations_required = []

        freq_keys = self.new_goal_relations.keys()
        freq_keys.sort()
        freq_keys.reverse()
        for freq in freq_keys[:least_freq_desired]:
            for rel_type in self.new_goal_relations[freq]:
                relations_required.extend(self.new_goal_relations[freq][rel_type].values())
        
        return set(relations_required)
    
    def get_relation_from_DAG(self,plank_num,relations_to_learn=set([])):        
        plank_name = "{}_".format(Config.OBJECT_NAME[0]) + str(plank_num)
        parameter2_type = Config.OBJECT_NAME[0]
        parameter1_type = Config.OBJECT_NAME[0]
        planks_required = []
        relations = set([])
        if plank_name not in self.failed_planks.keys():
            while len(self.unknown_plank_relations[plank_name]) > 0:
                _,edge = heapq.heappop(self.unknown_plank_relations[plank_name])
                region = edge[2]
                planks_required.append(tuple(edge[:2]))
                cr = len(self.new_lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])].keys())
                discretizer = Config.get_discretizer(obj1=parameter1_type,obj2=parameter2_type)

                relation = Relation(parameter1_type=parameter1_type,
                                    parameter2_type=parameter2_type,
                                    cr = cr,
                                    region = region,
                                    discretizer=discretizer)
                
                if relation not in relations_to_learn and relation not in self.new_lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])].values() :                
                    self.new_lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])][cr] = relation
                    self.new_lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])][0].region.extend(region)
                    if plank_name not in self.failed_planks.keys():
                        self.failed_planks[plank_name] = []
                    self.failed_planks[plank_name].append([edge,relation])
                    relations.add(relation)
                    self.data[Config.OBJECT_NAME[0]][Config.OBJECT_NAME[0]].append(region)
            
            self.test_structure_dag = self.load_dag(structure_name=self.test_structure,create=True,rcr_dict=self.data)

        else:
            for edge,relation in self.failed_planks[plank_name]:
                if relation not in relations_to_learn:
                    relations.add(relation)
                    planks_required.append(tuple(edge[:2]))

        return relations, planks_required

    def add_action_to_model(self,transition,add=False):
        current_state, next_state = (transition)
        changed_relations = set([])
        changed_auxilaries = set([])
        parameter_set = set([])
        deleted_relations = set([])
        added_relations = set([])
        deleted_aux = set([])
        added_aux = set([])
        lifted_added_relations = []
        lifted_deleted_relations = []

        for r1 in current_state.true_set: 
            if r1 not in next_state.true_set:
                changed_relations.add(r1)
                deleted_relations.add(r1)
        for r1 in next_state.true_set: 
            if r1 not in current_state.true_set: 
                changed_relations.add(r1)
                added_relations.add(r1)

        for a_re in next_state.aux_true_set:
            if a_re not in current_state.aux_true_set:
                changed_auxilaries.add(copy.deepcopy(a_re))
                added_aux.add(a_re)
            
        for a_re in current_state.aux_true_set:
            if a_re not in next_state.aux_true_set:
                changed_auxilaries.add(copy.deepcopy(a_re))
                deleted_aux.add(a_re)
        
        for re in changed_relations:
            parameter_set.add(re.parameter1)
            parameter_set.add(re.parameter2)
        
        for ax in changed_auxilaries:
            parameter_set.add(ax.parameter)

        relations_to_remove = set([])
        for re in current_state.true_set:
            if "goalLoc" in re.parameter1 or "goalLoc" in re.parameter2:
                if "goalLoc" in re.parameter1 and re.parameter2 not in parameter_set:
                    relations_to_remove.add(re)
                if "goalLoc" in re.parameter2 and re.parameter1 not in parameter_set:
                    relations_to_remove.add(re)
            elif "gripper" in re.parameter1 or "gripper" in re.parameter2:
                if "gripper" in re.parameter1 and re.parameter2 not in parameter_set:
                    relations_to_remove.add(re)
                if "gripper" in re.parameter2 and re.parameter1 not in parameter_set:
                    relations_to_remove.add(re)
            elif re.parameter1 not in parameter_set or re.parameter2 not in parameter_set:
                relations_to_remove.add(re)            

        for re in current_state.false_set:
            if "goalLoc" in re.parameter1 or "goalLoc" in re.parameter2:
                if "goalLoc" in re.parameter1 and re.parameter2 not in parameter_set:
                    relations_to_remove.add(re)
                if "goalLoc" in re.parameter2 and re.parameter1 not in parameter_set:
                    relations_to_remove.add(re)
            elif "gripper" in re.parameter1 or "gripper" in re.parameter2:
                if "gripper" in re.parameter1 and re.parameter2 not in parameter_set:
                    relations_to_remove.add(re)
                if "gripper" in re.parameter2 and re.parameter1 not in parameter_set:
                    relations_to_remove.add(re)
            elif re.parameter1 not in parameter_set or re.parameter2 not in parameter_set:
                relations_to_remove.add(re)            

        for re in next_state.true_set:
            if "goalLoc" in re.parameter1 or "goalLoc" in re.parameter2:
                if "goalLoc" in re.parameter1 and re.parameter2 not in parameter_set:
                    relations_to_remove.add(re)
                if "goalLoc" in re.parameter2 and re.parameter1 not in parameter_set:
                    relations_to_remove.add(re)
            elif "gripper" in re.parameter1 or "gripper" in re.parameter2:
                if "gripper" in re.parameter1 and re.parameter2 not in parameter_set:
                    relations_to_remove.add(re)
                if "gripper" in re.parameter2 and re.parameter1 not in parameter_set:
                    relations_to_remove.add(re)
            elif re.parameter1 not in parameter_set or re.parameter2 not in parameter_set:
                relations_to_remove.add(re)            
        
        for re in next_state.false_set:
            if "goalLoc" in re.parameter1 or "goalLoc" in re.parameter2:
                if "goalLoc" in re.parameter1 and re.parameter2 not in parameter_set:
                    relations_to_remove.add(re)
                if "goalLoc" in re.parameter2 and re.parameter1 not in parameter_set:
                    relations_to_remove.add(re)
            elif "gripper" in re.parameter1 or "gripper" in re.parameter2:
                if "gripper" in re.parameter1 and re.parameter2 not in parameter_set:
                    relations_to_remove.add(re)
                if "gripper" in re.parameter2 and re.parameter1 not in parameter_set:
                    relations_to_remove.add(re)
            elif re.parameter1 not in parameter_set or re.parameter2 not in parameter_set:
                relations_to_remove.add(re)            

        aux_to_remove = set([])
        for re in current_state.aux_true_set:
            if re.parameter not in parameter_set:
                aux_to_remove.add(re)
        
        for re in current_state.aux_false_set:
            if re.parameter not in parameter_set:
                aux_to_remove.add(re)

        for re in next_state.aux_true_set:
            if re.parameter not in parameter_set:
                aux_to_remove.add(re)
        
        for re in next_state.aux_false_set:
            if re.parameter not in parameter_set:
                aux_to_remove.add(re)

        new_set = set()
        for re in current_state.true_set:
            flag = 0
            for r in relations_to_remove:
                if re == r:
                    flag = 1
                    break
            if flag == 0:
                new_set.add(re)
        current_state.true_set = copy.deepcopy(new_set)

        new_set = set()
        for re in next_state.true_set:
            flag = 0
            for r in relations_to_remove:
                if re == r:
                    flag = 1
                    break
            if flag == 0:
                new_set.add(re)
        next_state.true_set = copy.deepcopy(new_set)
        
        new_set = set()
        for re in current_state.false_set:
            flag = 0
            for r in relations_to_remove:
                if re == r:
                    flag = 1
                    break
            if flag == 0:
                new_set.add(re)
        current_state.false_set = copy.deepcopy(new_set)

        new_set = set()
        for re in next_state.false_set:
            flag = 0
            for r in relations_to_remove:
                if re == r:
                    flag = 1
                    break
            if flag == 0:
                new_set.add(re)
        next_state.false_set = copy.deepcopy(new_set)

        new_set = set()
        for aux in current_state.aux_true_set:
            flag = 0
            for ax in aux_to_remove:
                if aux == ax:
                    flag = 1
                    break
            if flag == 0:
                new_set.add(aux)
        current_state.aux_true_set = copy.deepcopy(new_set)

        new_set = set()
        for aux in next_state.aux_true_set:
            flag = 0
            for ax in aux_to_remove:
                if aux == ax:
                    flag = 1
                    break
            if flag == 0:
                new_set.add(aux)
        next_state.aux_true_set = copy.deepcopy(new_set)

        new_set = set()
        for aux in current_state.aux_false_set:
            flag = 0
            for ax in aux_to_remove:
                if aux == ax:
                    flag = 1
                    break
            if flag == 0:
                new_set.add(aux)
        current_state.aux_false_set = copy.deepcopy(new_set)

        new_set = set()
        for aux in next_state.aux_false_set:
            flag = 0
            for ax in aux_to_remove:
                if aux == ax:
                    flag = 1
                    break
            if flag == 0:
                new_set.add(aux)
        next_state.aux_false_set = copy.deepcopy(new_set)

        added_relations_dict_to_use = copy.deepcopy(self.added_relations)
        if next_state not in added_relations_dict_to_use.keys():
            new_added_relations = set([])
        else:
            new_added_relations = added_relations_dict_to_use[next_state]

        added_relations = self.get_local_added_relations(new_added_relations,current_state,next_state)
        new_added_relations = new_added_relations.union(added_relations)

        if current_state not in added_relations_dict_to_use.keys():
            added_relations_dict_to_use[current_state] = set([])
        added_relations_dict_to_use[next_state] = new_added_relations

        LiftedPDDLAction.action_id = len(self.actions)
        action = LiftedPDDLAction.get_action_from_cluster([[current_state,next_state]],added_relations_dict_to_use)
        self.actions.append(action)
        print(action)

        if add:
            for re in added_relations:
                lifted_added_relations.append(re.get_lifted_relation())
            
            for re in deleted_relations:
                lifted_deleted_relations.append(re.get_lifted_relation())

            lifted_added_relations.sort()
            lifted_deleted_relations.sort()

            changed_relations_key = ChangedRelations(changed_lifted_added_relations=lifted_added_relations,
                                                    changed_lifted_deleted_relations=lifted_deleted_relations,
                                                    added_auxilary_relations=added_aux,
                                                    deleted_auxilary_relations=deleted_aux)
        
            if changed_relations_key not in self.transition_clusters.keys():
                self.transition_clusters[changed_relations_key] = []

            self.transition_clusters[changed_relations_key].append([current_state,next_state])

        return action

    def simulate_traj(self,traj):
        self.sim_object.set_env_state(traj[0])
        for i,env_state in enumerate(traj[1:]):
            self.sim_object.set_env_state(env_state)
            sleep(0.01)

    def get_auxilary_preds(self,object_dictionary=None,lifted_relations_dict=None):
        if object_dictionary is None:
            object_dictionary = self.object_dictionary
        if lifted_relations_dict is None:
            lifted_relations_dict = self.new_lifted_relations_dict

        auxilary_set = set()
        non_auxilary_object_set = Config.get_non_aux_object_set()
        for object_pair in lifted_relations_dict: 
            obj1, obj2 = object_pair.split("_")[0],object_pair.split("_")[1]
            if not (obj1 in non_auxilary_object_set and obj2 in non_auxilary_object_set):
                for cr in lifted_relations_dict[object_pair]: 
                    if cr != 0:
                        lr = lifted_relations_dict[object_pair][cr]
                        if lr.parameter1_type not in Config.CONST_TYPES[Config.DOMAIN_NAME] and lr.parameter2_type not in Config.CONST_TYPES[Config.DOMAIN_NAME]:
                            for obj in object_dictionary[lr.parameter1_type]:
                                if obj.name.split("_")[0] not in non_auxilary_object_set:
                                    aux3 = Const(parameter1_type=lr.parameter1_type,
                                                parameter2_type=lr.parameter2_type,
                                                cr = lr.cr,
                                                id = 3,
                                                parameter=obj.name)
                                
                                    auxilary_set.add(aux3)
                        
        return auxilary_set

    def get_domain_pddl(self, domain_name="Keva",edited_action_dict={},additional_constants=[],actions_to_use=None,create_domain_flag=True):
        Config.DOMAIN_NAME = domain_name
        domain_file_name = self.env_prefix+self.file_prefix+"{}_{}_domain.pddl".format(self.test_structure,self.plank_count)
        if create_domain_flag:
            s = "(define (domain {})".format(self.domain)
            s += "\n(:requirements :strips :typing :equality :conditional-effects :existential-preconditions :universal-preconditions)\n"

            types_string = "(:types \n"
            for obj_type in self.object_dictionary.keys():
                types_string += "\t{}\n".format(obj_type)
            types_string+=")\n"
            
            constants_string = ""
            if Config.DOMAIN_NAME != "Packing":
                constants_string = "\n(:constants \n"
                for constant_type in Config.CONST_TYPES[Config.DOMAIN_NAME]:
                    constants_string += "\t{}_Const - {}\n".format(*((constant_type+",")*4).split(",")[:-1])
                for constant in additional_constants:
                    if constant.split("_")[0] not in Config.CONST_TYPES[Config.DOMAIN_NAME]:
                        constants_string += "\t{} - {}\n".format(constant,constant.split("_")[0])
                constants_string += " )\n"
            
            predicate_string = "\n(:predicates \n"
            predicates = self.og_lifted_relations_dict.keys()
            for p in predicates:
                for p_val in self.og_lifted_relations_dict[p].values():
                    predicate_string += "\t{}\n".format(p_val.__str__())

            aux_predicate_set = set()
            for ap in self.aux_list:
                if ap.id == 3:
                    s_ap = str(ap).split()[0] + " ?x - {}".format(ap.parameter1_type)
                elif ap.id == 4:
                    s_ap = str(ap).split()[0] + " ?x - {}".format(ap.parameter2_type)
                else:
                    s_ap = str(ap)
                
                aux_predicate_set.add(s_ap)

            for s_ap in aux_predicate_set:        
                predicate_string += "\t({})\n".format(s_ap)

            predicate_string += ")\n\n"

            if actions_to_use is None:
                actions_to_use = self.actions

            action_string = ""
            for a in actions_to_use:
                if a.action_id not in edited_action_dict:
                    action_string += "{}\n".format(a.__str__())
                else:
                    action_string += "{}\n".format(edited_action_dict[a.action_id].__str__())

            s = s + types_string + constants_string + predicate_string + action_string + "\n)"

            if not os.path.exists(Config.PDDL_DIR):
                os.makedirs(Config.PDDL_DIR)

            with open(Config.PDDL_DIR+domain_file_name,"w") as f:
                f.writelines(s)
                f.close()
        
        else:
            if not os.path.isfile(Config.PDDL_DIR+domain_file_name) and os.path.isfile(Config.PDDL_DIR+"domain.pddl"):
                with open(Config.PDDL_DIR+"domain.pddl","r") as f:
                    default_domain = f.readlines()
                    f.close()

                with open(Config.PDDL_DIR+domain_file_name,"w") as f:
                    f.writelines(default_domain)
                    f.close()

        return True
    
    def get_problem_pddl(self,init_state, goal_state,additional_constants=[]):
        if self.test_structure is not None:
            problem_name = "{}_{}".format(self.test_structure,self.plank_count)
        else:
            problem_name = "{}_{}".format(self.domain,self.plank_count)
        
        additional_constants_list = additional_constants
        # for link_type in additional_constants_dict.keys():
        #     for link in additional_constants_dict[link_type]:
        #         additional_constants_list.append(link)

        if self.use_problem_file is not True:
            s = "(define (problem {})\n".format(problem_name)
            s+= "(:domain {})".format(self.domain)

            object_string = "(:objects \n"
            for obj_type in self.object_dictionary.keys():
                for obj in self.object_dictionary[obj_type]:
                    if obj_type not in Config.CONST_TYPES[Config.DOMAIN_NAME] and obj.name not in additional_constants_list:
                        object_string += "\t{} - {}\n".format(obj, obj_type)
            object_string += ")"

            s += object_string

            init_state_string = "\n(:init \n \t{} \n )".format(init_state.__str__())

            remove_re = set()
            for re in goal_state.true_set:
                if re.cr == 0 and not (re.parameter1_type in Config.OBJECT_NAME and re.parameter2_type in Config.OBJECT_NAME) and not (re.parameter1_type in Config.ROBOT_TYPES or re.parameter2_type in Config.ROBOT_TYPES):
                    remove_re.add(re)

            for re in remove_re:
                goal_state.true_set.remove(re)
                
            remove_aux_re = copy.deepcopy(goal_state.aux_true_set)
            for re in remove_aux_re:
                goal_state.aux_true_set.remove(re)
            
            goal_state_string = "\n(:goal \n \t(and {}) \n )".format(goal_state.__str__())

            s = s + init_state_string + goal_state_string + "\n )"
            
            if not os.path.exists(Config.PDDL_DIR):
                os.makedirs(Config.PDDL_DIR)

            with open(Config.PDDL_DIR+self.env_prefix+self.file_prefix+problem_name+".pddl","wb") as f:
                f.writelines(s)
            f.close()

        return problem_name

    def get_action_list(self,action_strings,current_planner):
        grounded_solutions = []
        for actions in action_strings:
            lifted_action_list = []
            grounded_action_list = []
            for a_string in actions:
                action_id = int(a_string[1:-1].split()[0][1:])
                parameters = a_string[1:-1].split()[1:]
                for action in self.actions:
                    if action.action_id == action_id:
                        lifted_action_list.append(action)
                        break
                grounding = { }
                for i,param in enumerate(action.parameters):
                    for l in self.object_dictionary[param.type]:
                        if current_planner == "FF":
                            name = l.name.upper()
                        else:
                            name = l.name
                        if name.upper() == parameters[i].upper():
                            grounding[param.pid] = l
                            break
                grounded_action_list.append(action.get_grounded_action(grounding,lifted_action_id=action_id))
            
            grounded_solutions.append(grounded_action_list)

        return grounded_solutions

    def get_state_list(self,init_env_state,action_list):
        init_pddl_state = get_abstract_state(env_state=init_env_state,
                                             object_dictionary=self.object_dictionary,
                                             lifted_relations_dictionary=self.new_lifted_relations_dict,
                                             aux_list = self.aux_list)

        state_list = [init_pddl_state]
        next_state = init_pddl_state
        for action in action_list:
            next_state = action.apply(next_state)
            state_list.append(next_state)
        
        return state_list

    # @blockPrinting
    def call_planner(self,problem_name,planner,num_plans=Config.K_PLANS):
        print("planning")
        domain_file_dir = Config.PDDL_DIR+self.env_prefix+self.file_prefix+"{}_{}_domain.pddl".format(self.test_structure,self.plank_count)
        problem_file = Config.PDDL_DIR+self.env_prefix+self.file_prefix+problem_name+".pddl"
        std_out_file = Config.PDDL_DIR+self.env_prefix+self.file_prefix+"seed_{}_".format(self.seed)+problem_name+"_planner_out.txt"
        
        solutions_dir = getattr(Config,"{}_SOLUTION_DIR".format(planner)) + "{}traj_count/".format(self.file_prefix)
        if not os.path.exists(solutions_dir):
            os.makedirs(solutions_dir)
            
        cd_flag = False
        curdir = None
        
        if planner=="FD":
            success_string = "Solution found."
            cd_flag = True
            curdir = os.getcwd()
            os.chdir(solutions_dir)
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
            os.chdir(solutions_dir)
            # command = "{} {} {} --search \"symq-bd(simple=true,plan_selection=top_k(num_plans={},dump_plans=false),quality=1.0)\"".format(Config.KP_FILE,domain_file_dir,problem_file,num_plans)
            command = "{} {} {} --search \'kstar(celmcut(),k={},q=1.0)\'".format(Config.KP_FILE,domain_file_dir,problem_file,num_plans)
        # args = shlex.split(command)
        # args = command.split(" ")

        p = Popen(command,shell=True,stdout=open(std_out_file,"w"))
            
        if cd_flag: 
            os.chdir(curdir)

        return p, std_out_file
        
    def save_traj_data(self,ll_plan):
        serialized_plan = []
        for ll_transition in ll_plan:
            action_id, robot, traj, grab_bool, _, req_grounded_pose,env_state, grabbed_object,static_object, rel,_ = ll_transition
            if traj is not None:
                if type(traj) == list:
                    traj_list = []
                    for t in traj:
                        serialized_traj = t.serialize()
                        traj_list.append(serialized_traj)
                    serialized_traj = traj_list
                else:
                    serialized_traj = traj.serialize()

                serialized_plan.append(serialized_traj)

            elif grab_bool is not None:
                if type(grab_bool) == list:
                    grab_list = []
                    for g in grab_bool:
                        if type(g) == bool:
                            grab_list.append(g)
                        else:
                            serialized_traj = g.serialize()
                            grab_list.append(serialized_traj)
                    grab_bool = grab_list

                serialized_plan.append(grab_bool)

        with open(Config.ALT_PLAN_DIR+"serialized_traj_{}_{}.p".format(Config.DOMAIN_NAME,self.test_structure),"wb") as f:
            cPickle.dump(serialized_plan,f,protocol=cPickle.HIGHEST_PROTOCOL)
            f.close()

    def get_ros_plan(self,ll_plan):
        ROSPlan = []
        for i, ll_transition in enumerate(ll_plan):
            action_id, robot, target_pose, grab_bool, _, req_grounded_pose,env_state, grabbed_object,static_object, rel,_ = ll_transition
            if target_pose is not None:
                if len(target_pose) == 4:
                    robot_name = "freight"
                else:
                    robot_name = "gripper"
            
            elif grab_bool is not None:
                robot_name = "gripper"

            Type = None
            SubType = ""
            if robot_name == "freight":
                Type = "BaseNavigation"
                if rel.cr == 0:
                    SubType = "skip"
            else:
                if grab_bool is not None:
                    if grab_bool:
                        Type = "GripperClose"
                    else:
                        Type = "GripperOpen"
                else:
                    Type = "Manipulation" 
            
            traj = None
            if target_pose is not None:
                traj = target_pose
            else:
                traj = grab_bool
        
            if traj is not None:
                # env_state = self.sim_object.execute_refinement(traj=traj,robot=robot,obj_name=grabbed_object,lock=False,move_gripper=True)
                self.sim_object.set_env_state(env_state)
            
            gripper_poses = []
            base_poses = []
            if static_object.split("_")[0] in Config.ROBOT_TYPES:
                static_object_pose = env_state.object_dict[static_object][1]
            else:
                static_object_pose = env_state.object_dict[static_object]
            
            is_ik = False
            desired_ik = []
            if (robot_name == "gripper" and rel.cr == 0):
                is_ik = True
                # is_ik = False
                # desired_ik = env_state.object_dict["gripper_{}".format(n+1)][0]
                desired_ik = self.sim_object.grabbed_armTuckDOFs
                # self.sim_object.robot.SetActiveDOFValues(self.sim_object.grabbed_armTuckDOFs)
                # static_object = "yumi_body"
                # static_object_pose = self.sim_object.get_pose_from_transform(self.sim_object.robot.GetLink("yumi_body").GetTransform())
            
            elif (rel.parameter1_type in Config.ROBOT_TYPES and rel.parameter2_type in Config.ROBOT_TYPES and rel.cr == 1):
                is_ik = True
                # is_ik = False
                desired_ik = self.sim_object.grabbed_armTuckDOFs
                # self.sim_object.robot.SetActiveDOFValues(self.sim_object.grabbed_armTuckDOFs)
                # static_object = "yumi_body"
                # static_object_pose = self.sim_object.get_pose_from_transform(self.sim_object.robot.GetLink("yumi_body").GetTransform())

            for n in range(env_state.num_robots):
                # ik_link_pose = self.sim_object.get_pose_from_transform(self.sim_object.ik_link.GetTransform())
                # gripper_poses.append(self.sim_object.get_relative_pose(static_object_pose,ik_link_pose))
                # gripper_poses.append(env_state.object_dict["gripper_{}".format(n+1)][1])
                gripper_poses.append(self.sim_object.get_relative_pose(static_object_pose,env_state.object_dict["gripper_{}".format(n+1)][1]))
                
                if Config.DOMAIN_NAME == "CafeWorld" or Config.DOMAIN_NAME == "DinnerTable":
                    # base_poses.append(env_state.object_dict["freight_{}".format(n+1)][1])
                    base_poses.append(self.sim_object.get_relative_pose(static_object_pose,env_state.object_dict["freight_{}".format(n+1)][1]))
            
            act = ROSAction(id=action_id,
                            robot=robot_name,
                            gripper_pose=gripper_poses,
                            base_pose=base_poses,
                            Type=Type,
                            SubType=SubType,
                            grabbed_object = grabbed_object,
                            base_frame=static_object,
                            is_ik=is_ik,
                            desired_ik=desired_ik)

            ROSPlan.append(act)

        # for i,current_action in enumerate(ROSPlan):
        #     prev_action = None
        #     next_action = None

        #     if i > 0:
        #         prev_action = ROSPlan[i-1]
        #     if i < len(ROSPlan)-1:
        #         next_action = ROSPlan[i+1]
            
        #     if next_action is not None:
        #         if next_action.Type == "GripperClose":
        #             current_action.SubType = "PreGrasp"
        #         elif next_action.Type == "GripperOpen":
        #             current_action.SubType = "PreDrop"

        #     if prev_action is not None:
        #         if prev_action.Type == "GripperOpen":
        #             current_action.SubType = "PostDrop"
        #         elif prev_action.Type == "GripperClose":
        #             current_action.SubType = "PostGrasp"               

            # current_action.SubType = SubType

        ROSMsgPlan = []
        for i,act in enumerate(ROSPlan):
            action_msg_object = Action()
            action_msg_object.id = act.id
            action_msg_object.type = act.Type
            action_msg_object.subtype = act.SubType
            action_msg_object.grasped_object = act.grabbed_object
            action_msg_object.base_frame = act.base_frame
            action_msg_object.desired_ik = act.desired_ik
            action_msg_object.is_ik = act.is_ik
            
            gripper_pose_t = transform_from_pose(act.gripper_pose[0])
            if len(act.base_pose)>0:
                base_pose_t = transform_from_pose(act.base_pose[0])
            else:
                base_pose_t = np.ones((4,4))

            action_msg_object.target_gripper_pose = get_pose_stamped(act.base_frame,gripper_pose_t)
            action_msg_object.target_base_pose = get_pose_stamped(act.base_frame,base_pose_t)
            
            ROSMsgPlan.append(action_msg_object)

            # (or (pg p3 goalConst) (not (= ?plank  plank4 )))        
        plan = Plan()
        plan.plan = ROSMsgPlan 
        
        self.get_pose_stamped_for_goals(self.test_structure+"_structure")
        with open(Config.ROS_WS_DIR+"ROSPlan_{}_{}.p".format(Config.DOMAIN_NAME,self.test_structure),"wb") as f:
            cPickle.dump(plan,f,protocol=cPickle.HIGHEST_PROTOCOL)
            f.close()          

        print("ROSPlan saved")
    
    def execute_ll_plan(self,ll_plan):        
        for i,ll_transition in enumerate(ll_plan):
            action_id,rob,target_pose,grab_bool,object_to_grab,req_grounded_pose,exact_current_env_state,grabbed_object,static_object,rel, eef_transform = ll_transition
            if type(target_pose) == list:
                if len(target_pose) < 4:
                    traj = None
                    count = 0
                    while traj is None and count < Config.MP_MAX_COUNT:
                        traj = self.sim_object.compute_motion_plan(goal=target_pose[:-1],robot=rob.robot)
                        count += 1
                else:
                    total_attempts = 0
                    while total_attempts < 10:
                        ik_count = 0
                        sampled_config = []
                        while ik_count < Config.MAX_IK_ATTEMPTS and len(sampled_config) == 0:
                            sampled_config = rob.get_ik_solutions(eef_transform,robot_param="gripper",collision_fn = self.sim_object.collision_check)
                            ik_count += 1

                        if len(sampled_config) != 0:
                            traj = None
                            count = 0
                            while traj is None and count < Config.MP_MAX_COUNT:
                                traj = self.sim_object.compute_motion_plan(goal=target_pose[:-1],robot=rob.robot)
                                count += 1
                        
                        total_attempts += 1
                        if traj is not None:
                            break
                
            elif grab_bool is not None:
                traj = grab_bool

            else:
                if target_pose is not None:
                    traj = target_pose
                else:
                    traj = grab_bool
            
            print(i+1)
            if traj is not None:
                self.sim_object.execute_refinement(traj=traj,robot=rob,obj_name=object_to_grab,lock=False)
            else:
                print("no trajectory found")
                return False

        print("Execution Completed")
        return True

    def load_model(self,model_num):
        with open(DATA_MISC_DIR+self.file_prefix+"{}_{}.p".format(Config.DOMAIN_NAME,model_num),"rb") as f:
            data = cPickle.load(f)
            f.close()
        
        return data

    def save_model(self,model_num,name=None):
        if model_num == 0:
            model_dict = {}
            model_dict["og_clusters"] = set(self.transition_clusters.keys())
            model_dict["new_clusters"] = set([])
            model_dict["low_level_transitions"] = []
        else:
            model_dict = self.load_model(model_num)
            
        model_dict["rcrs"] = self.data
        model_dict["og_relations"] = self.og_relations
        model_dict["actions"] = self.actions

        if name is None:
            with open(Config.DATA_MISC_DIR+self.file_prefix+"{}_{}.p".format(Config.DOMAIN_NAME,model_num),"wb") as f:
                cPickle.dump(model_dict,f,protocol=cPickle.HIGHEST_PROTOCOL)
                f.close()

            if model_num == 0:
                with open(Config.DATA_MISC_DIR+self.file_prefix+"{}_{}.p".format(Config.DOMAIN_NAME,1),"wb") as f:
                    cPickle.dump(model_dict,f,protocol=cPickle.HIGHEST_PROTOCOL)
                    f.close()
        
        else:
            if not os.path.exists(Config.DATA_MISC_DIR+"{}problem_models".format(self.file_prefix)):
                os.makedirs(Config.DATA_MISC_DIR+"{}problem_models".format(self.file_prefix))

            with open(Config.DATA_MISC_DIR+"{}problem_models/".format(self.file_prefix)+"{}_{}.p".format(Config.DOMAIN_NAME,name),"wb") as f:
                cPickle.dump(model_dict,f,protocol=cPickle.HIGHEST_PROTOCOL)
                f.close()
        
        return True

    def update_model(self,init_model_num,model_num,low_level_transitions,save=True):
        if init_model_num == 0:
            model_num = 1
            self.save_model(init_model_num)

        init_model = self.load_model(init_model_num)
        if "learnt_clusters" not in init_model.keys():
            init_model["learnt_clusters"] = set([])

        if "og_clusters" not in init_model.keys():
            init_model["og_clusters"] = set(self.transition_clusters.keys())

        og_clusters = init_model["og_clusters"]
        current_model = {}
        current_model["rcrs"] = self.data
        current_model["og_relations"] = self.og_relations
        current_model["actions"] = self.actions
        current_model["low_level_transitions"] = []
        
        learnt_clusters = init_model["learnt_clusters"]
        relation_union = copy.deepcopy(current_model["rcrs"])

        new_low_level_transitions = copy.deepcopy(low_level_transitions)
        new_abstracted_transitions = []
        new_clusters = {}

        new_low_level_transitions.extend(copy.deepcopy(init_model["low_level_transitions"]))       

        for transition in new_low_level_transitions:
            prev_env_state,next_env_state = transition 

            object_list = self.get_object_list_from_env_state(prev_env_state)
            object_dictionary = self.get_object_dictionary(object_list=object_list)
            lifted_relations_dict = self.og_relations
            aux_list = self.get_auxilary_preds(object_dictionary=object_dictionary,lifted_relations_dict=lifted_relations_dict)

            prev_state = get_abstract_state(env_state=prev_env_state,
                                            object_dictionary=object_dictionary,
                                            lifted_relations_dictionary=lifted_relations_dict,
                                            aux_list = aux_list)
            next_state = get_abstract_state(env_state=next_env_state,
                                            object_dictionary=object_dictionary,
                                            lifted_relations_dictionary=lifted_relations_dict,
                                            aux_list = aux_list)
            
            new_abstracted_transitions.append((prev_state,next_state))
        
        new_clusters = self.generate_transition_clusters(generate=True,trans=new_abstracted_transitions)
        new_cluster_keys = set(new_clusters.keys())
        
        cluster_keys_to_add = new_cluster_keys.difference(og_clusters.union(learnt_clusters))
        
        new_actions = self.actions
        added_relations_dict_to_use = copy.deepcopy(self.added_relations)
        print("checking for new actions")
        for cluster_key in cluster_keys_to_add:
            for current_state,next_state in new_clusters[cluster_key]:
                if next_state not in added_relations_dict_to_use.keys():
                    new_added_relations = set([])
                else:
                    new_added_relations = set(added_relations_dict_to_use[next_state])

                added_relations = self.get_local_added_relations(new_added_relations,current_state,next_state)
                new_added_relations = new_added_relations.union(added_relations)

                if current_state not in added_relations_dict_to_use.keys():
                    added_relations_dict_to_use[current_state] = set([])
                added_relations_dict_to_use[next_state] = new_added_relations
            
            action = LiftedPDDLAction.get_action_from_cluster(new_clusters[cluster_key],added_relations_dict_to_use)
            if action not in new_actions:
                action.action_id = len(new_actions)+1
                new_actions.append(action)
                learnt_clusters.add(cluster_key)
       
        new_model = {}
        new_model["actions"] = new_actions
        new_model["rcrs"] = relation_union
        new_model["og_relations"] = self.og_relations
        new_model["og_clusters"] = og_clusters
        new_model["low_level_transitions"] = new_low_level_transitions
        new_model["learnt_clusters"] = learnt_clusters

        if save:
            print("saving model")
            with open(Config.DATA_MISC_DIR+self.file_prefix+"{}_{}.p".format(Config.DOMAIN_NAME,model_num),"wb") as f:
                cPickle.dump(new_model,f,protocol=cPickle.HIGHEST_PROTOCOL)
                f.close()
        
            return model_num
        
        else:
            return new_model
        
    def get_object_list_from_env_state(self,env_state,objects_not_found=[]):
        object_name_list = []
        for obj in env_state.object_dict.keys():
            if obj not in objects_not_found:
                object_name_list.append(obj)
        
        return object_name_list

    def save_ll_state_pair(self,init_state,goal_state,traj_config,prefix=0,objects_to_move=-1):
        data = {
            "init_state": init_state,
            "goal_state": goal_state,
            "traj_config": traj_config,
            "objects_to_move": objects_to_move,
        }
        with open(Config.PROBLEM_STATES_DIR+"{}_problem.p".format(prefix),"wb") as f:
            cPickle.dump(data,f,protocol=cPickle.HIGHEST_PROTOCOL)
            f.close()

    def load_ll_state_pair(self,prefix=0):
        with open(Config.PROBLEM_STATES_DIR+"{}_problem.p".format(prefix)) as f:
            data = cPickle.load(f)
            f.close()
        
        return data

    def get_additional_constants(self,edited_actions_dict):
        additional_constants = []
        if Config.DOMAIN_NAME in ["Keva","Jenga"]:
            for act in edited_actions_dict.values():
                for planks in act.required_planks:
                    additional_constants.extend(planks)
        # else:
        #     for obj_type in self.object_dictionary:
        #         for obj in self.object_dictionary[obj_type]:
        #             additional_constants.append(obj.name)

        return list(set(additional_constants))

    def modify_actions(self,action_to_edit,required_planks,edited_actions_dict,state_to_neglect):
        faulty_action_id = action_to_edit
        lifted_action = None
        if faulty_action_id not in edited_actions_dict.keys():
            for ac in self.actions:
                if ac.action_id == faulty_action_id:
                    lifted_action = copy.deepcopy(ac)
                    break
        else:
            lifted_action = edited_actions_dict[faulty_action_id]
        
        lifted_action_required_planks = set([])
        lifted_action_states_to_neglect = set([])
        if Config.DOMAIN_NAME in ["Keva","Jenga"]:
            lifted_action_required_planks = lifted_action.required_planks
            lifted_action_required_planks = set(lifted_action_required_planks).union(set(required_planks))
        else:
            lifted_action_states_to_neglect = lifted_action.states_to_neglect
            lifted_action_states_to_neglect.add(state_to_neglect)

        new_lifted_action = LiftedPDDLAction(id=faulty_action_id,
                                             parameters=lifted_action.parameters,
                                             preconditions=lifted_action.preconditions,
                                             effects=lifted_action.effects,
                                             required_planks=lifted_action_required_planks,
                                             states_to_neglect=lifted_action_states_to_neglect)

        edited_actions_dict[faulty_action_id] = new_lifted_action

        return edited_actions_dict

    def rename_objects(self,env_state):
        new_object_dict = {}

        bowl_targetLoc = []
        bowl_initLoc = []
        bowl = []

        glass_targetLoc = []
        glass_initLoc = []
        glass = []

        for obj_name in env_state.object_dict.keys():
            if "bowl" in obj_name:
                if "targetLoc" in obj_name:
                    new_object_dict["bowltargetLoc_{}".format(len(bowl_targetLoc)+1)] = env_state.object_dict[obj_name]
                    bowl_targetLoc.append(obj_name)
                    continue
                if "initLoc" in obj_name:
                    new_object_dict["bowlinitLoc_{}".format(len(bowl_initLoc)+1)] = env_state.object_dict[obj_name]
                    bowl_initLoc.append(obj_name)
                    continue
                new_object_dict["bowl_{}".format(len(bowl)+1)] = env_state.object_dict[obj_name]
                bowl.append(obj_name)

            elif "glass" in obj_name:
                if "targetLoc" in obj_name:
                    new_object_dict["glasstargetLoc_{}".format(len(glass_targetLoc)+1)] = env_state.object_dict[obj_name]
                    glass_targetLoc.append(obj_name)
                    continue
                if "initLoc" in obj_name:
                    new_object_dict["glassinitLoc_{}".format(len(glass_initLoc)+1)] = env_state.object_dict[obj_name]
                    glass_initLoc.append(obj_name)
                    continue
                new_object_dict["glass_{}".format(len(glass)+1)] = env_state.object_dict[obj_name]
                glass.append(obj_name)

            else:
                new_object_dict[obj_name] = env_state.object_dict[obj_name]

        env_state.object_dict = new_object_dict
        
        return env_state

    def get_objects_to_move(self,init_state,goal_state):
        init_object_status = []
        goal_object_status = []
        init_object_placement = {}

        objects_to_move = 0

        if Config.DOMAIN_NAME == "CafeWorld" or Config.DOMAIN_NAME == "Packing":
            for re in init_state.true_set:
                if re.cr == 1 and re.parameter1_type == "can" and re.parameter2_type == "surface":
                    init_object_status.append(re)
                    init_object_placement[re.parameter1] = re.parameter2
            
            for re in goal_state.true_set:
                if re.cr == 1 and re.parameter1_type == "can" and re.parameter2_type == "surface":
                    goal_object_status.append(re)
        
        elif "Keva" in Config.DOMAIN_NAME:
            for re in init_state.true_set:
                if re.cr == 1 and re.parameter1_type == "goalLoc" and re.parameter2_type == "plank":
                    init_object_status.append(re)
                    init_object_placement[re.parameter2] = re.parameter1
            
            for re in goal_state.true_set:
                if re.cr == 1 and re.parameter1_type == "goalLoc" and re.parameter2_type == "plank":
                    goal_object_status.append(re)
        
        elif "Jenga" in Config.DOMAIN_NAME:
            for re in init_state.true_set:
                if re.cr == 1 and re.parameter1_type == "goalLoc" and re.parameter2_type == "jenga":
                    init_object_status.append(re)
                    init_object_placement[re.parameter2] = re.parameter1
            
            for re in goal_state.true_set:
                if re.cr == 1 and re.parameter1_type == "goalLoc" and re.parameter2_type == "jenga":
                    goal_object_status.append(re)

        for re in goal_object_status:
            if re not in init_object_status:
                objects_to_move += 1
        
        return objects_to_move, init_object_placement

    def get_placed_objects(self,init_object_placement,current_state):
        placed_object_count = 0

        if Config.DOMAIN_NAME == "CafeWorld":
            for re in current_state.true_set:
                if re.cr == 1 and re.parameter1_type == "can" and re.parameter2_type == "surface":
                    if init_object_placement[re.parameter1] != re.parameter2:
                        placed_object_count += 1

        elif "Keva" in Config.DOMAIN_NAME:
            for re in current_state.true_set:
                if re.cr == 1 and re.parameter1_type == "goalLoc" and re.parameter2_type == "plank":
                    if re.parameter2 not in init_object_placement.keys():
                        placed_object_count += 1
        
        elif "Jenga" in Config.DOMAIN_NAME:
            for re in current_state.true_set:
                if re.cr == 1 and re.parameter1_type == "goalLoc" and re.parameter2_type == "jenga":
                    if re.parameter2 not in init_object_placement.keys():
                        placed_object_count += 1

        elif Config.DOMAIN_NAME == "Packing":
            for re in current_state.true_set:
                if re.cr == 1 and re.parameter1_type == "can" and re.parameter2_type == "surface":
                    if re.parameter1 not in init_object_placement.keys():
                        placed_object_count += 1
        
        return placed_object_count

    def set_camera(self):
        t = np.load(Config.CAMERA_TRANSFORM_FILE)
        self.sim_object.env.GetViewer().SetCamera(t)

    def get_pose_stamped_for_goals(self,structure_name):
        goal_dict = {}
        drop_t = self.sim_object.env.GetKinBody("droparea").GetTransform()
        for goal in self.sim_object.goalLoc_list:
            t = goal.GetTransform()
            relative_t = get_relative_transform(drop_t,t)
            pose_stamped = get_pose_stamped("droparea",relative_t)
            goal_dict[str(goal.GetName())] = pose_stamped
    
        with open(Config.ROS_WS_DIR+"{}.p".format(structure_name),"wb") as f:
            cPickle.dump(goal_dict,f,protocol=cPickle.HIGHEST_PROTOCOL)
            f.close()

    def get_plank_to_pick(self,grounded_action,kept_planks_set):
        for re in grounded_action.effect.add_set:
            if re.parameter1_type in Config.ROBOT_TYPES and re.parameter2_type in Config.OBJECT_NAME and re.cr > 0:
                if int(re.parameter2.split("_")[1]) not in kept_planks_set:
                    return self.sim_object.env.GetKinBody(re.parameter2)
        
        return None

    def load_trajectory_files(self):
        with open(Config.ALT_PLAN_DIR+"serialized_traj_{}_{}.p".format(Config.DOMAIN_NAME,self.test_structure),"rb") as f:
                trajectories = cPickle.load(f)
                f.close()
        
        deserialized_trajectories = []
        for traj in trajectories:
            if type(traj) != bool:
                deserialized_traj = RaveCreateTrajectory(self.sim_object.env, '')
                Trajectory.deserialize(deserialized_traj, traj)

                deserialized_trajectories.append(deserialized_traj)
            else:
                deserialized_trajectories.append(traj)

        return deserialized_trajectories

    def execute_traj(self, traj):
        num = traj.GetNumWaypoints()
        for waypoint in range(num):
            self.sim_object.robot.SetActiveDOFValues(traj.GetWaypoint(waypoint))
            time.sleep(0.1)
        
    def learn_new_actions(self,relations_to_learn):
        learnt_flag = False
        if len(self.new_relations) > 0:
            lifted_transitions = set([])
            self.aux_list = self.get_auxilary_preds(lifted_relations_dict=self.og_lifted_relations_dict)

            for prev_env_state,next_env_state in self.new_ll_transitions.values():
                object_list = self.get_object_list_from_env_state(prev_env_state)
                object_dictionary = self.get_object_dictionary(object_list=object_list)

                prev_state = get_abstract_state(env_state=prev_env_state,
                                                object_dictionary=object_dictionary,
                                                lifted_relations_dictionary=self.og_lifted_relations_dict,
                                                aux_list = self.aux_list)
                next_state = get_abstract_state(env_state=next_env_state,
                                                object_dictionary=object_dictionary,
                                                lifted_relations_dictionary=self.og_lifted_relations_dict,
                                                aux_list = self.aux_list)

                lifted_transitions.add((prev_state,next_state))

            new_clusters = self.generate_transition_clusters(generate=True,trans=lifted_transitions)
            
            new_actions = []
            added_relations_dict_to_use = copy.deepcopy(self.added_relations)
            for cluster in new_clusters.values():
                for current_state,next_state in cluster:
                    if next_state not in added_relations_dict_to_use.keys():
                        new_added_relations = set([])
                    else:
                        new_added_relations = added_relations_dict_to_use[next_state]

                    added_relations = self.get_local_added_relations(new_added_relations,current_state,next_state)
                    new_added_relations = new_added_relations.union(added_relations)

                    if current_state not in added_relations_dict_to_use.keys():
                        added_relations_dict_to_use[current_state] = set([])
                    added_relations_dict_to_use[next_state] = new_added_relations
                
                LiftedPDDLAction.action_id = len(self.actions)
                action = LiftedPDDLAction.get_action_from_cluster(cluster,added_relations_dict_to_use)
                new_actions.append(action)
                self.learnt_action_count += 1

            print("learnt {} new actions".format(len(new_actions)))
            self.actions.extend(new_actions)
            learnt_flag = True

            relations_to_learn = relations_to_learn.difference(self.new_relations)
            self.learnt_relations = self.learnt_relations.union(self.new_relations)
            self.new_relations = set([])

        return learnt_flag, relations_to_learn            
    
    def get_transitions_from_traj(self,traj):
        transition_list = []
        for i in range(len(traj)-1):
            transition_list.append((traj[i],traj[i+1]))

        return transition_list

    def get_lifted_relations_in_state(self,env_state,lifted_relations_dict):
        re_dict = {}
        aux_present = set([])

        aux_list = self.get_auxilary_preds(lifted_relations_dict=lifted_relations_dict)
        obj_list = self.get_object_list_from_env_state(env_state)
        object_dictionary = self.get_object_dictionary(obj_list)

        abstract_state = get_abstract_state(env_state,object_dictionary, lifted_relations_dict, aux_list)

        for key in lifted_relations_dict.keys():
            if key != "{0}_{0}".format(Config.OBJECT_NAME[0]):
                re_dict[key] = copy.deepcopy(lifted_relations_dict[key])
            else:
                re_dict[key] = {}

        if "{0}_{0}".format(Config.OBJECT_NAME[0]) in lifted_relations_dict.keys():
            for re in abstract_state.true_set:
                if re.get_lifted_relation() in lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])].values():
                    re_dict["{0}_{0}".format(Config.OBJECT_NAME[0])][re.cr] = re.get_lifted_relation()
            
            cr_null_region = []
            for re in re_dict["{0}_{0}".format(Config.OBJECT_NAME[0])].values():
                if re.cr != 0:
                    cr_null_region.extend(re.region)
            re_dict["{0}_{0}".format(Config.OBJECT_NAME[0])][0].region = copy.deepcopy(cr_null_region)

        for re in aux_list:
            if re.cr in re_dict["{}_{}".format(re.parameter1_type,re.parameter2_type)].keys():
                aux_present.add(re)
        
        return re_dict, aux_present

    def remove_relations(self,lifted_relations_dict,goal_state):
        if len(self.relations_to_remove) == 0 and len(lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])]) > 0:
            objects = []
            for obj in self.goal_env_state.object_dict.keys():
                if obj.split("_")[0] == Config.OBJECT_NAME[0]:
                    objects.append(obj)
                    
            candidate_relations = self.get_relations_satisfied_by_objects(goal_state,objects)
            candidate_relations = self.filter_relations_using_effects(candidate_relations)
            if len(candidate_relations) == 0:
                candidate_relations = set(lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])].values()).difference(set([re for re in lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])].values() if re.cr == 0]))
            
            self.sort_relations_using_preconditions(candidate_relations)            
        
        if len(self.relations_to_remove) > 0:
            _,relation_to_remove = heapq.heappop(self.relations_to_remove)

            new_0_region = []
            for region in lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])][0].region:
                if region not in relation_to_remove.region:
                    new_0_region.append(region)
            
            lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])][0].region = new_0_region

            for cr in lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])].keys():
                if lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])][cr] == relation_to_remove:
                    del lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])][cr]
                    break

        return lifted_relations_dict

    def get_relations_satisfied_by_objects(self,goal_state,objects):
        obj_dictionary = {}
        lifted_relations_to_consider = []

        for re in goal_state.true_set:
            if re.cr > 0 and re.parameter1_type == re.parameter2_type == Config.OBJECT_NAME[0]:
                obj = re.parameter2    
                if obj not in obj_dictionary:
                    obj_dictionary[obj] = set([])
                obj_dictionary[obj].add(re.get_lifted_relation())
        
        objects = copy.deepcopy(obj_dictionary.keys())
        for obj in objects:
            if len(obj_dictionary[obj]) <= 1:
                del obj_dictionary[obj]
        
        return obj_dictionary.values()

    def filter_relations_using_effects(self,relations):
        candidate_relations = set([])
        for a in self.actions:
            for re_set in relations:
                if len(re_set.intersection(candidate_relations)) == 0:
                    if 0 < len(a.effects.get_lifted_add_set().intersection(re_set)) < len(re_set):
                        candidate_relations = candidate_relations.union(re_set)
        
        return candidate_relations

    def sort_relations_using_preconditions(self,relations):
        relations_count = {}
        for a in self.actions:
            for re in relations:
                if re not in relations_count.keys():
                    relations_count[re] = 0
                if re in a.preconditions.get_lifted_true_set():
                    relations_count[re]+=1
        
        for re in relations_count.keys():
            heapq.heappush(self.relations_to_remove,(relations_count[re],re))
        
        return self.relations_to_remove

    def check_plan_order(self,plan,kept_planks=[]):
        for i, action in enumerate(plan):
            for re in action.effect.add_set:
                if re.parameter1_type in Config.CONST_TYPES[Config.DOMAIN_NAME] and re.cr == 1:
                    plank_num = int(re.parameter2.split("_")[-1])
                    if not (Config.PLANKS_PROBLEM_ORDER[Config.DOMAIN_NAME][self.test_structure][plank_num].issubset(kept_planks)):
                        print(kept_planks+[plank_num])
                        return False
                    kept_planks.append(plank_num)
        
        return True

    def get_num_actions_with_cr(self,cr_list):
        action_ids = set([])
        for ac in self.actions:
            action_add_effects = set([])
            for r in ac.effects.add_set:
                re = r.parent_relation
                if re.parameter1_type == re.parameter2_type == "plank" and re.cr > 0:
                    action_add_effects.add(re.cr)
                    
            if action_add_effects.issubset(cr_list) and len(action_add_effects)>0:
                action_ids.add(ac.action_id)
                continue
        
        print(len(self.actions),len(action_ids),action_ids,cr_list,len(self.og_lifted_relations_dict["plank_plank"]))
        return len(action_ids)

    def get_action(self,id,action_set=None):
        if action_set == None:
            action_set = self.actions
        for a in action_set:
            if a.action_id == id:
                return a

    def testing(self,execute=False,config=None,generate=False,get_domain_flag=False,model_num=0,model_update_flag=False,seed_prefix=0,experiment_flag=False,seed=0):
        self.seed = seed
        start_time = time.time()
        replanning_time_list = []
        replanning_refinement_list = []
        plan_lengths = []
        actions_refined_in_plans = []
        complete_transition_list = []
        
        if self.sim_use:
            if experiment_flag:
                data = self.load_ll_state_pair(prefix=seed_prefix)
                init_env_state = data["init_state"]
                goal_env_state = data["goal_state"]
                traj_config = data["traj_config"]
                if "objects_to_move" in data.keys():
                    objects_to_move = min(self.plank_count,data["objects_to_move"])
                else:
                    objects_to_move = -1
                    
                objects_not_found = self.sim_object.set_env_state(init_env_state)
                self.sim_object.setup_exp(experiment_flag=experiment_flag)
                # if "Keva" in Config.DOMAIN_NAME:
                #     self.sim_object.remove_droparea()
            
            else:
                objects_not_found = []
                if Config.OBJECT_NAME[0] in self.data:
                    rcr_dict = self.data[Config.OBJECT_NAME[0]]
                else:
                    rcr_dict = {}
                init_env_state,goal_env_state,traj_config = self.sim_object.setup_exp(config,rcr_dict=rcr_dict)
                objects_to_move = -1
                self.sim_object.remove_droparea()
            
            # self.save_ll_state_pair(init_env_state,goal_env_state,traj_config,prefix=seed_prefix,objects_to_move=objects_to_move)  
            # self.sim_object.set_camera_wrt_obj("yumi",1)
            # self.sim_object.env.GetKinBody("droparea").SetTransform(np.eye(4))

            object_name_list = self.get_object_list_from_env_state(init_env_state,objects_not_found)
            self.init_env_state = init_env_state
            self.goal_env_state = goal_env_state
            if self.sim_object.env.GetViewer() is not None:
                if Config.DOMAIN_NAME == "Keva":
                    self.sim_object.set_camera_wrt_obj("yumi",1)
                elif Config.DOMAIN_NAME == "CafeWorld":
                    self.sim_object.set_camera_wrt_obj("world_final",1)
                elif Config.DOMAIN_NAME == "Packing":
                    self.sim_object.set_camera_wrt_obj("table6",2)

        else:
            with open(Config.DATA_MISC_DIR+"env_data.p","rb") as f:
                env_data = cPickle.load(f)
                f.close()
            
            object_name_list = env_data["object_name_list"]
            init_state = env_data["init_state"]
            goal_state = env_data["goal_state"]
        
        self.object_name_list = object_name_list
        init_model_num = copy.deepcopy(model_num)
        latest_model_num = copy.deepcopy(init_model_num)

        new_actions_learnt_flag = False
        self.object_dictionary=self.get_object_dictionary(object_name_list) 

        if model_num == 0:
            # if "Keva" in Config.DOMAIN_NAME or "CafeWorld" in Config.DOMAIN_NAME or "Jenga" in Config.DOMAIN_NAME:
            # else:
            #     self.data = self.load_data()

            self.data,self.env_traj_dict = self.load_data()
            self.og_lifted_relations_dict = self.get_lifted_relations_dict(object_name_list)
            self.new_lifted_relations_dict = copy.deepcopy(self.og_lifted_relations_dict)    
            
            if self.aux_list is None:
                self.aux_list = self.get_auxilary_preds()
            
            if not(self.graph_loaded):
                self.transitions_used = self.get_graph(generate=generate)

            self.transition_clusters = self.generate_transition_clusters(generate=generate)
            self.actions = self.gen_actions(self.transition_clusters)
            self.og_lifted_relations_dict = self.get_lifted_relations_dict(object_name_list)
            self.new_lifted_relations_dict = copy.deepcopy(self.og_lifted_relations_dict)
            self.og_relations = copy.deepcopy(self.og_lifted_relations_dict)
            
            model_num += 1

        else:
            model_objects = self.load_model(model_num)
            self.og_relations = model_objects["og_relations"]
            self.data = copy.deepcopy(model_objects["rcrs"])
            # self.transition_clusters = copy.deepcopy(model_objects["base_transition_clusters"])
            self.transition_clusters = self.generate_transition_clusters()
            self.actions = model_objects["actions"]
            self.og_lifted_relations_dict = self.get_lifted_relations_dict(object_name_list)
            _,self.new_lifted_relations_dict = self.add_relations_to_dict(relations_dict=copy.deepcopy(self.og_lifted_relations_dict),
                                                                          relations=self.relations_to_learn)

            if self.aux_list is None:
                self.aux_list = self.get_auxilary_preds()
            
            if not(self.graph_loaded):
                self.transitions_used = self.get_graph(generate=generate)

        current_num_actions = len(self.actions)
        init_relations_count = {}
        for prim_object in self.data.keys():
            for sec_object in self.data[prim_object].keys():
                init_relations_count["{}_{}".format(prim_object,sec_object)] = len(self.data[prim_object][sec_object])

        # self.save_model(0)
        # # self.save_model(1)
        # exit(0)

        if self.sim_use:
            init_state = get_abstract_state(env_state=init_env_state,
                                            object_dictionary=self.object_dictionary,
                                            lifted_relations_dictionary=self.og_lifted_relations_dict,
                                            aux_list = self.aux_list)
        
            goal_state = get_abstract_state(env_state=goal_env_state,
                                            object_dictionary=self.object_dictionary,
                                            lifted_relations_dictionary=self.og_lifted_relations_dict,
                                            aux_list = self.aux_list)
            
            # t = np.eye(4)
            # t[2,3] = -0.05
            # planks_to_move = []
            # for re in init_state.true_set:
            #     if re.parameter1_type == "plank" and re.parameter2_type == "plank" and re.cr > 0:
            #         planks_to_move.append(re.parameter2)
            
            # if len(planks_to_move) > 0 :
            #     print("INIT_STATE_FAULT")
            #     return None
            
            # for p in planks_to_move:
            #     plank = self.sim_object.env.GetKinBody(p)
            #     plank.SetTransform(t.dot(plank.GetTransform()))

            # t = np.eye(4)
            # for obj in self.sim_object.env.GetBodies():
            #     if "plank" in str(obj.GetName()):
            #         t[2,3] = -int(str(obj.GetName()).split("_")[1])*0.1
            #         obj.SetTransform(t.dot(obj.GetTransform()))
            
            # init_env_state = self.sim_object.get_current_state()

            # init_env_state = self.sim_object.set_planks_at_goalLoc(list(range(1,9)))

            # self.new_goal_relations,self.problem_object_pairs = ModelUpdater.find_missing_relations(goal_state,goal_env_state,self.og_lifted_relations_dict,self.data,self.object_dictionary,1)
            _,self.problem_object_pairs = ModelUpdater.find_missing_relations(goal_state,goal_env_state,self.og_lifted_relations_dict,self.data,self.object_dictionary,1)
            objects_transitioned, init_object_placement = self.get_objects_to_move(init_state,goal_state)

            env_data = {"object_name_list":object_name_list,
                        "init_state":init_state,
                        "goal_state":goal_state}

            with open(Config.DATA_MISC_DIR+"env_data.p","wb") as f:
                cPickle.dump(env_data,f,protocol=cPickle.HIGHEST_PROTOCOL)
                        
        kp_flag = False

        plan_flag = True
        relations_to_learn = copy.deepcopy(self.relations_to_learn)
        partial_ordering_count = 0
        get_problem_flag = True
        edited_actions_dict = copy.deepcopy(self.edited_actions_dict)
        problem_name=""
        total_plans = 0
        k_multiplier = 1

        while plan_flag:
            if self.planner != "PO" or partial_ordering_count > self.partial_order_plans:
                plan_flag = False
            else:
                partial_ordering_count += 1
            
            if self.planner == "KP":
                kp_flag = True
            
            self.goal_relations, self.goal_aux_list = self.get_lifted_relations_in_state(goal_env_state,self.og_lifted_relations_dict)
            # self.get_num_actions_with_cr(self.goal_relations["plank_plank"].keys())
            # exit(0)
            
            replan = True
            if self.sim_use:
                while replan:
                    replan = False
                    init_state = get_abstract_state(env_state=init_env_state,
                                                    object_dictionary=self.object_dictionary,
                                                    lifted_relations_dictionary=self.og_lifted_relations_dict,
                                                    aux_list = self.aux_list)

                    goal_state = get_abstract_state(env_state=goal_env_state,
                                                    object_dictionary=self.object_dictionary,
                                                    lifted_relations_dictionary=self.goal_relations,
                                                    aux_list = self.goal_aux_list)
                    
                    planner = self.planner
                    if kp_flag:
                        planner = "FF" 
                    if self.planner == "PO":
                        planner = "FF"

                    plan_init_time = time.time()
                    k_plans = 0

                    start_i = total_plans
                    if planner == "KP":
                        end_i = k_multiplier*Config.K_PLANS
                    else:
                        end_i = start_i

                    plan_generator,problem_name = self.get_plan(init_pddl_state=init_state,
                                                                goal_pddl_state=goal_state,
                                                                get_domain_flag=get_domain_flag,
                                                                get_problem_flag=get_problem_flag,
                                                                problem_name=problem_name,
                                                                use_plan_file=self.use_plan_file,
                                                                planner=planner,
                                                                start_i=start_i,
                                                                end_i=end_i,
                                                                k_multiplier=k_multiplier)
                    
                    refinement_start = time.time()
                    validation_flag = False
                    refinement_check_flag = False
                    actions = []
                    stop_iteration_flag = False
                    while True:
                        try:
                            [actions] = plan_generator.next()         
                        except StopIteration:
                            stop_iteration_flag = True
                            # break               

                        if len(actions) == 0 and "{0}_{0}".format(Config.OBJECT_NAME[0]) in self.goal_relations and len(self.goal_relations["{0}_{0}".format(Config.OBJECT_NAME[0])]) > 1:
                            print("removing goal relations")
                            self.goal_relations = self.remove_relations(self.goal_relations,goal_state)
                            replan = True
                            kp_flag = True
                            break
                            
                        elif len(actions) == 0:
                            if "{0}_{0}".format(Config.OBJECT_NAME[0]) in self.goal_relations and len(self.goal_relations["{0}_{0}".format(Config.OBJECT_NAME[0])]) == 1:
                                print("all relations already removed")

                            print("PROBLEM UNSOLVABLE")
                            break                          

                        if stop_iteration_flag:
                            break

                        total_plans += 1
                        k_plans += 1
                        print("current plan num: {}".format(total_plans))
                        if planner == "FF":
                            actions = actions[:-1]

                        plan_lengths.append(len(actions))

                        validation_flag, plan, relations_to_learn,low_level_transitions,faulty_action_id,required_planks,last_state,actions_executed,env_state_list = self.check_plan_refinement(init_env_state=init_env_state,
                                                                                                                                                                                                 plan=actions,
                                                                                                                                                                                                 relations_to_learn=relations_to_learn,
                                                                                                                                                                                                 init_object_placement=init_object_placement,
                                                                                                                                                                                                 objects_to_move=objects_to_move)

                        env_transitions = self.get_transitions_from_traj(env_state_list)
                        complete_transition_list.extend(env_transitions) 
                        
                        actions_refined_in_plans.append(actions_executed)
                        # learnt_flag,relations_to_learn = self.learn_new_actions(relations_to_learn)

                        if validation_flag:
                            if execute:
                                refinement_check_flag = self.execute_ll_plan(plan)
                            else:
                                refinement_check_flag = True
                        
                        current_refinement_time = time.time() - refinement_start
                        replanning_refinement_list.append(current_refinement_time)
                        
                        if validation_flag and refinement_check_flag:
                            print("refinable plan")    
                            if self.planner == "PO":
                                plan_flag = False
                            
                            if self.p is not None:
                                if self.p.poll() is None:
                                    self.p.kill()

                            break
                        
                        else:
                            if plan == -1 or plan == 2:
                                print("incorrect sequence")
                                missing_relations_string = "relations to be learnt -> "
                                for re in relations_to_learn:
                                    missing_relations_string += str(re)
                                    missing_relations_string += ", "
                                print(missing_relations_string)
                                
                                if kp_flag:
                                    kp_flag = False
                                    replan = True

                                if self.planner == "PO":
                                    edited_actions_dict = self.modify_actions(action_to_edit=faulty_action_id,required_planks=required_planks,edited_actions_dict=edited_actions_dict,state_to_neglect=last_state)
                                    additional_constants = self.get_additional_constants(edited_actions_dict)
                                    self.get_domain_pddl(domain_name=self.domain,edited_action_dict=edited_actions_dict,additional_constants=additional_constants)
                                    self.get_problem_pddl(init_state=init_state,goal_state=goal_state,additional_constants=additional_constants)
                                    get_problem_flag = False
                                    get_domain_flag = False
                                    plan_flag = True

                                continue_flag = False
                                if self.planner == "KP":
                                    k_multiplier = int(total_plans/Config.K_PLANS)+1

                                # elif self.planner == "PO":
                                #     continue_flag = partial_ordering_count < (self.partial_order_plans - 1)
                                    
                                # if continue_flag:
                                #     continue
                                # break
                                    
                    if total_plans <= self.total_plans:
                        add_flag = False
                        # candidate_relation = self.get_relation_to_add(complete_transition_list)
                        candidate_relation = None
                        if candidate_relation is not None:
                            total_plans = 0
                            k_multiplier = 1
                            print("adding relation")
                            plan_flag = not (validation_flag and refinement_check_flag) 
                            add_flag,self.og_lifted_relations_dict = self.add_relations_to_dict(self.og_lifted_relations_dict,[candidate_relation])
                            self.relations_from_goal.add(candidate_relation)
                            if add_flag:
                                if Config.OBJECT_NAME[0] not in self.data.keys():
                                    self.data[Config.OBJECT_NAME[0]] = {}
                                if Config.OBJECT_NAME[0] not in self.data[Config.OBJECT_NAME[0]].keys():
                                    self.data[Config.OBJECT_NAME[0]][Config.OBJECT_NAME[0]] = []
                                    
                                self.data[Config.OBJECT_NAME[0]][Config.OBJECT_NAME[0]].append(candidate_relation.region)

                                print("model updating")
                                latest_model = self.update_model(init_model_num,model_num,complete_transition_list,save=False)
                                self.actions = latest_model["actions"]
                                self.data = latest_model["rcrs"]
                                env_object_list = self.get_object_list_from_env_state(self.sim_object.get_current_state())
                                self.og_lifted_relations_dict = self.get_lifted_relations_dict(env_object_list)
                        elif not (validation_flag and refinement_check_flag):
                            print("no relation yet to add")
                            replan = True                                                   
                    else:
                        print("exhausted opions")
        
        #adding few more relations
        # if validation_flag and refinement_check_flag and Config.DOMAIN_NAME in ["Keva","Jenga"] and model_update_flag:
        #     relations = self.get_freq_relations(2)
        #     for re in relations:
        #         if re not in self.relations_from_goal:
        #             if re.parameter1_type not in self.data.keys():
        #                 self.data[re.parameter1_type] = {}
        #             if re.parameter2_type not in self.data[re.parameter1_type].keys():
        #                 self.data[re.parameter1_type][re.parameter2_type] = []
                        
        #             self.data[re.parameter1_type][re.parameter2_type].append(re.region)
        
        if validation_flag and refinement_check_flag:
            solved_plans_path = Config.SOLVED_PLAN_FILES+"{}/{}/".format(Config.DOMAIN_NAME,self.total_demo_count)
            if not os.path.exists(solved_plans_path):
                os.makedirs(solved_plans_path)
            
            actions_used_set = set([])
            for ac in actions:
                actions_used_set.add(ac.lifted_action_id)

            if not os.path.exists(solved_plans_path):
                os.makedirs(solved_plans_path)

            file_name =  "seed_{}_problems.json".format(self.seed)
            
            if os.path.isfile(solved_plans_path + file_name):
                with open(solved_plans_path+file_name,"r") as f:
                    problem_dict = json.load(f)
                    f.close()
            else:
                problem_dict = {}
            
            problem_dict[str(seed_prefix)] = list(actions_used_set)

            dump_json_object = json.dumps(problem_dict)

            with open(solved_plans_path + file_name,"w") as f:
                f.write(dump_json_object)
                f.close()
            
            print("actions_set_saved")

        if model_update_flag:
            print("final model saving")
            self.get_domain_pddl(domain_name=self.domain)
            latest_model_num = self.update_model(init_model_num,model_num,complete_transition_list)
        
        self.save_model(model_num=1,name="{}_{}".format(self.test_structure,self.plank_count))

        #Deleting the plan files
        if not self.keep_plans:
            # if len(action_list) != 0:
                planner = self.planner
                if self.planner == "PO":
                    planner = "FD"
                file_ = getattr(Config,"{}_SOLUTION_DIR".format(planner))
                
                if self.planner == "FD":                
                    shutil.rmtree(file_,ignore_errors=True)
                    
                elif self.planner == "FF":
                    if self.test_structure is not None:
                        problem_name = "{}_{}".format(self.test_structure,self.plank_count)
                    else:
                        problem_name = "{}_{}".format(self.domain,self.plank_count)
                    
                    file_ += self.env_prefix+self.file_prefix+problem_name+".pddl.soln"
                    if os.path.isfile(file_):
                        os.remove(file_)
                
                elif self.planner == "KP":
                    shutil.rmtree(file_,ignore_errors=True)

        #Adding the learnt relations in the DAG
        total_num_actions = len(self.actions)
        learnt_actions = self.learnt_action_count

        current_relations_count = {}
        for prim_object in self.data.keys():
            for sec_object in self.data[prim_object].keys():
                current_relations_count["{}_{}".format(prim_object,sec_object)] = len(self.data[prim_object][sec_object])

        learnt_relations = {}
        for object_pairs in init_relations_count.keys():
            if current_relations_count[object_pairs] != init_relations_count[object_pairs]:
                learnt_relations[object_pairs] = current_relations_count[object_pairs] - init_relations_count[object_pairs]

        if self.learnt_action_count > 0:
            print("new action and relation learnt")
            print("creating new domain file")
            self.get_domain_pddl(domain_name=self.domain)

            print("saving critical regions data")
            data_dict = {
                "rcr_dict":self.data,
                "env_traj_dict": self.env_traj_dict
            }
            with open(Config.DATA_MISC_DIR+self.file_prefix+"rcr_indices.p" ,"wb") as f:
                cPickle.dump(data_dict,f,protocol=cPickle.HIGHEST_PROTOCOL)
                f.close()
                    
        log_dict = {
                    "total_plan_time": sum(replanning_time_list),
                    "total_refinement_time": sum(replanning_refinement_list),
                    "replanning_time_list":replanning_time_list,
                    "replanning_refinement_list":replanning_refinement_list,
                    "total_time": time.time() - start_time,
                    "actions_refined_in_plans": actions_refined_in_plans

                }
        
        # self.get_ros_plan(plan)
        # self.save_traj_data(plan)

        if self.sim_use:
            if refinement_check_flag:
                if execute and not experiment_flag:
                    raw_input("execute?")
                    self.execute_ll_plan(plan)

                return True, total_plans, plan_lengths, learnt_actions, learnt_relations, traj_config, log_dict, latest_model_num
            
            else:
                return False, total_plans, plan_lengths, learnt_actions, learnt_relations, traj_config, log_dict, latest_model_num
        else:
            return False, total_plans, plan_lengths, learnt_actions, learnt_relations, traj_config, log_dict, latest_model_num
