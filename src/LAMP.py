from useful_functions import *
from Config import Config
import cPickle 
import os
import sys
import tqdm
import copy
import time
import shutil
import json
import heapq
from itertools import product

from src.utilities.Discretizer import Discretizer
from src.data_structures.TransitionGraph import TransitionGraph 
from src.data_structures.Relation import Relation
from src.utilities.Planner import Planner
from src.utilities.TransitionClusters import TransitionClusters
from src.data_structures.WorldModel import WorldModel

class LAMP(object): 
    def __init__(self,robot_name,env_list,test_structure,test_env_name,axis_for_offset,total_traj_count,visualize=False,object_count=1,random=False,planner="FF",use_problem_file=False,objects_in_init_state=0,k_plans=None,num_robots=1,use_plan_file=False,keep_plans=False,mp=True,sim_use=True,replan=True,order=False,process_count=100000000000,object_list=["bowl","glass"],experiment_flag=False,real_world_experiment=False,prefix=0):
        self.env_list = env_list
        self.test_env_name = test_env_name

        self.total_traj_count = total_traj_count
        self.prefix = prefix
        self.env_prefix = "{}_".format(env_list[0][3:])
        self.file_prefix = "{}_{}_".format(total_traj_count,prefix)
        # self.data = load_rcrs(self.file_prefix)
        self.object_count = object_count
        self.actions = []
        self.aux_list = None
        self.bound_object_name = Config.BOUND_OBJECT_NAME
        self.transition_graph = TransitionGraph()
        self.graph_loaded = False
        self.object_dictionary = None
        self.domain = Config.DOMAIN_NAME
        self.num_robots = num_robots
        self.compute_motion_plan = mp
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

        TransitionGraph.set_params(self.file_prefix)
        TransitionClusters.set_params(self.file_prefix)
        WorldModel.set_params(self.file_prefix)

        self.test_structure = test_structure
        if self.test_structure is not None:
            problem_name = "{}_{}".format(self.test_structure,self.object_count)
        else:
            problem_name = "{}_{}".format(self.domain,self.object_count)

        self.pddl_domain_name = self.env_prefix+self.file_prefix+"{}_{}_domain.pddl".format(self.test_structure,self.object_count)
        self.pddl_problem_name = problem_name
        Planner.init_planner(id=self.env_prefix,
                             file_prefix=self.file_prefix,
                             problem_name=self.pddl_problem_name,
                             prefix=self.prefix)

        domain_class = get_domain_class(self.domain)
        self.domain_object = domain_class(env_name=test_env_name,
                                          visualize=visualize,
                                          object_count=object_count,
                                          axis_for_offset=axis_for_offset,
                                          reference_structure_name=test_structure,
                                          random=random,
                                          objects_in_init_state=objects_in_init_state,
                                          num_robots=self.num_robots,
                                          order=order,
                                          object_list=object_list,
                                          experiment_flag=experiment_flag,
                                          )

        self.sim_object = self.domain_object.sim_object

        self.best_state = None
        self.non_applicable_dictionary = {}
        self.action_refinement_dictionary = {}
        self.test_structure = test_structure
        self.planner = planner
        self.use_problem_file = use_problem_file
        self.total_plans = k_plans
        self.partial_order_plans = k_plans
        self.failed_planks = {}
        self.use_plan_file = use_plan_file
        self.keep_plans = keep_plans
        self.relations_to_learn = set([])

        self.new_ll_transitions = {}
        self.og_relations = {}
    
    # @blockPrinting
    def init_planning(self,init_pddl_state,goal_pddl_state,start_i, end_i, get_problem_flag=True,problem_name="",get_domain_flag=False,use_plan_file=False,planner=None,k_multiplier=1):
        additional_constants = []
        if get_domain_flag:
            additional_constants = self.get_additional_constants(self.edited_actions_dict)
        self.get_domain_pddl(edited_action_dict=self.edited_actions_dict,additional_constants=additional_constants,create_domain_flag=get_domain_flag)

        if get_problem_flag:
            problem_name = self.get_problem_pddl(init_state=init_pddl_state,
                                                goal_state=goal_pddl_state)

        std_out_file = ""
        if not use_plan_file:
            num_plans = end_i
            if planner == "FF":
                Planner.ff_start_time = time.time()
            if planner == "KP":
                num_plans = self.total_plans

            if planner == "FF" or k_multiplier == 1:                                      
                Planner.init_planning(planner=planner,
                                      start_i=start_i,
                                      end_i=end_i,
                                      num_plans=num_plans,
                                      test_structure=self.test_structure,
                                      object_count=self.object_count)

        return problem_name
    
    def get_kept_planks_in_state(self,env_state):
        kept_planks_set = set()
        c_state = get_abstract_state(env_state=env_state,
                                     object_dictionary=self.object_dictionary,
                                     lifted_relations_dictionary=self.og_lifted_relations_dict,
                                     aux_list=self.aux_list)
        for re in c_state.true_set:
            if re.parameter1_type in Config.CONST_TYPES and re.cr == 1:
                kept_planks_set.add(int(re.parameter2.split("_")[Config.OBJ_ID_IND]))
            elif re.parameter2_type in Config.CONST_TYPES and re.cr == 1:
                kept_planks_set.add(int(re.parameter1.split("_")[Config.OBJ_ID_IND]))

        return kept_planks_set

    # @blockPrinting
    def check_plan_refinement(self,init_env_state,plan,relations_to_learn,init_object_placement={},objects_to_move=-1):
        """
        This method executes the high level plan that is given by off-the-shelf planners.
        """
        action_instance_list = {}
        refined_plan = []
        if len(plan) == 0:
            return False
        env_state = init_env_state
        env_state_list = [env_state]
        required_plank = ""
        self.sim_object.set_env_state(init_env_state)

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
                    env_state = self.domain_object.set_plank(plank_to_pick,real_world_flag=self.sim_object.real_world_exp)
                    time.sleep(0.0001)

            current_env_state = copy.deepcopy(env_state)
            current_state = get_abstract_state(env_state=env_state,
                                               object_dictionary=self.object_dictionary,
                                               lifted_relations_dictionary=self.og_lifted_relations_dict,
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
                    real, og = traj
                    env_state_to_evaluate = self.sim_object.execute_refinement(robot=rob,traj=og)
                    env_state = self.sim_object.execute_refinement(robot=rob,traj=real)
                else:
                    env_state = self.sim_object.execute_refinement(robot=rob,traj=traj)
                    env_state_to_evaluate = env_state

            obj_name=None
            _,_,action_order = action_info
            if grab_bool is not None:
                for relation in grounded_action.changed_relations:
                    if relation.parameter1.split("_")[Config.OBJ_TYPE_IND] not in Config.IMMOVABLE_OBJECTS and relation.parameter1_type not in Config.ROBOT_TYPES.keys():
                        obj_name = relation.parameter1
                        break
                    elif relation.parameter2.split("_")[Config.OBJ_TYPE_IND] not in Config.IMMOVABLE_OBJECTS and relation.parameter2_type not in Config.ROBOT_TYPES.keys():
                        obj_name = relation.parameter2
                        break
                if type(grab_bool) == list:
                    self.sim_object.execute_refinement(robot=rob,traj=False)
                    self.sim_object.execute_refinement(robot=rob,traj=grab_bool[1])
                    env_state_to_evaluate = self.sim_object.execute_refinement(robot=rob,traj=grab_bool[0],obj_name=obj_name,delta_mp=True)

                    self.sim_object.set_env_state(current_env_state)
                    env_state = self.sim_object.execute_refinement(robot=rob,traj=grab_bool[0],obj_name=obj_name)
                else:
                    env_state_to_evaluate = self.sim_object.execute_refinement(robot=rob,traj=grab_bool,obj_name=obj_name)
                    env_state = env_state_to_evaluate
                    
            grabbed_object = obj_name
            
            next_state = get_abstract_state(env_state=env_state_to_evaluate,
                                            object_dictionary=self.object_dictionary,
                                            lifted_relations_dictionary=self.og_lifted_relations_dict,
                                            aux_list = self.aux_list)

            if grab_bool is not None and type(grab_bool) == list:
                env_state = self.sim_object.execute_refinement(robot=rob,traj=grab_bool[1])

            pddl_states_list.append(self.get_next_pddl_state(pddl_states_list[i],grounded_action))

            if i+1 < len(plan):
                state_action_mapping[plan[i+1]] = pddl_states_list[i+1]

            for re in grounded_action.effect.add_set:
                if re not in next_state.true_set:
                    validation_flag = False
                    print("{} not in add".format(re))
            
            if validation_flag:
                last_refined_state = next_state
                if Config.DOMAIN_NAME in ["Keva","Jenga"]:
                    newly_kept_plank = set()
                    for re in next_state.true_set:
                        if "goalLoc" in re.__str__() and re.cr != 0:
                            if re.parameter1_type == Config.OBJECT_NAME[0]:
                                plank_num = int(re.parameter1.split("_")[Config.OBJ_ID_IND])
                                if plank_num not in kept_planks_set:
                                    newly_kept_plank.add(plank_num)
                            if re.parameter2_type == Config.OBJECT_NAME[0]:
                                plank_num = int(re.parameter2.split("_")[Config.OBJ_ID_IND])
                                if plank_num not in kept_planks_set:
                                    newly_kept_plank.add(plank_num)
                    
                    if len(objects_in_collision_list) == 0:
                        for p in newly_kept_plank:
                            if not (Config.PLANKS_PROBLEM_ORDER[Config.DOMAIN_NAME][self.test_structure][p].issubset(kept_planks_set)):
                                validation_flag = False
                                return validation_flag, -1, relations_to_learn, [], grounded_action.lifted_action_id, required_planks, state_action_mapping[grounded_action], i+1, env_state_list
                    else:
                        validation_flag = False
                        plank_to_keep = self.get_plank_to_be_kept(rob,env_state)
                        required_planks = self.get_collision_object_tuple(plank_to_keep,objects_in_collision_list)              
                        return validation_flag, 2, relations_to_learn, [], grounded_action.lifted_action_id, required_planks, state_action_mapping[grounded_action], i+1, env_state_list
            
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
                    print("Refinement Failed")
                    self.sim_object.set_env_state(init_env_state)
                    return validation_flag, 3, relations_to_learn, env_state_list, plan[refined_action_counts[-1] - 1].lifted_action_id, required_planks, state_action_mapping[plan[refined_action_counts[-1] - 1]], i+1, env_state_list

            print("Refinement Completed")
            self.sim_object.set_env_state(init_env_state)
            return validation_flag,refined_plan, relations_to_learn, env_state_list, plan[refined_action_counts[-1] - 1].lifted_action_id, required_planks, state_action_mapping[plan[refined_action_counts[-1] - 1]], i+1, env_state_list

        else:
            print("Refinement Failed")
            self.sim_object.set_env_state(init_env_state)
            if len(refined_action_counts) == 0:
                refined_action_count_index = 0
            else:
                refined_action_count_index = refined_action_counts[-1]
            
            if continue_flag:
                return validation_flag, -1, relations_to_learn, [], plan[refined_action_count_index].lifted_action_id, required_planks, state_action_mapping[plan[refined_action_count_index]], i+1, env_state_list
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

    def get_domain_pddl(self,edited_action_dict={},additional_constants=[],actions_to_use=None,create_domain_flag=True):
        if create_domain_flag:
            domain = self.world_model.get_domain_pddl(edited_action_dict=edited_action_dict,
                                                      additional_constants=additional_constants,
                                                      actions_to_use=actions_to_use)

            self.world_model.save_domain_file(domain_name=self.pddl_domain_name,domain=domain)

        else:
            if not os.path.isfile(Config.PDDL_DIR+self.pddl_domain_name) and os.path.isfile(Config.PDDL_DIR+"domain.pddl"):
                with open(Config.PDDL_DIR+"domain.pddl","r") as f:
                    default_domain = f.readlines()
                    f.close()

                with open(Config.PDDL_DIR+self.pddl_domain_name,"w") as f:
                    f.writelines(default_domain)
                    f.close()

        return True
    
    def get_problem_pddl(self,init_state, goal_state,additional_constants=[]):
        if self.use_problem_file is not True:
            s = "(define (problem {})\n".format(self.pddl_problem_name)
            s+= "(:domain {})".format(self.domain)

            object_string = "(:objects \n"
            for obj_type in get_object_types_from_rcr_dict(self.world_model.rcrs):
                for obj in self.object_dictionary[obj_type]:
                    if obj_type not in Config.CONST_TYPES and obj.name not in additional_constants:
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

            with open(Config.PDDL_DIR+self.env_prefix+self.file_prefix+self.pddl_problem_name+".pddl","wb") as f:
                f.writelines(s)
            f.close()

        return self.pddl_problem_name
    
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

        return list(set(additional_constants))

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

    def get_plank_to_pick(self,grounded_action,kept_planks_set):
        for re in grounded_action.effect.add_set:
            if re.parameter1_type in Config.ROBOT_TYPES and re.parameter2_type in Config.OBJECT_NAME and re.cr > 0:
                if int(re.parameter2.split("_")[Config.OBJ_ID_IND]) not in kept_planks_set:
                    return self.sim_object.get_obj(re.parameter2)
        
        return None

    def remove_relations(self,lifted_relations_dict,goal_state):
        """
        This method is used to relax the goal by updating the set of relations to be used to lift the ll_state.
        """
        if len(self.relations_to_remove) == 0 and len(lifted_relations_dict["{0}_{0}".format(Config.OBJECT_NAME[0])]) > 0:
            objects = []
            for obj in self.goal_env_state.object_dict.keys():
                if obj.split("_")[Config.OBJ_TYPE_IND] == Config.OBJECT_NAME[0]:
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

    def location_relation_check(self,relation):
        return (relation.parameter1_type in Config.CONST_TYPES and \
                relation.parameter2_type in Config.OBJECT_NAME) or \
                (relation.parameter2_type in Config.CONST_TYPES and \
                relation.parameter1_type in Config.OBJECT_NAME)
                
    def check_plan_order(self,plan,kept_planks=[]):
        for i, action in enumerate(plan):
            for re in action.effect.add_set:
                if self.location_relation_check(re) and re.cr == 1:
                    if re.parameter1_type in Config.OBJECT_NAME:
                        plank_num = int(re.parameter1.split("_")[-1])
                    elif re.parameter2_type in Config.OBJECT_NAME:
                        plank_num = int(re.parameter2.split("_")[-1])

                    if not (Config.PLANKS_PROBLEM_ORDER[self.test_structure][plank_num].issubset(kept_planks)):
                        print(kept_planks+[plank_num])
                        return False
                    kept_planks.append(plank_num)
        
        return True

    def get_action(self,id,action_set=None):
        if action_set == None:
            action_set = self.actions
        for a in action_set:
            if a.action_id == id:
                return a    

    def lamp_start(self,execute=False,config=None,generate=False,get_domain_flag=False,model_num=1,model_update_flag=False,problem_prefix=0,experiment_flag=False,prefix=0):
        """
        This method is the implementation of the LAMP algorithm.
        The method is invoked from run_lamp.py
        """
        start_time = time.time()
        replanning_time_list = []
        replanning_refinement_list = []
        plan_lengths = []
        actions_refined_in_plans = []
        complete_transition_list = []
        
        object_name_list = get_object_list_from_env_state(self.sim_object.get_current_state())
        self.object_name_list = object_name_list

        new_actions_learnt_flag = False
        self.object_dictionary=get_object_dictionary(object_name_list) 

        model_num = min(WorldModel.get_latest_model_num(), model_num)
        init_model_num = copy.deepcopy(model_num)
        latest_model_num = copy.deepcopy(init_model_num)

        self.transition_graph, self.added_relations ,self.transitions_used = TransitionGraph.get_graph()
        self.transtion_clusters = TransitionClusters.get_clusters()
        self.world_model = WorldModel.get_model(model_num)
        self.data = self.world_model.rcrs
        self.og_relations = copy.deepcopy(self.world_model.relations)
        self.og_lifted_relations_dict = self.world_model.relations
        self.actions = self.world_model.actions
        self.aux_list = get_auxilary_preds(self.object_dictionary,self.og_lifted_relations_dict)

        if experiment_flag:
            data = self.load_ll_state_pair(prefix=problem_prefix)
            init_env_state = data["init_state"]
            goal_env_state = data["goal_state"]
            traj_config = data["traj_config"]
            if "objects_to_move" in data.keys():
                objects_to_move = min(self.object_count,data["objects_to_move"])
            else:
                objects_to_move = -1
                
            objects_not_found = self.sim_object.set_env_state(init_env_state)
            self.domain_object.setup_exp(experiment_flag=experiment_flag)

            for obj in self.sim_object.get_current_state().object_dict.keys():
                if obj not in init_env_state.object_dict.keys():
                    self.sim_object.remove_obj(obj)
                    
            self.object_dictionary=get_object_dictionary(self.sim_object.get_current_state().object_dict.keys())

        else:
            objects_not_found = []
            req_relations = [self.og_lifted_relations_dict[r] for r in self.og_lifted_relations_dict.keys() if set(r.split("_")) in [set(s) for s in product(Config.OBJECT_NAME,Config.SURFACE_NAME)]]
            if len(req_relations) > 0:
                req_relation = [req_relations[0][r] for r in req_relations[0].keys() if r != 0]
            else:
                req_relation = []
            init_env_state,goal_env_state,traj_config = self.domain_object.setup_exp(config,req_relation=req_relation)
            objects_to_move = -1

        object_name_list = get_object_list_from_env_state(init_env_state,objects_not_found)
        self.init_env_state = init_env_state
        self.goal_env_state = goal_env_state
        if self.sim_object.visualize:
            if Config.DOMAIN_NAME == "Keva":
                self.sim_object.set_camera_wrt_obj("yumi",1)
            elif Config.DOMAIN_NAME == "CafeWorld":
                self.sim_object.set_camera_wrt_obj("world_final",1)
            elif Config.DOMAIN_NAME == "Packing":
                self.sim_object.set_camera_wrt_obj("table6",2)

        current_num_actions = len(self.actions)
        init_relations_count = {}
        for prim_object in self.data.keys():
            for sec_object in self.data[prim_object].keys():
                init_relations_count["{}_{}".format(prim_object,sec_object)] = len(self.data[prim_object][sec_object])

        init_state = get_abstract_state(env_state=init_env_state,
                                        object_dictionary=self.object_dictionary,
                                        lifted_relations_dictionary=self.og_lifted_relations_dict,
                                        aux_list = self.aux_list)
    
        goal_state = get_abstract_state(env_state=goal_env_state,
                                        object_dictionary=self.object_dictionary,
                                        lifted_relations_dictionary=self.og_lifted_relations_dict,
                                        aux_list = self.aux_list)
        
        objects_transitioned, init_object_placement = self.get_objects_to_move(init_state,goal_state)

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
            
            self.goal_relations, self.goal_aux_list = get_lifted_relations_in_state(goal_env_state,self.og_lifted_relations_dict)
            
            replan = True
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

                k_plans = 0

                start_i = total_plans
                if planner == "KP":
                    end_i = k_multiplier*Config.K_PLANS
                else:
                    end_i = start_i

                problem_name = self.init_planning(init_pddl_state=init_state,
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
                        plan = Planner.generator.next() 
                        [actions] = get_action_list(action_strings=[plan],
                                                    model_actions=self.actions,
                                                    object_dictionary=self.object_dictionary,
                                                    planner=planner,
                                                    object_list=get_object_list_from_env_state(init_env_state))      
                    except StopIteration:
                        stop_iteration_flag = True
                        # break               
                    
                    if planner == "FF":
                        actions = actions[:-1]

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

                    plan_lengths.append(len(actions))

                    validation_flag, plan, relations_to_learn,low_level_transitions,faulty_action_id,required_planks,last_state,actions_executed,env_state_list = self.check_plan_refinement(init_env_state=init_env_state,
                                                                                                                                                                                             plan=actions,
                                                                                                                                                                                             relations_to_learn=relations_to_learn,
                                                                                                                                                                                             init_object_placement=init_object_placement,
                                                                                                                                                                                             objects_to_move=objects_to_move)

                    env_transitions = zip(env_state_list[:-1],env_state_list[1:])
                    complete_transition_list.extend(env_transitions) 
                    
                    actions_refined_in_plans.append(actions_executed)

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
                        
                        if Planner.process is not None:
                            if Planner.process.poll() is None:
                                Planner.process.kill()

                        break
                    
                    else:
                        if plan == 2:
                            print("incorrect sequence")
                            missing_relations_string = "relations to be learnt -> "
                            for re in relations_to_learn:
                                missing_relations_string += str(re)
                                missing_relations_string += ", "
                            print(missing_relations_string)
                            
                            if kp_flag:
                                kp_flag = False
                                replan = True

                            continue_flag = False
                            if self.planner == "KP":
                                k_multiplier = int(total_plans/Config.K_PLANS)+1
                        else:
                            break
            
        if validation_flag and refinement_check_flag:
            solved_plans_path = Config.SOLVED_PLAN_FILES+"{}/{}/".format(Config.DOMAIN_NAME,self.total_traj_count)
            if not os.path.exists(solved_plans_path):
                os.makedirs(solved_plans_path)
            
            actions_used_set = set([])
            for ac in actions:
                actions_used_set.add(ac.lifted_action_id)

            if not os.path.exists(solved_plans_path):
                os.makedirs(solved_plans_path)

            file_name =  "prefix_{}_problems.json".format(self.prefix)
            
            if os.path.isfile(solved_plans_path + file_name):
                with open(solved_plans_path+file_name,"r") as f:
                    problem_dict = json.load(f)
                    f.close()
            else:
                problem_dict = {}
            
            problem_dict[str(problem_prefix)] = list(actions_used_set)

            dump_json_object = json.dumps(problem_dict)

            with open(solved_plans_path + file_name,"w") as f:
                f.write(dump_json_object)
                f.close()
            
            print("actions_set_saved")

        if model_update_flag:
            print("final model saving")
            self.world_model.save_domain_file(domain_name=self.pddl_domain_name)
            self.world_model = WorldModel.update_model(self.world_model,complete_transition_list)
        
        WorldModel.save_model(self.world_model,model_num=1,name="{}_{}".format(self.test_structure,self.object_count))

        if not self.keep_plans:
            planner = self.planner
            if self.planner == "PO":
                planner = "FD"
            file_ = getattr(Config,"{}_SOLUTION_DIR".format(planner))
            
            if self.planner == "FD":                
                shutil.rmtree(file_,ignore_errors=True)
                
            elif self.planner == "FF":
                if self.test_structure is not None:
                    problem_name = "{}_{}".format(self.test_structure,self.object_count)
                else:
                    problem_name = "{}_{}".format(self.domain,self.object_count)
                
                file_ += self.env_prefix+self.file_prefix+problem_name+".pddl.soln"
                if os.path.isfile(file_):
                    os.remove(file_)
            
            elif self.planner == "KP":
                shutil.rmtree(file_,ignore_errors=True)

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
            self.world_model.save_domain_file(domain_name=self.pddl_domain_name)

            print("saving critical regions data")
            with open(Config.DATA_MISC_DIR+self.file_prefix+"rcr_indices.p" ,"wb") as f:
                cPickle.dump(self.data,f,protocol=cPickle.HIGHEST_PROTOCOL)
                f.close()
                    
        log_dict = {
                    "total_plan_time": sum(replanning_time_list),
                    "total_refinement_time": sum(replanning_refinement_list),
                    "replanning_time_list":replanning_time_list,
                    "replanning_refinement_list":replanning_refinement_list,
                    "total_time": time.time() - start_time,
                    "actions_refined_in_plans": actions_refined_in_plans
                }
        
        # self.save_traj_data(plan)

        if refinement_check_flag:
            if execute and not experiment_flag:
                raw_input("execute?")
                self.execute_ll_plan(plan)

            return True, total_plans, plan_lengths, learnt_actions, learnt_relations, traj_config, log_dict, latest_model_num
        
        else:
            return False, total_plans, plan_lengths, learnt_actions, learnt_relations, traj_config, log_dict, latest_model_num