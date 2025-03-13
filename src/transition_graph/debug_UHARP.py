# from openravepy import *
from src.data_structures.discretizer import Discretizer
from Config import *
from src.data_structures.TransitionGraph import TransitionGraph 
# from src.data_structures.RaveExecution import GetRaveExecution
from src.data_structures.PDDLAction import LiftedPDDLAction
from src.data_structures.Relation import Relation
from src.data_structures.ChangedRelations import ChangedRelations
from src.data_structures.Const import Const
from src.data_structures.Link import Link
from time import sleep
from src.useful_functions import *
import Config
import pickle 
import os
import tqdm
import copy
import importlib

class LAMP(object): 
    def __init__(self,robot_name,env_list,test_structure,test_env_name,axis_for_offset,visualize=False,plank_count=1,random=False,planner="FF",use_problem_file=False,planks_in_init_state=0,k_plans=None,num_robots=1,use_plan_file=False,keep_plans=False,mp=True,sim_use=True): 
        self.env_list = env_list
        self.test_env_name = test_env_name
        
        self.file_name = "debug_rcr_indices.p"
        self.data = self.load_data()
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
        
        self.sim_object = None
        
        self.best_state = None
        self.non_applicable_dictionary = {}
        self.action_refinement_dictionary = {}
        self.test_structure = test_structure
        if "Keva" in Config.DOMAIN_NAME:
            self.test_structure_dag,self.unknown_plank_relations = self.load_dag(test_structure)
        self.lifted_relations_dict = {}
        self.planner = planner
        self.use_problem_file = use_problem_file
        self.k_plans = k_plans
        self.failed_planks = {}
        self.use_plan_file = use_plan_file
        self.keep_plans = keep_plans

    def load_data(self):
        print("loading {} data".format(self.env_list))
        data_load = pickle.load(open(Config.DATA_MISC_DIR+self.file_name))
        return data_load
    
    def load_dag(self,structure_name,create=False):
        try:
            with open(Config.DAG_DIR + structure_name + "_dag.p","rb") as f:
                data = pickle.load(f)
                f.close()
        except:
            create = True
        
        if create:
            discretizer = Config.get_discretizer(obj1="plank",obj2="plank")
            data=create_dag(structure=structure_name,
                            cr_list=self.data["plank"]["plank"],
                            discretizer=discretizer)
        
        return data["dag"],data["unknown_plank_relations"]
        
    def get_lifted_relations_dict(self,env_object_list):
        lifted_relations_dict = {}
        for link1 in self.data.keys(): 
            for link2 in self.data[link1].keys(): 
                link1_list = []
                link2_list = []
                for obj in env_object_list:
                    if obj.split("_")[0] == link1:
                        link1_list.append(obj)
                    if obj.split("_")[0] == link2:
                        link2_list.append(obj)
                        
                lifted_regions = self.data[link1][link2]
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
            with open(Config.DATA_MISC_DIR+"debug_transition_graph.p") as f:
                transition_graph._g = pickle.load(f)
            
        except:
            generate = True
        
        if generate:
            transition_graph = TransitionGraph()
            print("generating graph")
            for i,env in enumerate(self.env_list):
                print("loading {} trajectory data".format(env))
                with open(Config.DATA_MISC_DIR+env+"/"+env+"_data.p") as f:
                    traj_data = pickle.load(f)
                
                if i == 0:
                    self.lifted_relations_dict = self.get_lifted_relations_dict(traj_data["env_states"][0][0].object_dict.keys()) 
                    object_dictionary = self.get_object_dictionary(traj_data["env_states"][0][0].object_dict.keys())

                print("using {} trajectories".format(env))
                for traj in tqdm.tqdm(traj_data['env_states']):
                # for traj in traj_data['env_states'][:2]:
                    prev_state = get_abstract_state(env_state=traj[0],
                                                    object_dictionary=object_dictionary,
                                                    lifted_relations_dictionary=self.lifted_relations_dict,
                                                    aux_list = self.aux_list)
                    if prev_state not in transition_graph._g.nodes:
                        transition_graph.add_state(prev_state)

                    for env_state in traj[1:]:
                        next_state = get_abstract_state(env_state=env_state,
                                                        object_dictionary=object_dictionary,
                                                        lifted_relations_dictionary=self.lifted_relations_dict,
                                                        aux_list = self.aux_list)
                        if ((prev_state,next_state) not in transition_graph._g.edges) and (prev_state != next_state):
                            if next_state not in transition_graph._g.nodes:
                                transition_graph.add_state(next_state)
                            transition_graph.add_transition(prev_state,next_state)
                        prev_state = next_state
                                                
            print("num nodes {}".format(len(list(transition_graph._g.nodes))))
            print("num edges {}".format(len(transition_graph._g.edges)))
            transition_graph.save()

        self.transition_graph = transition_graph
        self.graph_loaded = True
        return True
    
    @blockPrinting
    def get_plan(self,init_pddl_state,goal_pddl_state,get_domain_flag=False,use_plan_file=False):
        if get_domain_flag:
            self.get_domain_pddl(domain_name=self.domain)

        problem_name = self.get_problem_pddl(init_state=init_pddl_state,
                                             goal_state=goal_pddl_state)

        if not use_plan_file:
            self.call_planner(problem_name=problem_name)

        solution = []
        if self.planner == "FD":
            plan = []
            with open(Config.FD_SOLUTION_FILE,"r") as f:
                solution_file = f.readlines()
                for l in solution_file[:-1]:
                    plan.append(l.strip("\n"))
            solution.append(plan)

        elif self.planner == "FF":
            plan = []
            with open(Config.FF_SOLUTION_DIR+problem_name+".pddl.soln","r") as f:
                solution_file = f.readlines()
                for l in solution_file[1:]:
                    plan.append(l.strip("\n"))
            solution.append(plan)

        elif self.planner == "KP":
            for p in range(1,self.k_plans+1):
                plan = []
                with open(Config.KP_SOLUTION_FILE+".{}".format(p),"r") as f:
                    solution_file = f.readlines()
                    for l in solution_file[:-1]:
                        plan.append(l.strip("\n"))
                solution.append(plan)
                
        if len(solution[0]) == 0:
            return []

        action_list = self.get_action_list(solution)
        
        return action_list
    
    def get_variable_string(self,variable_list):
        s = ""
        for var in variable_list:
            s+=var.__str__()
            s+=", "
        
        return s
    
    def gen_actions(self,clusters):
        actions = []       

        for cluster in clusters.keys():
            action = LiftedPDDLAction.get_action_from_cluster(clusters[cluster])
            actions.append(action)
        
        return actions
    
    def generate_transition_clusters(self,generate=False):
        try:
            print("loading clusters")                
            with open(Config.DATA_MISC_DIR+"debug_transition_clusters.p") as f:
                transition_clusters = pickle.load(f)
            
        except:
            generate = True
        
        if generate:
            print("generating clusters")
            transition_clusters = {}
            for transition in self.transition_graph._g.edges:
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
                        a = copy.deepcopy(a_re)
                        if a.id == 3:
                            a.parameter = a.parameter1_type
                        if a.id == 4:
                            a.parameter = a.parameter2_type

                        added_auxilaries.append(a)
                    
                for a_re in state1.aux_true_set:
                    if a_re not in state2.aux_true_set:
                        a = copy.deepcopy(a_re)
                        if a.id == 3:
                            a.parameter = a.parameter1_type
                        if a.id == 4:
                            a.parameter = a.parameter2_type

                        deleted_auxilaries.append(a)

                added_relations.sort()
                deleted_relations.sort()
                deleted_auxilaries.sort()
                added_auxilaries.sort()

                lifted_added_relations = []
                for re in added_relations:
                    lifted_added_relations.append(re.get_lifted_relation())

                lifted_deleted_relations = []
                for re in deleted_relations:
                    lifted_deleted_relations.append(re.get_lifted_relation())

                lifted_added_relations.sort()
                lifted_deleted_relations.sort()

                changed_relations_key = ChangedRelations(changed_lifted_added_relations=lifted_added_relations,
                                                         changed_lifted_deleted_relations=lifted_deleted_relations,
                                                         added_auxilary_relations=added_auxilaries,
                                                         deleted_auxilary_relations=deleted_auxilaries)
                
                if changed_relations_key not in transition_clusters.keys():
                    transition_clusters[changed_relations_key] = []
                
                transition_clusters[changed_relations_key].append(transition)

            with open(Config.DATA_MISC_DIR+"debug_transition_clusters.p","wb") as f:
                pickle.dump(transition_clusters,f,protocol=pickle.HIGHEST_PROTOCOL)

        return transition_clusters
    
    # @blockPrinting
    def check_plan_refinement(self,init_env_state,plan,relation_to_learn=None):
        action_instance_list = {}
        print("REFINING plan with len {}".format(len(plan)))
        refined_plan = []
        if len(plan) == 0:
            return False
        env_state = init_env_state
        env_state_list = [env_state]
        self.sim_object.set_env_state(init_env_state)
        print("Plan ->")
        for ac in plan:
            print(ac)

        kept_planks_set = set()
        c_state = get_abstract_state(env_state=env_state,
                                    object_dictionary=self.object_dictionary,
                                    lifted_relations_dictionary=self.lifted_relations_dict,
                                    aux_list = self.aux_list)
        for re in c_state.true_set:
            if "goalLoc" in re.parameter1 and re.cr != 0:
                kept_planks_set.add(int(re.parameter2.split("_")[1]))
            elif "goalLoc" in re.parameter2 and re.cr != 0:
                kept_planks_set.add(int(re.parameter1.split("_")[1]))

        validation_flag = True
        i = 0
        learnt_action = None
        while i < len(plan):
            grounded_action = plan[i]
            print(i,len(env_state_list))
            
            current_state = get_abstract_state(env_state=env_state,
                                               object_dictionary=self.object_dictionary,
                                               lifted_relations_dictionary=self.lifted_relations_dict,
                                               aux_list = self.aux_list)

            if i not in action_instance_list:
                action_instance_list[i] = {}

            traj,grab_bool,(rel,instance,instance_count),rob = grounded_action.compute_refinement(env_state=env_state,
                                                                                                  sim_object=self.sim_object,
                                                                                                  previous_instances=action_instance_list[i],
                                                                                                  compute_motion_plan = self.compute_motion_plan)
            if rel is None and instance is None:
                if i != 0:
                    action_instance_list[i] = {}
                    i -= 1 
                    validation_flag = True
                else:
                    validation_flag = False
                    break

                env_state = copy.deepcopy(env_state_list[i])
                env_state_list = copy.deepcopy(env_state_list[:-1])
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
                env_state = self.sim_object.execute_refinement(robot=rob,traj=traj)

            obj_name=None
            if grab_bool is not None:
                for relation in grounded_action.changed_relations:
                    if relation.parameter1.split("_")[0] not in Config.IMMOVABLE_OBJECTS and relation.parameter1_type not in Config.ROBOT_TYPES.keys():
                        obj_name = relation.parameter1
                        break
                    elif relation.parameter2.split("_")[0] not in Config.IMMOVABLE_OBJECTS and relation.parameter2_type not in Config.ROBOT_TYPES.keys():
                        obj_name = relation.parameter2
                        break
                env_state = self.sim_object.execute_refinement(robot=rob,traj=grab_bool,obj_name=obj_name)
            
            next_state = get_abstract_state(env_state=env_state,
                                            object_dictionary=self.object_dictionary,
                                            lifted_relations_dictionary=self.lifted_relations_dict,
                                            aux_list = self.aux_list)

            for re in grounded_action.effect.add_set:
                if re not in next_state.true_set:
                    validation_flag = False
                    print("{} not in add".format(re))
            
            for re in grounded_action.effect.delete_set:
                if re not in next_state.false_set:
                    validation_flag = False
                    print("{} not in delete".format(re))

            for re in next_state.true_set:
                if re not in current_state.true_set:
                    if relation_to_learn is not None:
                        if re.get_lifted_relation() == relation_to_learn:
                            print("the transition to add -> {}".format(grounded_action))
                            _current_state = copy.deepcopy(current_state)
                            _next_state = copy.deepcopy(next_state)
                            transition = (_current_state, _next_state)
                            learnt_action = self.add_action_to_model(transition)

            if validation_flag is not True:
                env_state = copy.deepcopy(env_state_list[i])
                validation_flag = True
                continue
            
            i += 1
            if "Keva" in Config.DOMAIN_NAME:
                newly_kept_plank = set()
                for re in next_state.true_set:
                    if "goalLoc" in re.__str__() and re.cr != 0:
                        if re.parameter1_type == "plank":
                            plank_num = int(re.parameter1.split("_")[1])
                            if plank_num not in kept_planks_set:
                                newly_kept_plank.add(plank_num)
                        if re.parameter2_type == "plank":
                            plank_num = int(re.parameter2.split("_")[1])
                            if plank_num not in kept_planks_set:
                                newly_kept_plank.add(plank_num)

                for p in newly_kept_plank:
                    if not (Config.PLANKS_PROBLEM_ORDER[Config.DOMAIN_NAME][self.test_structure][p].issubset(kept_planks_set)):
                        validation_flag = False
                        relation_to_learn = self.get_relation_from_DAG(p)
                        return validation_flag, -1, relation_to_learn
                
                kept_planks_set = copy.deepcopy(kept_planks_set.union(newly_kept_plank))
            
            refined_plan.append((rob,traj,grab_bool,obj_name))
            env_state_list.append(env_state)

        if validation_flag:
            print("Validation Completed")
            self.sim_object.set_env_state(init_env_state)
            return validation_flag,refined_plan, None

        else:
            print("Validation Failed")
            self.sim_object.set_env_state(init_env_state)
            return validation_flag, [], None
    
    #TODO: make this learning more general interms of object names
    def get_relation_from_DAG(self,plank_num):        
        plank_name = "plank_" + str(plank_num)
        parameter2_type = "plank"
        parameter1_type = "plank"

        if plank_name not in self.failed_planks.keys():
            _,edge = heapq.heappop(self.unknown_plank_relations[plank_name])
            region = edge[2]
            cr = len(self.lifted_relations_dict["plank_plank"].keys())
            discretizer = Config.get_discretizer(obj1=parameter1_type,obj2=parameter2_type)

            relation = Relation(parameter1_type=parameter1_type,
                                parameter2_type=parameter2_type,
                                cr = cr,
                                region = region,
                                discretizer=discretizer)
            
            self.lifted_relations_dict["plank_plank"][cr] = relation
            self.lifted_relations_dict["plank_plank"][0].region.extend(region)
            self.failed_planks[plank_name] = [edge,relation]
            self.data["plank"]["plank"].append(region)
            
            self.test_structure_dag = self.load_dag(structure_name=self.test_structure,create=True)

        else:
            _,relation = self.failed_planks[plank_name]       

        return relation

    def add_action_to_model(self,transition):
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

        action = LiftedPDDLAction.get_action_from_cluster([[current_state,next_state]])
        self.actions.append(action)
        print(action)

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

    def get_auxilary_preds(self):
        auxilary_set = set()
        non_auxilary_object_set = Config.get_non_aux_object_set()
        for object_pair in self.lifted_relations_dict: 
            obj1, obj2 = object_pair.split("_")[0],object_pair.split("_")[1]
            # if object_pair != "{}_{}".format(Config.OBJECT_NAME,Config.OBJECT_NAME):
            if not (obj1 in non_auxilary_object_set and obj2 in non_auxilary_object_set):
                for cr in self.lifted_relations_dict[object_pair]: 
                    lr = self.lifted_relations_dict[object_pair][cr]
                    # aux1 = Const(parameter1_type=lr.parameter1_type,
                    #              parameter2_type=lr.parameter2_type,
                    #              cr = lr.cr,
                    #              id = 1)
                    # aux2 = Const(parameter1_type=lr.parameter1_type,
                    #              parameter2_type=lr.parameter2_type,
                    #              cr = lr.cr,
                    #              id = 2)

                    # auxilary_set.add(aux1)
                    # auxilary_set.add(aux2)

                    if lr.parameter1_type not in Config.CONST_TYPES[Config.DOMAIN_NAME] and lr.parameter2_type not in Config.CONST_TYPES[Config.DOMAIN_NAME]:
                        for obj in self.object_dictionary[lr.parameter1_type]:
                            aux3 = Const(parameter1_type=lr.parameter1_type,
                                        parameter2_type=lr.parameter2_type,
                                        cr = lr.cr,
                                        id = 3,
                                        parameter=obj.name)
                            
                            auxilary_set.add(aux3)
                        
                        for obj in self.object_dictionary[lr.parameter2_type]:
                            aux4 = Const(parameter1_type=lr.parameter1_type,
                                        parameter2_type=lr.parameter2_type,
                                        cr = lr.cr,
                                        id = 4,
                                        parameter=obj.name)
                            
                            auxilary_set.add(aux4)

        return auxilary_set

    def get_domain_pddl(self, domain_name="Keva"):
        Config.DOMAIN_NAME = domain_name
        s = "(define (domain {})".format(self.domain)
        s += "\n(:requirements :strips :typing :equality :conditional-effects :existential-preconditions :universal-preconditions)\n"

        types_string = "(:types \n"
        for obj_type in self.object_dictionary.keys():
            types_string += "\t{}\n".format(obj_type)
        types_string+=")\n"

        constants_string = "\n(:constants "
        for constant_type in Config.CONST_TYPES[Config.DOMAIN_NAME]:
            constants_string += "{}_Const - {}".format(*((constant_type+",")*4).split(",")[:-1])
        constants_string += " )\n"
        
        predicate_string = "\n(:predicates \n"
        predicates = self.lifted_relations_dict.keys()
        for p in predicates:
            for p_val in self.lifted_relations_dict[p].values():
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

        action_string = ""
        for a in self.actions:
            action_string += "{}\n".format(a.__str__())

        s = s + types_string + constants_string + predicate_string + action_string + "\n)"
        
        with open(Config.PDDL_DIR+"domain.pddl","w") as f:
            f.writelines(s)
        f.close()

        return True
    
    def get_problem_pddl(self,init_state, goal_state):
        if self.test_structure is not None:
            problem_name = "{}_{}{}".format(self.test_structure,Config.OBJECT_NAME,self.plank_count)
        else:
            problem_name = "{}_{}{}".format(self.domain,Config.OBJECT_NAME,self.plank_count)
        
        if self.use_problem_file is not True:
            s = "(define (problem {})\n".format(problem_name)
            s+= "(:domain {})".format(self.domain)

            object_string = "(:objects \n"
            for obj_type in self.object_dictionary.keys():
                for obj in self.object_dictionary[obj_type]:
                    if obj_type not in Config.CONST_TYPES[Config.DOMAIN_NAME]:
                        object_string += "\t{} - {}\n".format(obj, obj_type)

            object_string += ")"

            s += object_string

            init_state_string = "\n(:init \n \t{} \n )".format(init_state.__str__())

            remove_re = set()
            for re in goal_state.true_set:
                if re.cr == 0 and not (re.parameter1_type == Config.OBJECT_NAME and re.parameter2_type == Config.OBJECT_NAME):
                    remove_re.add(re)

            for re in remove_re:
                goal_state.true_set.remove(re)
                
            remove_aux_re = copy.deepcopy(goal_state.aux_true_set)
            for re in remove_aux_re:
                goal_state.aux_true_set.remove(re)
            
            goal_state_string = "\n(:goal \n \t(and {}) \n )".format(goal_state.__str__())

            s = s + init_state_string + goal_state_string + "\n )"

            with open(Config.PDDL_DIR+problem_name+".pddl","wb") as f:
                f.writelines(s)
            f.close()

        return problem_name

    def get_action_list(self,action_strings):
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
                        if self.planner == "FF":
                            name = l.name.upper()
                        else:
                            name = l.name
                        if name == parameters[i]:
                            grounding[param.pid] = l
                            break
                grounded_action_list.append(action.get_grounded_action(grounding))
            
            grounded_solutions.append(grounded_action_list)

        return grounded_solutions

    def get_state_list(self,init_env_state,action_list):
        init_pddl_state = get_abstract_state(env_state=init_env_state,
                                             object_dictionary=self.object_dictionary,
                                             lifted_relations_dictionary=self.lifted_relations_dict,
                                             aux_list = self.aux_list)

        state_list = [init_pddl_state]
        next_state = init_pddl_state
        for action in action_list:
            next_state = action.apply(next_state)
            state_list.append(next_state)
        
        return state_list

    def call_planner(self,problem_name):
        print("planning")
        domain_file_dir = Config.PDDL_DIR+"domain.pddl"
        problem_file = Config.PDDL_DIR+problem_name+".pddl"
        
        if self.planner=="FD":
            command = "python {} {} {} --search \"lazy_greedy([ff()], preferred=[ff()])\"".format(Config.FD_FILE, domain_file_dir, problem_file)
            # command = "python {} --alias seq-sat-lama-2011 {} {}".format(Config.FD_FILE, domain_file_dir, problem_file)

        elif self.planner=="FF":
            command = "{} -o {} -f {}".format(Config.FF_FILE, domain_file_dir, problem_file)
        
        elif self.planner=="KP":
            command = "{} {} {} --search \"symk-bd(plan_selection=top_k(num_plans={},dump_plans=false))\"".format(Config.KP_FILE,domain_file_dir,problem_file,self.k_plans)

        os.system(command)
        return True
    
    def execute_ll_plan(self,ll_plan):
        raw_input("execute?")
        for i,ll_transition in enumerate(ll_plan):
            robot, target_pose, grab_bool, object_to_grab = ll_transition
            if type(target_pose) == list:
                traj = None
                if target_pose is not None:
                    count = 0
                    while traj is None and count < 5:
                        traj = self.sim_object.compute_motion_plan(goal=target_pose[:-1],robot=robot.robot)
                        count += 1
                
                elif grab_bool is not None:
                    traj = grab_bool
            else:
                if target_pose is not None:
                    traj = target_pose
                else:
                    traj = grab_bool
            
            print(i+1)
            if traj is not None:
                self.sim_object.execute_refinement(traj=traj,robot=robot,obj_name=object_to_grab,lock=False)
            else:
                print("no trajectory found")
                return False

        print("Execution Completed")
        return True

    def testing(self,execute=False,config=None,generate=False,get_domain_flag=False):
        if self.sim_use:
            init_env_state,goal_env_state = self.sim_object.setup_exp(config,rcr_dict=self.data[Config.OBJECT_NAME])
        
            object_name_list = []
            for obj in init_env_state.object_dict.keys():
                object_name_list.append(obj)
            
        else:
            with open(Config.DATA_MISC_DIR+"debug_env_data.p","rb") as f:
                env_data = pickle.load(f)
                f.close()
            
            object_name_list = env_data["object_name_list"]
            init_state = env_data["init_state"]
            goal_state = env_data["goal_state"]        

        self.lifted_relations_dict = self.get_lifted_relations_dict(object_name_list)
        self.object_dictionary=self.get_object_dictionary(object_name_list) 
        
        if self.aux_list is None:
            self.aux_list = self.get_auxilary_preds()
        
        if not(self.graph_loaded):
            self.graph_loaded = self.get_graph(generate=generate)
        
        self.transition_clusters = self.generate_transition_clusters(generate=generate)
        self.actions = self.gen_actions(self.transition_clusters)
        init_len_actions = len(self.actions)
    
        if self.sim_use:
            init_state = get_abstract_state(env_state=init_env_state,
                                            object_dictionary=self.object_dictionary,
                                            lifted_relations_dictionary=self.lifted_relations_dict,
                                            aux_list = self.aux_list)
        
            goal_state = get_abstract_state(env_state=goal_env_state,
                                            object_dictionary=self.object_dictionary,
                                            lifted_relations_dictionary=self.lifted_relations_dict,
                                            aux_list = self.aux_list)
            
            env_data = {"object_name_list":object_name_list,
                        "init_state":init_state,
                        "goal_state":goal_state}

            with open(Config.DATA_MISC_DIR+"env_data.p","wb") as f:
                pickle.dump(env_data,f,protocol=pickle.HIGHEST_PROTOCOL)
            
        action_list = self.get_plan(init_pddl_state=init_state,
                                    goal_pddl_state=goal_state,
                                    get_domain_flag=get_domain_flag,
                                    use_plan_file=self.use_plan_file)
        
        if len(action_list) == 0:
            print("NO SOLUTION FOUND")
            return False

        if self.sim_use:
            relation_to_learn = None
            for a,actions in enumerate(action_list):
                refinable, plan, relation_to_learn = self.check_plan_refinement(init_env_state=init_env_state,
                                                                                plan=actions,
                                                                                relation_to_learn=relation_to_learn)
                
                if refinable:
                    print("refinable plan")
                    break
                
                else:
                    if plan == -1:
                        print("incorrect sequence")
                        print("relation to be learnt -> {}".format(relation_to_learn))
                        if a < len(action_list)-1:
                            continue
                                    
                    print("plan not refinable")
        
        #Deleting the plan files
        if not self.keep_plans:
            if len(action_list) != 0:
                file_ = getattr(Config,"{}_SOLUTION_FILE".format(self.planner))
                
                if self.planner == "FD":                
                    os.remove(file_)
                    
                elif self.planner == "FF":
                    if self.test_structure is not None:
                        problem_name = "{}_{}{}".format(self.test_structure,Config.OBJECT_NAME,self.plank_count)
                    else:
                        problem_name = "{}_{}{}".format(self.domain,Config.OBJECT_NAME,self.plank_count)
                    
                    file_ += problem_name+".pddl.soln"
                
                    os.remove(file_)
                
                elif self.planner == "KP":
                    for p in range(1,self.k_plans+1):
                        suffix = ".{}".format(p)
                        os.remove(file_+suffix)

        #Adding the learnt relations in the DAG
        current_len_actions = len(self.actions)
        if (current_len_actions - init_len_actions) > 0:
            print("new action and relation learnt")
            print("creating new domain file")
            self.get_domain_pddl(domain_name=self.domain)

            print("saving critical regions data")
            with open(Config.DATA_MISC_DIR+"debug_rcr_indices.p" ,"wb") as f:
                pickle.dump(self.data,f,protocol=pickle.HIGHEST_PROTOCOL)
                f.close()
            
            print("saving transition clusters")
            with open(Config.DATA_MISC_DIR+"debug_transition_clusters.p","wb") as f:
                pickle.dump(self.transition_clusters,f,protocol=pickle.HIGHEST_PROTOCOL)
        
        if self.sim_use:
            if refinable:
                if execute:
                    self.execute_ll_plan(plan)

                return True
        else:
            return False