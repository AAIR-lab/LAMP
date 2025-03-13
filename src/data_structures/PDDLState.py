from itertools import product
import functools
import Config

@functools.total_ordering
class PDDLState(object):
    def __init__(self,true_set,false_set,aux_true_set,aux_false_set): 
        self.true_set = true_set
        self.false_set = false_set
        self.aux_true_set = aux_true_set
        self.aux_false_set = aux_false_set

    def is_relation_true(self,grounded_relation): 
        if grounded_relation in self.true_set: 
            return True
        else:
            return False
    
    @staticmethod
    def get_from_ll(lifted_relations_dict, object_dict, ll_state, aux_list):
        true_set = set()
        false_set = set()
        aux_true_set = set()

        for object_pair in lifted_relations_dict: 
            for cr in lifted_relations_dict[object_pair]: 
                relation = lifted_relations_dict[object_pair][cr]
                l1 = object_dict[relation.parameter1_type]
                l2 = object_dict[relation.parameter2_type]
                combinations = product(l1,l2)
                for combination in combinations:
                    grounded_relation = relation.get_grounded_relation(combination[0],combination[1])
                    if grounded_relation.evaluate_in_ll_state(ll_state): 
                        true_set.add(grounded_relation)
                    else:
                        false_set.add(grounded_relation)

        for relation in aux_list:
            if relation.id == 1:
                flag = True
                for r in true_set:
                    if r.parameter1_type == relation.parameter1_type and r.parameter2_type == relation.parameter2_type and relation.cr == r.cr:
                        flag = False
                        break
                if flag:
                    aux_true_set.add(relation)

            elif relation.id == 2:
                n1 = len(object_dict[relation.parameter1_type])
                n2 = len(object_dict[relation.parameter2_type])
                c = 0
                for r in true_set:
                    if r.parameter1_type == relation.parameter1_type and r.parameter2_type == relation.parameter2_type and relation.cr == r.cr:
                        c += 1
                if c == n1 * n2:
                    aux_true_set.add(relation)

            elif relation.id == 3:
                flag = True                
                for r in true_set:
                    if (r.parameter1_type == relation.parameter1_type) and (r.parameter2_type == relation.parameter2_type) and (r.cr == relation.cr) and (r.parameter1 == relation.parameter):
                        flag = False
                        break
                if flag:
                    aux_true_set.add(relation)
            
            elif relation.id == 4:
                flag = True
                for r in  true_set:
                    if (r.parameter1_type == relation.parameter1_type) and (r.parameter2_type == relation.parameter2_type) and (r.cr == relation.cr) and (r.parameter2 == relation.parameter):
                        flag = False
                        break
                if flag:
                    aux_true_set.add(relation)
                    
        return PDDLState(true_set,false_set,aux_true_set,set())
    
    def __str__(self):
        s = ""
        true_set_list = list(self.true_set)
        true_set_list.sort()
        for i,prop in enumerate(true_set_list): 
            s += str(prop)
            s += " "
            if i > 0 and i % 4 == 0: 
                s += "\n"
        
        aux_list = list(self.aux_true_set)
        aux_list.sort()
        for i, ap in enumerate(aux_list):
            s += "("+str(ap)+")"
            s += " "
            if i > 0 and i % 4 == 0: 
                s += "\n"
        return s

    def __eq__(self,o):
        if len(self.true_set) != len(o.true_set):
            return False
        if len(self.aux_true_set) != len(o.aux_true_set):
            return False

        for prop in self.true_set: 
            if prop not in o.true_set: 
                return False        
        for aux_prop in self.aux_true_set:
            if aux_prop not in o.aux_true_set:
                return False

        return True
    
    def __lt__(self,o):
        return True
    
    def __hash__(self):
        return hash(self.__str__())

    def __deepcopy__(self,memodict={}):
        new_pddl_state = PDDLState(self.true_set,self.false_set,self.aux_true_set,self.aux_false_set)
        return new_pddl_state