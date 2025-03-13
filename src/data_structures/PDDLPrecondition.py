import functools
import Config

class GroundedPDDLPrecondition(object): 
    def __init__(self,true_set,false_set): 
        self.true_set = true_set
        self.false_set = false_set
    
    def check_in_state(self,pddl_state):
        for prop in self.true_set: 
            if prop not in pddl_state.true_set:
                return False
        for prop in self.false_set:
            if prop in pddl_state.true_set: 
                return False
        return True

@functools.total_ordering
class LiftedPDDLPrecondition(object):
    def __init__(self, true_set, false_set,true_aux_set,false_aux_set=set()):
        self.true_set = true_set
        self.false_set = false_set
        self.true_aux_set = true_aux_set
        self.false_aux_set = false_aux_set

    def get_grounded_precondition(self,grounding): 
        true_set = set()
        false_set = set() 
        for prop in self.true_set: 
            true_set.add(prop.ground_relation(grounding))
        for prop in self.false_set: 
            false_set.add(prop.ground_relation(grounding))
        return GroundedPDDLPrecondition(true_set, false_set)
    
    def get_lifted_true_set(self):
        lifted_set = set([])
        for param_re in self.true_set:
            lifted_set.add(param_re.parent_relation)
        
        return lifted_set
    
    def get_lifted_false_set(self):
        lifted_set = set([])
        for param_re in self.false_set:
            lifted_set.add(param_re.parent_relation)
        
        return lifted_set

    def sort_set(self,to_sort):
        sort_list = list(to_sort)
        sort_list.sort()

        return sort_list
    
    def __eq__(self,o):
        if (self.true_set != o.true_set) or (self.false_set != o.false_set) or (self.true_aux_set != o.true_aux_set) or (self.false_aux_set != o.false_aux_set):
            return False
        return True
    
    def __lt__(self,o):
        return self.__str__() < o.__str__()
    
    def __str__(self):
        precondition_string = ""
        for prop in self.sort_set(self.true_set):
            precondition_string += "\t{}\n".format(str(prop))
        
        for prop in self.sort_set(self.false_set):
            precondition_string += "\t(not {})\n".format(str(prop))

        auxillary_string = ""
        for a_prop in self.sort_set(self.true_aux_set):
            if a_prop.id <= 2:
                auxillary_string += "\t({}) \n".format(str(a_prop))
            else:
                if str(a_prop).split()[1].split("_")[0] in Config.CONST_TYPES[Config.DOMAIN_NAME]:
                    s_ap = str(a_prop).split()[0] + " " + str(a_prop).split()[1]
                else:
                    s_ap = str(a_prop).split()[0] + " ?" + str(a_prop).split()[1]

                auxillary_string += "\t({}) \n".format(s_ap)

        for a_prop in self.sort_set(self.false_aux_set):
            if a_prop.id <= 2:
                auxillary_string += "\t({}) \n".format(str(a_prop))
            else:
                if str(a_prop).split()[1].split("_")[0] in Config.CONST_TYPES[Config.DOMAIN_NAME]:
                    s_ap = str(a_prop).split()[0] + " " + str(a_prop).split()[1]
                else:
                    s_ap = str(a_prop).split()[0] + " ?" + str(a_prop).split()[1]

                auxillary_string += "\t(not ({})) \n".format(s_ap)
        
        precondition_string+=auxillary_string

        return precondition_string

    def __hash__(self):
        return hash(self.__str__())