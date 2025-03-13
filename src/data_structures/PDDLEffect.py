import copy
import functools
import Config

class GroundedPDDLEffect(object):
    def __init__(self, add_set, delete_set): 
        self.add_set = add_set
        self.delete_set = delete_set 

    def apply(self, pddl_state):
        new_state = copy.deepcopy(pddl_state)
        for prop in self.delete_set: 
            new_state.true_set.remove(prop)
        for prop in self.add_set:
            new_state.true_set.add(prop)
        return new_state

@functools.total_ordering
class LiftedPDDLEffect(object):
    def __init__(self, add_set, delete_set,aux_add, aux_delete): 
        self.add_set = add_set 
        self.delete_set = delete_set
        self.aux_add = aux_add
        self.aux_delete = aux_delete

    def get_grounded_effect(self,grounding): 
        add_set = set()
        delete_set = set() 
        for prop in self.add_set: 
            add_set.add(prop.ground_relation(grounding))
        for prop in self.delete_set: 
            delete_set.add(prop.ground_relation(grounding))
        return GroundedPDDLEffect(add_set, delete_set)

    def get_lifted_add_set(self):
        lifted_set = set([])
        for param_re in self.add_set:
            lifted_set.add(param_re.parent_relation)
        
        return lifted_set
    
    def get_lifted_delete_set(self):
        lifted_set = set([])
        for param_re in self.delete_set:
            lifted_set.add(param_re.parent_relation)
        
        return lifted_set
    
    def sort_set(self,to_sort):
        sort_list = list(to_sort)
        sort_list.sort()

        return sort_list
    
    def __eq__(self,o):
        if (self.add_set != o.add_set) or (self.delete_set != o.delete_set) or (self.aux_add != o.aux_add) or (self.aux_delete != o.aux_delete):
            return False
        return True
    
    def __lt__(self,o):
        return self.__str__() < o.__str__()
    
    def __str__(self):
        effect_string = ""
        for prop in self.sort_set(self.add_set):
            effect_string += "\t{} \n".format(str(prop))
        for prop in self.sort_set(self.delete_set):
            effect_string += "\t(not {})\n".format(str(prop))
        
        auxillary_string = ""
        for a_prop in self.sort_set(self.aux_add):
            if a_prop.id <= 2:
                auxillary_string += "\t({}) \n".format(str(a_prop))
            else:
                if str(a_prop).split()[1].split("_")[0] in Config.CONST_TYPES[Config.DOMAIN_NAME]:
                    s_ap = str(a_prop).split()[0] + " " + str(a_prop).split()[1]
                else:
                    s_ap = str(a_prop).split()[0] + " ?" + str(a_prop).split()[1]

                auxillary_string += "\t({}) \n".format(s_ap)

        for a_prop in self.sort_set(self.aux_delete):
            if a_prop.id <= 2:
                auxillary_string += "\t(not ({})) \n".format(str(a_prop))
            else:
                if str(a_prop).split()[1].split("_")[0] in Config.CONST_TYPES[Config.DOMAIN_NAME]:
                    s_ap = str(a_prop).split()[0] + " " + str(a_prop).split()[1]
                else:
                    s_ap = str(a_prop).split()[0] + " ?" + str(a_prop).split()[1]

                auxillary_string += "\t(not ({})) \n".format(s_ap)
        
        effect_string+=auxillary_string

        return effect_string

    def __hash__(self):
        return hash(self.__str__())