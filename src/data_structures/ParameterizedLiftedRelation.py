from src.data_structures.Link import Link
import functools
import Config

@functools.total_ordering
class ParameterizedLiftedRelation(object):
    def __init__(self, pid1, pid2, parent_relation): 
        self.pid1 = pid1
        self.pid2 = pid2 
        self.parent_relation = parent_relation

    def ground_relation(self,grounding): 
        if "Const" in self.pid1:
            grounded_param1 = Link(link_name=self.pid1,link_type=self.pid1.split("_")[0])
        else:
            grounded_param1 = grounding[self.pid1]
            
        if "Const" in self.pid2:
            grounded_param2 = Link(link_name=self.pid2,link_type=self.pid2.split("_")[0])
        else:
            grounded_param2 = grounding[self.pid2]

        return self.parent_relation.get_grounded_relation(grounded_param1, grounded_param2)    

    def __str__(self):
        if self.parent_relation.parameter1_type in Config.CONST_TYPES[Config.DOMAIN_NAME]:
            return "({}_{}_{} {} ?{})".format(self.parent_relation.parameter1_type, self.parent_relation.parameter2_type,self.parent_relation.cr, self.pid1, self.pid2)
        elif self.parent_relation.parameter2_type in Config.CONST_TYPES[Config.DOMAIN_NAME]:
            return "({}_{}_{} ?{} {})".format(self.parent_relation.parameter1_type, self.parent_relation.parameter2_type,self.parent_relation.cr, self.pid1, self.pid2)
        
        return "({}_{}_{} ?{} ?{})".format(self.parent_relation.parameter1_type, self.parent_relation.parameter2_type,self.parent_relation.cr, self.pid1, self.pid2)
    
    def __hash__(self):
        return hash(self.__str__())
    
    def __eq__(self,o):
        if self.pid1 != o.pid1:
            return False
        if self.pid2 != o.pid2:
            return False
        if self.parent_relation != o.parent_relation:
            return False

        return True
    
    def __lt__(self,o):
        return self.__str__() < o.__str__()