import Config
import functools

@functools.total_ordering
class Const(object):
    def __init__(self, parameter1_type, parameter2_type, cr, id,parameter=""):
        self.parameter1_type = parameter1_type
        self.parameter2_type = parameter2_type
        self.cr = cr
        self.id = id
        self.parameter = parameter

    def __str__(self):
        if self.parameter == "":
            s = "clear{}_{}_{}_{}".format(self.id,self.parameter1_type,self.parameter2_type,self.cr)
        else:
            parameter_str = self.parameter
            if parameter_str:
                if self.parameter.split("_")[0] in Config.CONST_TYPES[Config.DOMAIN_NAME]:
                    parameter_str = self.parameter.split("_")[0] + "_Const"

            s = "clear{}_{}_{}_{} {}".format(self.id,self.parameter1_type,self.parameter2_type,self.cr,parameter_str)

        return s
    
    def __hash__(self):
        return hash(self.__str__())
    
    def __eq__(self,o):
        if self.parameter1_type != o.parameter1_type:
            return False
        if self.parameter2_type != o.parameter2_type:
            return False
        if self.cr != o.cr:
            return False
        if self.id != o.id:
            return False
        if self.parameter != o.parameter:
            return False
        return True
    
    def __lt__(self,o):
        if self.__str__() < o.__str__():
            return True
        else:
            return False

    def __deepcopy__(self,memodict={}):
        new_const_relation = Const(self.parameter1_type, self.parameter2_type, self.cr, self.id,self.parameter)
        return new_const_relation