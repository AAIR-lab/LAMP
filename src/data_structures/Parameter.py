import functools

@functools.total_ordering
class Parameter(object):
    def __init__(self,pid, type):
        self.pid = pid
        self.type = type

    def __str__(self):
        return str(self.pid)
    
    def __hash__(self):
        return hash(self.__str__())
    
    def __eq__(self,o):
        if self.type == o.type and self.pid == o.pid:
            return True
        else:
            return False

    def __lt__(self,o):
        if self.type < o.type:
            return True
        elif self.type == o.type and self.pid < o.pid:
            return True
        else:
            return False

    def get_grounded_parameter(self,value):
        return GroundedParameter(self.pid, self.type, value)

class GroundedParameter(Parameter):
    def __init__(self,pid, type, value):
        super(GroundedParameter,self).__init__(pid,type)
        self.value = value

    def __str__(self):
        return super(GroundedParameter,self).__str__() + " : " + str(self.value)
    
    def hash(self):
        return hash(self.__str__())
    
    def __eq__(self,o):
        if super(GroundedParameter,self).__eq__(o) and self.value == o.value:
            return True
        else:
            return False