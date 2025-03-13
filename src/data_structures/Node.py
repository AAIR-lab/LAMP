import functools

@functools.total_ordering
class Node(object):
    def __init__(self,state):
        self.state = state
        self.g = 0
        self.h = 0
        self.total_cost = 0
        self.parent = state
        self.parent_to_current_action = None

    def __str__(self):
        s = self.state.__str__()
        return s
    
    def __hash__(self):
        return hash(self.__str__())

    def __eq__(self,o):
        if self.__hash__() != o.__hash__():
            return False    
        return True

    def __lt__(self,o):
        return True