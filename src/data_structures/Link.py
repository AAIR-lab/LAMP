import functools

@functools.total_ordering
class Link(object):
    def __init__(self, link_name, link_type):
        self.name = link_name
        self.type = link_type
    
    def __eq__(self,o):
        if type(o) == type("s"): 
            if self.name == o: 
                return True
            else:
                return False
        if self.name == o.name and self.type == o.type:
            return True
        else:
            return False
    
    def __lt__(self,o):
        if self.name < o.name:
            return True
        else:
            return False
    
    def __str__(self):
        return self.name

    def __hash__(self):
        return hash(self.__str__())
