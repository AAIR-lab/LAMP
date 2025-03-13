import functools

@functools.total_ordering
class ChangedRelations(object):
    def __init__(self,changed_lifted_added_relations,changed_lifted_deleted_relations,added_auxilary_relations,deleted_auxilary_relations):
        self.changed_lifted_added_relations = changed_lifted_added_relations
        self.changed_lifted_deleted_relations = changed_lifted_deleted_relations
        self.added_auxilary_relations = added_auxilary_relations
        self.deleted_auxilary_relations = deleted_auxilary_relations

    def __eq__(self,o):        
        return self.__hash__() == o.__hash__()

    def __lt__(self,o):
        return True

    def __str__(self):
        s = "added -> "
        for re in self.changed_lifted_added_relations:
            s+= "("
            s+= str(re)
            s+= "), "

        s += "deleted -> "
        for re in self.changed_lifted_deleted_relations:
            s+= "("
            s+= str(re)
            s+= "), "

        s += "added_auxillary -> "
        for re in self.added_auxilary_relations:
            s+= "("
            s+= str(re)
            s+= "), "
        
        s += "deleted_auxillary -> "
        for re in self.deleted_auxilary_relations:
            s+= "("
            s+= str(re)
            s+= "), "

        return s

    def __hash__(self):
        return hash(self.__str__())
