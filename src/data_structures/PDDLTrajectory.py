import functools

@functools.total_ordering
class PDDLTrajectory(object):
    def __init__(self,pddl_state_seq=[]):
        self.traj = pddl_state_seq
    
    def add(self,state):
        self.traj.append(state)

    def __str__(self):
        s=""        
        for state in self.traj:
            s+= "[{}]\n".format(str(state))
        return s

    def __eq__(self,o):
        if len(self.traj) != len(o.traj):
            return False
        
        for i in range(len(self.traj)):
            if (self.traj[i] != o.traj[i]):
                return False

        return True

    def __lt__(self,o):
        return True

    def __hash__(self):
        return hash(self.__str__())

    def __deepcopy__(self,memodict={}):
        new_seq = PDDLTraj(self.traj)
        return new_seq

    def __len__(self):
        return len(self.traj)