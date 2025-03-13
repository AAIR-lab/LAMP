import openravepy
class OpenRaveIKSolver(object):
    def __init__(self,env,robot):
        self.env = env
        self.robot = robot
        
    def get_ik_solutions(self, end_effector_transform, check_collisions=True):

        if check_collisions:
            filter_option = openravepy.IkFilterOptions.CheckEnvCollisions
        else:
            filter_option = openravepy.IkFilterOptions.IgnoreEndEffectorCollisions

        with self.env:
            ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=openravepy.IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                openravepy.raveLogInfo("Generating IKModel for " + str(self.robot))
                ikmodel.autogenerate()
            solutions = ikmodel.manip.FindIKSolutions(end_effector_transform, filter_option)

        if len(solutions) == 0:
            print("NO IKs found, Probably Un-reachable transform")

        return solutions

    def get_ik_solutions_to_obj(self, end_effector_transform, check_collisions=True):

        if check_collisions:
            filter_option = openravepy.IkFilterOptions.CheckEnvCollisions
        else:
            filter_option = openravepy.IkFilterOptions.IgnoreEndEffectorCollisions

        with self.env:
            ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=openravepy.IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                openravepy.raveLogInfo("Generating IKModel for " + str(self.robot))
                ikmodel.autogenerate()
            solutions = ikmodel.manip.FindIKSolutions(end_effector_transform, filter_option)

        if len(solutions) == 0:
            print("NO IKs found, Probably Un-reachable transform")

        return solutions
