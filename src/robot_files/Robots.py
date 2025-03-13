from openravepy import *
from openravepy.misc import DrawAxes
import Config
import numpy as np
import time
import os
from src.data_gen import utils
from trac_ik_python.trac_ik import IK
from src.useful_functions import blockPrinting, pose_from_transform, transform_from_pose


class Robot(object): 

    def __init__(self):
        pass 

    @staticmethod
    def check_collision(robot, solution, collision_fn):  
        # if collision_fn is not None: 
        cdof = robot.GetActiveDOFValues() 
        robot.SetActiveDOFValues(solution)
        collision = collision_fn(robot.GetEnv().GetBodies()) 
        robot.SetActiveDOFValues(cdof)
        return collision

    def openGripper(self):
        pass
    
    def closeGripper(self):
        pass

class MagicGripper(object):
    def __init__(self,env,id,collection=False):
        self.env = env
        self.id = id
        self.robot_name = "gripper_{}".format(id)
        self.robot = self.load_robot()

        self.robot_offset = 0.07
        self.grab_range = [0.12,0.15]
        self.offset_axis = 2

        self.robot_type_object_mappings = {"base":"gripper_{}".format(self.id)}

        setattr(self,"grabbed_flag_{}".format(self.id),False)
        setattr(self,"grabbed_object_{}".format(self.id),None)

        self.activate_manip_joints(collection_flag=collection)
        self.init_pose = []
        self.grasping_offset = {"plank": -0.135}
        self.gripper_link = self.robot
        self.ik_link = self.gripper_link

    def load_robot(self):
        self.env.Load(Config.ROB_DIR+'MagicGripper/robot.xml')
        robot = self.env.GetRobot("gripper")
        robot.SetName(self.robot_name)
        return robot
    
    def set_init_pose(self):
        self.init_pose = self.robot.GetActiveDOFValues()

    def grab(self,obj,arm=None):
        o = self.env.GetKinBody(obj)
        robot_t = self.robot.GetTransform()
        ot = o.GetTransform()
        euclidean_distance = np.linalg.norm(robot_t[:3,3]-ot[:3,3])
        if euclidean_distance<self.grab_range[1] and euclidean_distance>self.grab_range[0]:
            self.robot.Grab(o)
            setattr(self,"grabbed_flag_{}".format(self.id),True) 
            setattr(self,"grabbed_object_{}".format(self.id),obj)
        else:
            setattr(self,"grabbed_flag_{}".format(self.id),False) 
            print("object out of grasp range")

    def release(self,arm=None):
        self.robot.ReleaseAllGrabbed()
        setattr(self,"grabbed_flag_{}".format(self.id),False) 
        setattr(self,"grabbed_object_{}".format(self.id),None)

    def activate_base_joints(self):
        pass

    def activate_manip_joints(self,collection_flag=False):
        self.robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.Rotation3D)
        if collection_flag:
            self.robot.SetAffineTranslationLimits(np.array([-1,-1,-1]),np.array([1,1,2]))
        else:
            self.robot.SetAffineTranslationLimits(np.array([-100,-100,-5]),np.array([100,100,5]))

    def get_ik_solutions(self,end_effector_solution,check_collisions=False,robot_param=None,collision_fn = None):
        
        solution = pose_from_transform(end_effector_solution)
        if collision_fn is not None: 
            if Robot.check_collision(self.robot, solution, collision_fn):
                return [ ]
        return solution

class ImageGripper(object):
    def __init__(self,env,id,collection=False):
        self.env = env
        self.id = id
        self.robot_name = "gripper_{}".format(id)
        self.robot = self.load_robot()

        self.robot_offset = 0.07
        self.grab_range = [0.12,0.15]
        self.offset_axis = 2

        self.robot_type_object_mappings = {"base":"gripper_{}".format(self.id)}

        setattr(self,"grabbed_flag_{}".format(self.id),False)
        setattr(self,"grabbed_object_{}".format(self.id),None)

        self.activate_manip_joints()
        self.init_pose = []
        self.grasping_offset = {"plank": -0.065}
        self.gripper_link = self.robot
        self.ik_link = self.gripper_link

    def load_robot(self):
        self.env.Load(Config.ROB_DIR+'MagicGripper/robot_images.xml')
        robot = self.env.GetRobot("gripper")
        robot.SetName(self.robot_name)
        return robot
    
    def set_init_pose(self):
        self.init_pose = self.robot.GetActiveDOFValues()

    def grab(self,obj,arm=None):
        o = self.env.GetKinBody(obj)
        robot_t = self.robot.GetTransform()
        ot = o.GetTransform()
        euclidean_distance = np.linalg.norm(robot_t[:3,3]-ot[:3,3])
        if euclidean_distance<self.grab_range[1] and euclidean_distance>self.grab_range[0]:
            self.robot.Grab(o)
            setattr(self,"grabbed_flag_{}".format(self.id),True) 
            setattr(self,"grabbed_object_{}".format(self.id),obj)
        else:
            setattr(self,"grabbed_flag_{}".format(self.id),False) 
            print("object out of grasp range")

    def release(self,arm=None):
        self.robot.ReleaseAllGrabbed()
        setattr(self,"grabbed_flag_{}".format(self.id),False) 
        setattr(self,"grabbed_object_{}".format(self.id),None)

    def activate_base_joints(self):
        pass

    def activate_manip_joints(self):
        self.robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.Rotation3D)
        self.robot.SetAffineTranslationLimits(np.array([-1.5,-1.5,-2]),np.array([1.5,1.5,2]))

    def get_ik_solutions(self,end_effector_solution,check_collisions=False,robot_param=None,collision_fn = None):
        
        solution = pose_from_transform(end_effector_solution)
        if collision_fn is not None: 
            if Robot.check_collision(self.robot, solution, collision_fn):
                return [ ]
        return solution

class MagneticGripper(object):
    def __init__(self,env,id,collection=False):
        self.env = env
        self.id = id
        self.robot_name = "gripper_{}".format(id)
        self.robot = self.load_robot()

        self.robot_offset = 0.07
        self.grab_range = [0.19,0.22]
        self.offset_axis = 2

        self.robot_type_object_mappings = {"base":"gripper_{}".format(self.id)}
        self.gripper_link = self.robot
        self.ik_link = self.gripper_link

        setattr(self,"grabbed_flag_{}".format(self.id),False)
        setattr(self,"grabbed_object_{}".format(self.id),None)

        self.activate_manip_joints()
        self.init_pose = []
        self.grasping_offset = {"plank": -0.07,
                                "can": 0.21}

    def load_robot(self):
        self.env.Load(Config.ROB_DIR+'MagneticGripper/robot.xml')
        robot = self.env.GetRobot("gripper")
        robot.SetName(self.robot_name)
        return robot
    
    def set_init_pose(self):
        self.init_pose = self.robot.GetActiveDOFValues()

    def grab(self,obj,arm=None):
        o = self.env.GetKinBody(obj)
        robot_t = self.robot.GetTransform()
        ot = o.GetTransform()
        euclidean_distance = np.linalg.norm(robot_t[:3,3]-ot[:3,3])
        if euclidean_distance<self.grab_range[1] and euclidean_distance>self.grab_range[0]:
            self.robot.Grab(o)
            setattr(self,"grabbed_flag_{}".format(self.id),True) 
            setattr(self,"grabbed_object_{}".format(self.id),obj)
        else:
            setattr(self,"grabbed_flag_{}".format(self.id),False) 
            print("object out of grasp range")

    def release(self,arm=None):
        self.robot.ReleaseAllGrabbed()
        setattr(self,"grabbed_flag_{}".format(self.id),False) 
        setattr(self,"grabbed_object_{}".format(self.id),None)

    def activate_base_joints(self):
        pass

    def activate_manip_joints(self):
        self.robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.Rotation3D)
        self.robot.SetAffineTranslationLimits(np.array([-1.5,-1.5,-2]),np.array([1.5,1.5,2]))

    def get_ik_solutions(self,end_effector_solution,check_collisions=False,robot_param=None,collision_fn = None):
        solution = pose_from_transform(end_effector_solution)
        if collision_fn is not None: 
            if Robot.check_collision(self.robot, solution, collision_fn):
                return [ ]
        return solution
    
class Fetch(object):
    def __init__(self,env,id,collection=False):
        self.env = env
        self.jointnames = (["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                            "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"])

        # self.armTuckDOFs = [0.0, 1.32, 1.4, -0.2, 1.72, 0.0, 1.66, 0.0]
        if Config.DOMAIN_NAME != "DinnerTable":
            self.grabbed_armTuckDOFs = [0, 1.32, 1.4, -0.2, 1.72, 0, 1.3599999999999999, 0.0]
        else:
            self.grabbed_armTuckDOFs =[0.15, 1.32, 1.4, -0.2, 1.72, -0.35, 1.3599999999999999, 0.0] #for Dinner Table
        # self.grabbed_armTuckDOFs = [0.15, 0.72466344, -0.05064385, -1.73952133, 2.25099986, -1.50486781, -0.02543545, 2.76926565]

        self.id = id
        self.fetch_urdf = Config.URDF_DIR + "Fetch.urdf"
        self.fetch_srdf = Config.URDF_DIR + "Fetch.srdf"

        self.robot = self.load_robot()
        self.robot.SetActiveManipulator("arm_torso")
        self.robot.SetAffineTranslationMaxVels([10.5, 10.5, 10.5])
        self.robot.SetAffineRotationAxisMaxVels(np.ones(4))
        robot_spawn_t = np.eye(4)
        robot_spawn_t[0,3] -= 0.025
        self.robot.SetTransform(robot_spawn_t)

        self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in self.jointnames])
        # self.robot.SetActiveDOFValues(np.asarray(self.armTuckDOFs))
        self.robot.SetActiveDOFValues(np.asarray(self.grabbed_armTuckDOFs))
        self.manip_joints = self.robot.GetActiveDOFIndices()
        self.initGripper()
        self.openGripper()
        self.env.UpdatePublishedBodies()

        self.robot_name = self.robot.GetName()
        self.robot_type_object_mappings = {"base_link":"freight_{}".format(self.id),
                                           "wrist_roll_link":"gripper_{}".format(self.id)}

        setattr(self,"grabbed_flag_{}".format(self.id),False)
        setattr(self,"grabbed_object_{}".format(self.id),None)

        self.gripper_link = self.robot.GetLink("wrist_roll_link") #TODO: test it with link between fingers.

        self.grab_range = {
                            "can":[0.20,0.26],
                            "glass": [0.18,0.22],
                            "bowl": [0.23,0.26],
                            "jenga": [0.187,0.22] 
                        }
        self.robot_offset = 0.13
        self.offset_axis = 0
        
        self.grasping_offset = {"can": -0.04,
                                "glass": -0.015,
                                "bowl": 0.24,
                                "jenga": -0.04
                                }

        self.ik_link = self.gripper_link

    def load_robot(self):
        sedstr = "sed -i \"s|project_directory|"+Config.ROOT_DIR[:-1]+"|g\" " + Config.ROOT_DIR
        os.system(sedstr + "src/robot_files/Fetch/URDF/"+"Fetch.urdf")
        module = RaveCreateModule(self.env, 'urdf')
        with self.env:
            name = module.SendCommand('loadURI ' + self.fetch_urdf+" "+self.fetch_srdf)
        sedstr = "sed -i \"s|"+Config.ROOT_DIR[:-1]+"|project_directory|g\" " + Config.ROOT_DIR
        os.system(sedstr + "src/robot_files/Fetch/URDF/"+"Fetch.urdf")
        
        robot = self.env.GetRobot(name)
        robot.SetName("{}_{}".format(name,self.id))

        return robot
    
    def initGripper(self):
        """Setup gripper closing direction and tool direction """
        gripperManip = self.robot.GetActiveManipulator()
        gripperIndices = gripperManip.GetGripperIndices()
        closingDirection = np.zeros(len(gripperIndices))

        for i, gi in enumerate(gripperIndices):
            closingDirection[i] = -1.

        gripperManip.SetChuckingDirection(closingDirection)
        gripperManip.SetLocalToolDirection([1, 0, 0])

    def openGripper(self):
        taskmanip = interfaces.TaskManipulation(self.robot)
        with self.robot:
            taskmanip.ReleaseFingers()
        self.robot.WaitForController(0)

    def tuck_arm(self):
        self.release()
        self.activate_manip_joints()
        # self.robot.SetActiveDOFValues(np.asarray(self.armTuckDOFs))
        self.robot.SetActiveDOFValues(np.asarray(self.grabbed_armTuckDOFs))

    def activate_base_joints(self):
        self.robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis)

    def activate_manip_joints(self):
        self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in self.jointnames])
    
    @blockPrinting
    def get_ik_solutions(self,end_effector_solution,check_collisions=True,robot_param="gripper",collision_fn = None):
        solutions = []

        if robot_param == Config.BASE_NAME:
            _x = end_effector_solution[0,3]
            _y = end_effector_solution[1,3]
            _yaw = axisAngleFromRotationMatrix(end_effector_solution[:3,:3])[-1]
            solutions = [[_x,_y,_yaw]]
            self.activate_base_joints()
            if collision_fn is not None: 
                if Robot.check_collision(self.robot, solutions[0], collision_fn):
                    return [ ]
        else:  
            self.activate_manip_joints()
            if check_collisions:
                filter_option = IkFilterOptions.CheckEnvCollisions
            else:
                filter_option = IkFilterOptions.IgnoreEndEffectorCollisions
            
            with self.env:
                ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot,
                                                                             iktype=IkParameterization.Type.Transform6D)
                if not ikmodel.load():
                    ikmodel.autogenerate()
                try:
                    solutions = ikmodel.manip.FindIKSolutions(end_effector_solution, filter_option)
                except:
                    print("error")

            if len(solutions) == 0:
                print("NO IKs found, Probably Un-reachable transform")
        
        if len(solutions) > 0:
            if len(solutions) == 1:
                i = 0
            else:
                i = np.random.randint(0,len(solutions))
        else:
            return []
        
        return solutions[i]

    def grab(self,obj,arm=None):
        o = self.env.GetKinBody(obj)
        robot_t = self.gripper_link.GetTransform()
        ot = o.GetTransform()
        euclidean_distance = np.linalg.norm(robot_t[:3,3]-ot[:3,3])
        obj_type = obj.split("_")[0]
        grab_range = self.grab_range[obj_type]
        if euclidean_distance<grab_range[1] and euclidean_distance>grab_range[0]:
            self.robot.Grab(o)
            setattr(self,"grabbed_flag_{}".format(self.id),True) 
            setattr(self,"grabbed_object_{}".format(self.id),obj)
        else:
            setattr(self,"grabbed_flag_{}".format(self.id),False) 
            print("object out of grasp range")
    
    def release(self,arm=None):
        self.robot.ReleaseAllGrabbed()
        setattr(self,"grabbed_flag_{}".format(self.id),False) 
        setattr(self,"grabbed_object_{}".format(self.id),None)

    def get_arm_tuck_dofs(self,object_name=None,pose = [],grasp_num=None,surface_id=None):
        # grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
        # if grabbed_flag:
        #     return self.grabbed_armTuckDOFs
        
        # return self.armTuckDOFs
        return self.grabbed_armTuckDOFs

class yumi(object):
    def __init__(self,env,id,collection=False):
        self.env = env
        self.right_arm_joints = ["yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
                                 "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"]
        self.left_arm_joints = ["yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
                                "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"]

        self.left_arm_tuck_DOFs = [ 0., -2.26892803, 2.35619449, 0.52359878,  0. ,  0.6981317 , -0. ]
        self.right_arm_tuck_DOFs = [  0.00000000e+00,  -2.26892803e+00, -2.35619449e+00,  5.23947841e-01, 5.23598776e-04,   6.76489618e-01,  -1.74532925e-04 ]
        self.grabbed_armTuckDOFs = self.left_arm_tuck_DOFs

        self.yumi_urdf = Config.URDF_DIR + "yumi.urdf"
        self.yumi_srdf = Config.URDF_DIR + "yumi.srdf"
        with open(self.yumi_urdf,"r") as f:
            self.urdf_string = f.read()

        self.robot = self.load_robot()
        self.id = id
        self.manipulator_groups = ["right","left"] 
        
        r_T = np.eye(4)
        r_T[:3,3] = [-0.46,0,-0.08]
        self.robot_init_transform = r_T
        self.robot.SetTransform(r_T)

        self.initGripper()
        self.openGripper()
        self.env.UpdatePublishedBodies()
        
        self.robot.SetName(Config.ROBOT_NAME)
        self.robot_name = str(self.robot.GetName())
        self.robot_type_object_mappings = {"gripper_l_base":"gripper_1",}
                                        #    "gripper_r_base":"gripper_2"}
        
        setattr(self,"grabbed_flag_{}".format(self.id),False)
        setattr(self,"grabbed_object_{}".format(self.id),None)

        for manip in self.manipulator_groups:
            self.tuck_arm(manip)

        self.robot_offset = 0.055
        self.grab_range = [0.12,0.15]
        self.offset_axis = 2

        self.left_gripper_link = self.robot.GetLink("gripper_l_base")
        self.right_gripper_link = self.robot.GetLink("gripper_r_base")
        self.gripper_link = self.left_gripper_link
        # self.ik_link = self.robot.GetLink("gripper_l_finger_r")
        self.ik_link = self.left_gripper_link

        self.left_ik_solver = IK("world","gripper_l_base",urdf_string=self.urdf_string)
        self.right_ik_solver = IK("world","gripper_r_base",urdf_string=self.urdf_string)
        
        self.grasping_offset = {"plank": -0.135}
        
    def load_robot(self):
        sedstr = "sed -i \"s|project_directory|"+Config.ROOT_DIR[:-1]+"|g\" " + Config.ROOT_DIR
        os.system(sedstr + "src/robot_files/yumi/URDF/"+"yumi.urdf")
        module = RaveCreateModule(self.env, 'urdf')
        with self.env:
            name = module.SendCommand('loadURI ' + self.yumi_urdf+" "+self.yumi_srdf)
        sedstr = "sed -i \"s|"+Config.ROOT_DIR[:-1]+"|project_directory|g\" " + Config.ROOT_DIR
        os.system(sedstr + "src/robot_files/yumi/URDF/"+"yumi.urdf")
        
        robot = self.env.GetRobot(name)
        robot.SetName("{}_1".format(name))

        return robot
    
    def activate_manip_joints(self,arm="left"):
        if arm not in self.manipulator_groups:
            arm = "left"

        joints = getattr(self,"{}_arm_joints".format(arm))
        self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in joints])
        self.robot.SetActiveManipulator("{}_arm_effector".format(arm))

    def openGripper(self,arm="left"):
        self.activate_manip_joints(arm)
        self.initGripper()
        taskmanip = interfaces.TaskManipulation(self.robot)
        attempt_count = 0
        # time.sleep(1)
        while attempt_count < 10:
            try:
                with self.robot:
                    taskmanip.ReleaseFingers(movingdir=[1])
            except:
                attempt_count += 1
            else:
                break

        self.release()
        self.robot.WaitForController(0)

    def closeGripper(self,obj,arm="left"):
        self.activate_manip_joints(arm)
        self.initGripper()
        taskmanip = interfaces.TaskManipulation(self.robot)
        with self.robot:
            taskmanip.CloseFingers(movingdir=[-1])
        self.grab(obj,arm)
        self.robot.WaitForController(0)

    def tuck_arm(self,arm):
        self.activate_manip_joints(arm)
        self.release(arm)
        solution = getattr(self,"{}_arm_tuck_DOFs".format(arm))
        try:
            self.robot.SetActiveDOFValues(solution)
        except:
            pass
        
        self.openGripper()        
        return True
    
    def initGripper(self):
        """Setup gripper closing direction and tool direction """
        gripperManip = self.robot.GetActiveManipulator()
        gripperIndices = gripperManip.GetGripperIndices()
        closingDirection = np.zeros(len(gripperIndices))

    def grab(self,obj,arm="left"):
        o = self.env.GetKinBody(obj)
        gripper_link = getattr(self,"{}_gripper_link".format(arm))
        robot_t = gripper_link.GetTransform()
        ot = o.GetTransform()
        euclidean_distance = np.linalg.norm(robot_t[:3,3]-ot[:3,3])
        if euclidean_distance<self.grab_range[1] and euclidean_distance>self.grab_range[0]:
            # self.closeGripper()
            self.robot.Grab(o)
            setattr(self,"grabbed_flag_{}".format(self.id),True) 
            setattr(self,"grabbed_object_{}".format(self.id),obj)
        else:
            setattr(self,"grabbed_flag_{}".format(self.id),False) 
            print("object out of grasp range")

    def release(self,arm="left"):
        self.activate_manip_joints(arm)
        # if getattr(self,"grabbed_flag_{}".format(self.id)):
        #     self.openGripper()
        self.robot.ReleaseAllGrabbed()
        setattr(self,"grabbed_flag_{}".format(self.id),False) 
        setattr(self,"grabbed_object_{}".format(self.id),None)

    @blockPrinting
    def get_ik_solutions(self,end_effector_solution,robot_param="left",collision_fn = None ):
        self.activate_manip_joints(arm=robot_param)
        current_state = self.robot.GetActiveDOFValues()
        collision = True
        if robot_param not in self.manipulator_groups:
            robot_param = "left"
        ik_solver = getattr(self,"{}_ik_solver".format(robot_param))
        ik_count = 0
        
        required_T = np.linalg.pinv(self.robot_init_transform).dot(end_effector_solution)
        pose = pose_from_transform(required_T)
        pos = pose[:3]
        orn = quatFromAxisAngle(pose[3:])

        while collision:
            seed_state = [np.random.uniform(-3.14, 3.14)] * ik_solver.number_of_joints
            joint_values = ik_solver.get_ik(seed_state,
                                                 pos[0], pos[1], pos[2],  # X, Y, Z
                                                 orn[1], orn[2], orn[3], orn[0]  # QX, QY, QZ, QW
                                                 )
            ik_count += 1
            if ik_count<=Config.MAX_IK_ATTEMPTS:
                if joint_values is not None:
                    self.robot.SetActiveDOFValues(joint_values)
                    print(joint_values)        
                    # for obj in self.env.GetBodies():
                    #     gripper_link = getattr(self,"{}_gripper_link".format(robot_param))
                        # collision = self.env.CheckCollision(gripper_link,obj) and self.robot.GetName() != obj.GetName() and obj not in self.robot.GetGrabbed()
                    if collision_fn is not None: 
                        if Robot.check_collision(self.robot, joint_values, collision_fn):
                            collision=True
                        else:
                            collision = False
                # else:
                #     print("no joint_values")
            else:
                print("max ik attempts exceeded")
                joint_values = []
                break
        
        self.robot.SetActiveDOFValues(current_state)
        return joint_values
    
class LongGripper(object):
    def __init__(self,env,id,collection=False):
        self.env = env
        self.id = id
        self.robot_name = "gripper_{}".format(id)
        self.robot = self.load_robot()

        self.robot_offset = 0.07
        self.grab_range = [0.14,0.17]
        self.offset_axis = 2

        self.robot_type_object_mappings = {"base":"gripper_{}".format(self.id)}

        setattr(self,"grabbed_flag_{}".format(self.id),False)
        setattr(self,"grabbed_object_{}".format(self.id),None)

        self.activate_manip_joints(collection_flag=collection)
        self.init_pose = []
        self.grasping_offset = {"plank": -0.155}
        self.gripper_link = self.robot
        self.ik_link = self.gripper_link

    def load_robot(self):
        self.env.Load(Config.ROB_DIR+'MagicGripper/long_gripper.xml')
        robot = self.env.GetRobot("gripper")
        robot.SetName(self.robot_name)
        return robot
    
    def set_init_pose(self):
        self.init_pose = self.robot.GetActiveDOFValues()

    def grab(self,obj,arm=None):
        o = self.env.GetKinBody(obj)
        robot_t = self.robot.GetTransform()
        ot = o.GetTransform()
        euclidean_distance = np.linalg.norm(robot_t[:3,3]-ot[:3,3])
        if euclidean_distance<self.grab_range[1] and euclidean_distance>self.grab_range[0]:
            self.robot.Grab(o)
            setattr(self,"grabbed_flag_{}".format(self.id),True) 
            setattr(self,"grabbed_object_{}".format(self.id),obj)
        else:
            setattr(self,"grabbed_flag_{}".format(self.id),False) 
            print("object out of grasp range")

    def release(self,arm=None):
        self.robot.ReleaseAllGrabbed()
        setattr(self,"grabbed_flag_{}".format(self.id),False) 
        setattr(self,"grabbed_object_{}".format(self.id),None)

    def activate_base_joints(self):
        pass

    def activate_manip_joints(self,collection_flag=False):
        self.robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z|DOFAffine.Rotation3D)
        if collection_flag:
            self.robot.SetAffineTranslationLimits(np.array([-1,-1,-1]),np.array([1,1,2]))
        else:
            self.robot.SetAffineTranslationLimits(np.array([-100,-100,-5]),np.array([100,100,5]))

    def get_ik_solutions(self,end_effector_solution,check_collisions=False,robot_param=None,collision_fn = None):
        
        solution = pose_from_transform(end_effector_solution)
        if collision_fn is not None: 
            if Robot.check_collision(self.robot, solution, collision_fn):
                return [ ]
        return solution