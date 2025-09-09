import numpy as np
import time
import os
import sys
from trac_ik_python.trac_ik import IK
from useful_functions import blockPrinting, pose_from_transform, transform_from_pose, get_relative_transform

def get_parent_with_file(file_name):
    current_dir = os.getcwd()
    while True:
        if file_name in os.listdir(current_dir):
            return current_dir
        else:
            current_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
    
    raise Exception("File Not Found in any Parent")

ROOT_DIR = get_parent_with_file("__init__.py")
if ROOT_DIR is not None and ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

from SimConfig import SimConfig
from Simulator import Simulator
from Object import Object

sim = Simulator()

class Robot(Object): 
    def __init__(self,env,id,real_world_exp=False):
        super(Robot,self).__init__(obj=None)

        self.real_world_exp = real_world_exp
        self.env = env
        self.id = id
        self.init_pose = []

        setattr(self,"grabbed_flag_{}".format(self.id),False)
        setattr(self,"grabbed_object_{}".format(self.id),None)

    def get_init_pose(self):
        self.init_pose = self.obj.GetActiveDOFValues()
    
    def set_init_pose(self):
        self.set_active_dof_values(self.init_pose)

    def grab(self,obj=None,arm=None,delta_mp=False):
        if obj is not None:
            o = self.env.GetKinBody(obj)
            robot_t = self.gripper_link.GetTransform()
            ot = o.GetTransform()
            euclidean_distance = np.linalg.norm(robot_t[:3,3]-ot[:3,3])
            obj_type = obj.split("_")[SimConfig.OBJ_TYPE_IND]
            grab_range = self.grab_range[obj_type]
            if delta_mp or \
               (euclidean_distance<grab_range[1] and euclidean_distance>grab_range[0]):
                self.obj.Grab(o)
                setattr(self,"grabbed_flag_{}".format(self.id),True) 
                setattr(self,"grabbed_object_{}".format(self.id),obj)
            else:
                setattr(self,"grabbed_flag_{}".format(self.id),False) 
                print("object out of grasp range")

    def release(self,arm=None):
        self.obj.ReleaseAllGrabbed()
        setattr(self,"grabbed_flag_{}".format(self.id),False) 
        setattr(self,"grabbed_object_{}".format(self.id),None)

    def activate_base_joints(self):
        pass

    def activate_manip_joints(self):
        pass

    def set_active_dof_values(self,values):
        self.obj.SetActiveDOFValues(values)

    def get_active_dof_values(self):
        return self.obj.GetActiveDOFValues()
    
    def get_robot_transform(self):
        return self.obj.GetTransform()

    def get_grabbed_object(self):
        return getattr(self,"grabbed_object_{}".format(self.id))

    def get_link(self,link_name):
        return self.obj.GetLink(link_name)

    def get_link_transform(self,link):
        if type(link) == str:
            link = self.get_link(link)
        
        return link.GetTransform()
    
    def get_links(self):
        return self.obj.GetLinks()

    def random_config_robot(self,current_dof=None,collision_fn=None,limits=None,use_collision_set=False,**kwargs):
        if current_dof is None:
            current_active_dof = self.obj.GetActiveDOF()
            current_dof = self.get_active_dof_values()
        else:
            current_active_dof = len(current_dof)

        if current_active_dof == 3:
            self.activate_base_joints()
            t = self.get_link("base_link").GetTransform()
        else:
            self.activate_manip_joints()
            t = self.gripper_link.GetTransform()

        if limits is None:
            lower_limits,upper_limits = self.obj.GetActiveDOFLimits()
        else:
            lower_limits,upper_limits = limits

        while True:
            random_dof_values = []
            for i in range(current_active_dof):
                range_offset = abs(upper_limits[i]-lower_limits[i])/6.0
                r = np.random.uniform(lower_limits[i]+range_offset,upper_limits[i]-range_offset)
                random_dof_values.append(r)
            
            if getattr(self,"grabbed_object_{}".format(self.id)) is None:
                if not Robot.check_collision(self,random_dof_values,collision_fn,use_collision_set):
                    break
            else:
                grabbed_object = self.env.GetKinBody(getattr(self,"grabbed_object_{}".format(self.id)))
                if not Robot.check_collision(self,random_dof_values,collision_fn,use_collision_set):
                    break
            
        self.set_active_dof_values(current_dof)
        return random_dof_values, t

    def __getattr__(self, name):
        if hasattr(self.obj,name) and callable(getattr(self.obj,name)):
            return getattr(self.obj,name)
        else:
            return super(Robot,self).__getattr__(name)

    @staticmethod
    def check_collision(robot, solution, collision_fn,use_collision_set=False):  
        cdof = robot.GetActiveDOFValues() 
        try:
            robot.SetActiveDOFValues(solution)
        except:
            return True
        
        env_objects = []
        if not use_collision_set:
            env_objects = [Object(o) for o in robot.GetEnv().GetBodies()]
            
        collision = collision_fn(env_objects,use_collision_set) 
        try:
            robot.SetActiveDOFValues(cdof)
        except:
            return True
            
        return collision

    def get_ik_solutions(self,end_effector_solution,check_collisions=True,robot_param=None,collision_fn = None):
        solution = pose_from_transform(end_effector_solution)
        if collision_fn is not None and check_collisions: 
            if Robot.check_collision(self.obj, solution, collision_fn):
                return [ ]
        return solution

    def openGripper(self):
        pass
    
    def closeGripper(self):
        pass

class MagicGripper(Robot):
    def __init__(self,env,id,collection=False,real_world_exp=False):
        super(MagicGripper,self).__init__(env,id,real_world_exp)

        self.obj_name = "gripper_{}".format(id)
        self.obj = self.load_robot()

        self.obj_offset = 0.07
        self.grab_range = {
            "plank": [0,0.15]
        }
        self.collection_flag = False

        finger_length = 0.02708
        finger_name = "rfinger"
        finger_wrist_T = get_relative_transform(self.obj.GetLink("base").GetTransform(),
                                                self.obj.GetLink(finger_name).GetTransform())

        self.geometry_limits = {
            "gripper": (2,[finger_wrist_T[2,3]-finger_length/2.0,finger_wrist_T[2,3]+finger_length/2.0]), 
            "base": (0,[0])
        }

        self.robot_type_object_mappings = {"base":"gripper_{}".format(self.id)}

        self.activate_manip_joints(collection_flag=collection)
        self.grasping_offset = {"plank": -0.135}
        self.gripper_link = self.obj
        self.ik_link = self.gripper_link

    def load_robot(self):
        self.env.Load(SimConfig.ROB_DIR+'MagicGripper/robot.xml')
        robot = self.env.GetRobot("gripper")
        robot.SetName(self.obj_name)
        return robot

    def activate_manip_joints(self,collection_flag=False):
        self.obj.SetActiveDOFs([],sim.DOFAffine.X|sim.DOFAffine.Y|sim.DOFAffine.Z|sim.DOFAffine.Rotation3D)
        self.obj.SetAffineRotation3DLimits(np.array([-np.pi]*3),np.array([np.pi]*3))

        if not self.collection_flag and collection_flag:
            self.collection_flag = collection_flag

        if self.collection_flag:
            self.obj.SetAffineTranslationLimits(np.array([-1,-1,-1]),np.array([1,1,2]))
        else:
            self.obj.SetAffineTranslationLimits(np.array([-100,-100,-5]),np.array([100,100,5]))

class ImageGripper(Robot):
    def __init__(self,env,id,collection=False,real_world_exp=False):
        super(ImageGripper,self).__init__(env,id,real_world_exp)

        self.obj_name = "gripper_{}".format(id)
        self.obj = self.load_robot()

        self.obj_offset = 0.07
        self.grab_range = {
            "plank": [0,0.15]
        }
        
        finger_length = 0.018
        finger_name = "rfinger"
        finger_wrist_T = get_relative_transform(self.obj.GetLink("base").GetTransform(),
                                                self.obj.GetLink(finger_name).GetTransform())

        self.geometry_limits = {
            "gripper": (2,[finger_wrist_T[2,3]-finger_length/2.0,finger_wrist_T[2,3]+finger_length/2.0]), 
            "base": (0,[0])
        }

        self.robot_type_object_mappings = {"base":"gripper_{}".format(self.id)}

        self.activate_manip_joints()
        self.grasping_offset = {"plank": -0.065}
        self.gripper_link = self.obj
        self.ik_link = self.gripper_link

    def load_robot(self):
        self.env.Load(SimConfig.ROB_DIR+'MagicGripper/robot_images.xml')
        robot = self.env.GetRobot("gripper")
        robot.SetName(self.obj_name)
        return robot
    
    def activate_manip_joints(self):
        self.obj.SetActiveDOFs([],sim.DOFAffine.X|sim.DOFAffine.Y|sim.DOFAffine.Z|sim.DOFAffine.Rotation3D)
        self.obj.SetAffineRotation3DLimits(np.array([-np.pi]*3),np.array([np.pi]*3))
        self.obj.SetAffineTranslationLimits(np.array([-1.5,-1.5,-2]),np.array([1.5,1.5,2]))

class MagneticGripper(Robot):
    def __init__(self,env,id,collection=False,real_world_exp=False):
        super(MagneticGripper,self).__init__(env,id,real_world_exp)

        self.name = "gripper_{}".format(id)
        self.obj = self.load_robot()

        self.obj_offset = 0.07
        self.grab_range = {
            "can": [0,0.24]
        }

        self.robot_type_object_mappings = {"base":"gripper_{}".format(self.id)}
        self.gripper_link = self.obj
        self.ik_link = self.gripper_link

        self.geometry_limits = {
            "gripper": (2,[-0.20,-0.22]), #TODO: needse to be the actual grasp offset between pose of object and gripper frame
            "base": (0,[0])
        }

        self.activate_manip_joints()
        self.grasping_offset = {"plank": -0.07,
                                "can": 0.21}

    def load_robot(self):
        self.env.Load(SimConfig.ROB_DIR+'MagneticGripper/robot.xml')
        robot = self.env.GetRobot("gripper")
        robot.SetName(self.name)
        return robot

    def activate_manip_joints(self):
        self.obj.SetActiveDOFs([],sim.DOFAffine.X|sim.DOFAffine.Y|sim.DOFAffine.Z|sim.DOFAffine.Rotation3D)
        self.obj.SetAffineRotation3DLimits(np.array([-np.pi]*3),np.array([np.pi]*3))
        self.obj.SetAffineTranslationLimits(np.array([-1.5,-1.5,-2]),np.array([1.5,1.5,2]))

class Fetch(Robot):
    def __init__(self,env,id,collection=False,real_world_exp=False):
        super(Fetch,self).__init__(env,id,real_world_exp)
        
        self.jointnames = (["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                            "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"])

        if SimConfig.DOMAIN_NAME != "DinnerTable":
            self.grabbed_armTuckDOFs = [0, 1.32, 1.4, -0.2, 1.72, 0, 1.3599999999999999, 0.0]
        else:
            self.grabbed_armTuckDOFs = [0.15, 1.32, 1.4, -0.2, 1.72, -0.35, 1.3599999999999999, 0.0] 

        self.fetch_urdf = SimConfig.URDF_DIR + "Fetch.urdf"
        self.fetch_srdf = SimConfig.URDF_DIR + "Fetch.srdf"

        self.obj = self.load_robot()
        self.obj.SetActiveManipulator("arm_torso")
        self.obj.SetAffineTranslationMaxVels([10.5, 10.5, 10.5])
        self.obj.SetAffineRotationAxisMaxVels(np.ones(4))
        robot_spawn_t = np.eye(4)
        robot_spawn_t[0,3] -= 0.025
        self.obj.SetTransform(robot_spawn_t)

        self.obj.SetActiveDOFs([self.obj.GetJoint(name).GetDOFIndex() for name in self.jointnames])
        # self.obj.SetActiveDOFValues(np.asarray(self.armTuckDOFs))
        self.obj.SetActiveDOFValues(np.asarray(self.grabbed_armTuckDOFs))
        self.manip_joints = self.obj.GetActiveDOFIndices()
        self.initGripper()
        self.openGripper()
        self.env.UpdatePublishedBodies()

        self.obj_name = self.obj.GetName()
        self.robot_type_object_mappings = {"base_link":"freight_{}".format(self.id),
                                           "wrist_roll_link":"gripper_{}".format(self.id)}

        self.gripper_link = self.obj.GetLink("wrist_roll_link")
        self.base_link = self.obj.GetLink("base_link")

        finger_length = 0.06
        finger_name = "r_gripper_finger_link"
        finger_wrist_T = get_relative_transform(self.obj.GetLink("wrist_roll_link").GetTransform(),
                                                self.obj.GetLink(finger_name).GetTransform())

        self.geometry_limits = {
            "gripper": (0,[finger_wrist_T[0,3]-finger_length/2.0 + 0.07,finger_wrist_T[0,3]+finger_length/2.0 + 0.07]), #TODO: make it for each object 
            "freight": (0,[0.560])
        }

        self.grab_range = {
                            "can":[0,0.26],
                            "glass": [0,0.22],
                            "bowl": [0,0.28],
                            "jenga": [0,0.22] 
                        }
        self.obj_offset = 0.15
        
        self.grasping_offset = {"can": -0.04,
                                "glass": -0.015,
                                "bowl": 0.24,
                                "jenga": -0.04
                                }

        self.ik_link = self.gripper_link

        self.urdf_string = None
        with open(self.fetch_urdf,"r") as f:
            self.urdf_string = f.read()

        self.ik_solver = IK("base_link","wrist_roll_link",urdf_string=self.urdf_string)
    
    def set_translation_limits(self,limits):
        self.activate_base_joints()
        self.obj.SetAffineTranslationLimits(np.array(limits[0] + [0]),np.array(limits[1] + [0]))

    def load_robot(self):
        sedstr = "sed -i \"s|project_directory|"+SimConfig.ROOT_DIR[:-1]+"|g\" " + SimConfig.SIM_ROOT_DIR
        os.system(sedstr + SimConfig.REL_URDF_DIR +"Fetch.urdf")
        module = self.RaveCreateModule(self.env, 'urdf')
        with self.env:
            name = module.SendCommand('loadURI ' + self.fetch_urdf+" "+self.fetch_srdf)
        sedstr = "sed -i \"s|"+SimConfig.ROOT_DIR[:-1]+"|project_directory|g\" " + SimConfig.SIM_ROOT_DIR
        os.system(sedstr + SimConfig.REL_URDF_DIR +"Fetch.urdf")
        
        robot = self.env.GetRobot(name)
        robot.SetName("{}_{}".format(name,self.id))

        return robot
    
    def random_config_robot(self,current_dof=None,collision_fn=None,limits=None,**kwargs):
        if self.real_world_exp:
            random_dof_values = [0.15, 0.72466344, -0.05064385, -1.73952133, 2.25099986, -1.50486781, -0.02543545, 2.76926565]
            t = self.gripper_link.GetTransform()

            return random_dof_values, t
                    
        else:
            return super(Fetch,self).random_config_robot(current_dof,collision_fn,limits,**kwargs)

    def initGripper(self):
        """Setup gripper closing direction and tool direction """
        gripperManip = self.obj.GetActiveManipulator()
        gripperIndices = gripperManip.GetGripperIndices()
        closingDirection = np.zeros(len(gripperIndices))

        for i, gi in enumerate(gripperIndices):
            closingDirection[i] = -1.

        gripperManip.SetChuckingDirection(closingDirection)
        gripperManip.SetLocalToolDirection([1, 0, 0])

    def openGripper(self):
        taskmanip = self.interfaces.TaskManipulation(self.obj)
        with self.obj:
            taskmanip.ReleaseFingers()
        self.obj.WaitForController(0)

    def tuck_arm(self):
        self.release()
        self.activate_manip_joints()
        # self.obj.SetActiveDOFValues(np.asarray(self.armTuckDOFs))
        self.obj.SetActiveDOFValues(np.asarray(self.grabbed_armTuckDOFs))

    def activate_base_joints(self):
        self.obj.SetActiveDOFs([], sim.DOFAffine.X | sim.DOFAffine.Y | sim.DOFAffine.RotationAxis)

    def activate_manip_joints(self):
        self.obj.SetActiveDOFs([self.obj.GetJoint(name).GetDOFIndex() for name in self.jointnames])
    
    @blockPrinting
    def get_ik_solutions(self,end_effector_solution,check_collisions=True,robot_param="gripper",collision_fn = None):
        solutions = []

        if robot_param == SimConfig.BASE_NAME:
            _x = end_effector_solution[0,3]
            _y = end_effector_solution[1,3]
            _yaw = self.axisAngleFromRotationMatrix(end_effector_solution[:3,:3])[-1]
            solutions = [_x,_y,_yaw]
            self.activate_base_joints()
            if collision_fn is not None and check_collisions: 
                if Robot.check_collision(self.obj, solutions, collision_fn):
                    return [ ]
        
        else:
            self.activate_manip_joints()
            current_state = self.obj.GetActiveDOFValues()
            collision = check_collisions
    
            required_T = np.linalg.pinv(self.base_link.GetTransform()).dot(end_effector_solution)
            pose = pose_from_transform(required_T)
            pos = pose[:3]
            orn = sim.quatFromAxisAngle(pose[3:])

            ik_count = 0
            collision = True
            while collision:
                seed_state = [np.random.uniform(-3.14, 3.14)] * self.ik_solver.number_of_joints
                solutions = self.ik_solver.get_ik(seed_state,
                                                 pos[0], pos[1], pos[2],  # X, Y, Z
                                                 orn[1], orn[2], orn[3], orn[0]  # QX, QY, QZ, QW
                                                 )
                ik_count += 1
                if ik_count<=SimConfig.MAX_IK_ATTEMPTS:
                    if solutions is not None:
                        collision = False
                        if collision_fn is not None and check_collisions: 
                            if Robot.check_collision(self.obj, solutions, collision_fn):
                                collision=True
                else:
                    print("max ik attempts exceeded")
                    solutions = []
                    break
            
            self.obj.SetActiveDOFValues(current_state)
            
        return solutions
                
    def get_arm_tuck_dofs(self,object_name=None,pose = [],grasp_num=None,surface_id=None):
        # grabbed_flag = getattr(self,"grabbed_flag_{}".format(self.id))
        # if grabbed_flag:
        #     return self.grabbed_armTuckDOFs
        
        # return self.armTuckDOFs
        return self.grabbed_armTuckDOFs

class yumi(Robot):
    def __init__(self,env,id,collection=False,real_world_exp=False):
        super(yumi,self).__init__(env,id,real_world_exp)

        self.right_arm_joints = ["yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
                                 "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"]
        self.left_arm_joints = ["yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
                                "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"]

        self.left_arm_tuck_DOFs = [ 0., -2.26892803, 2.35619449, 0.52359878,  0. ,  0.6981317 , -0. ]
        self.right_arm_tuck_DOFs = [  0.00000000e+00,  -2.26892803e+00, -2.35619449e+00,  5.23947841e-01, 5.23598776e-04,   6.76489618e-01,  -1.74532925e-04 ]
        self.grabbed_armTuckDOFs = self.left_arm_tuck_DOFs

        self.yumi_urdf = SimConfig.URDF_DIR + "yumi.urdf"
        self.yumi_srdf = SimConfig.URDF_DIR + "yumi.srdf"
        with open(self.yumi_urdf,"r") as f:
            self.urdf_string = f.read()

        self.obj = self.load_robot()
        self.manipulator_groups = ["right","left"] 
        
        r_T = np.eye(4)
        r_T[:3,3] = [-0.46,0,-0.08]
        self.obj_init_transform = r_T
        self.obj.SetTransform(r_T)

        self.initGripper()
        self.openGripper()
        self.env.UpdatePublishedBodies()
        
        self.obj.SetName(SimConfig.ROBOT_NAME)
        self.obj_name = str(self.obj.GetName())
        self.robot_type_object_mappings = {"gripper_l_base":"gripper_1",}
                                        #    "gripper_r_base":"gripper_2"}

        for manip in self.manipulator_groups:
            self.tuck_arm(manip)

        self.obj_offset = 0.055
        self.grab_range = {
            "plank": [0,0.15]
        }

        self.left_gripper_link = self.obj.GetLink("gripper_l_base")
        self.right_gripper_link = self.obj.GetLink("gripper_r_base")
        self.gripper_link = self.left_gripper_link
        # self.ik_link = self.obj.GetLink("gripper_l_finger_r")
        self.ik_link = self.left_gripper_link

        self.left_ik_solver = IK("world","gripper_l_base",urdf_string=self.urdf_string)
        self.right_ik_solver = IK("world","gripper_r_base",urdf_string=self.urdf_string)
         
        finger_length = 0.047
        finger_name = "gripper_l_finger_r"
        finger_wrist_T = get_relative_transform(self.obj.GetLink("gripper_l_base").GetTransform(),
                                                self.obj.GetLink(finger_name).GetTransform())

        self.geometry_limits = {
            "gripper": (2,[finger_wrist_T[2,3]-finger_length/2.0,finger_wrist_T[2,3]+finger_length/2.0]), 
            "base": (0,[0])
        }
        
        self.grasping_offset = {"plank": -0.135}
    
    def random_config_robot(self,current_dof=None,collision_fn=None,limits=None,**kwargs):
        if self.real_world_exp:
            random_dof_values = [ 0., -2.26892803, 2.35619449, 0.52359878,  0. ,  0.6981317 , -0. ]
            t = self.gripper_link.GetTransform()

            return random_dof_values, t
        
        else:
            return super(yumi,self).random_config_robot(current_dof,collision_fn,limits)
        
    def load_robot(self):
        sedstr = "sed -i \"s|project_directory|"+SimConfig.ROOT_DIR[:-1]+"|g\" " + SimConfig.SIM_ROOT_DIR
        os.system(sedstr + SimConfig.REL_URDF_DIR+"yumi.urdf")
        module = self.RaveCreateModule(self.env, 'urdf')
        with self.env:
            name = module.SendCommand('loadURI ' + self.yumi_urdf+" "+self.yumi_srdf)
        sedstr = "sed -i \"s|"+SimConfig.ROOT_DIR[:-1]+"|project_directory|g\" " + SimConfig.SIM_ROOT_DIR
        os.system(sedstr + SimConfig.REL_URDF_DIR+"yumi.urdf")
        
        robot = self.env.GetRobot(name)
        robot.SetName("{}_1".format(name))

        return robot
    
    def activate_manip_joints(self,arm="left",**kwargs):
        if arm not in self.manipulator_groups:
            arm = "left"

        joints = getattr(self,"{}_arm_joints".format(arm))
        self.obj.SetActiveDOFs([self.obj.GetJoint(name).GetDOFIndex() for name in joints])
        self.obj.SetActiveManipulator("{}_arm_effector".format(arm))

    def openGripper(self,arm="left"):
        self.activate_manip_joints(arm)
        self.initGripper()
        taskmanip = self.interfaces.TaskManipulation(self.obj)
        attempt_count = 0
        # time.sleep(1)
        while attempt_count < 10:
            try:
                with self.obj:
                    taskmanip.ReleaseFingers(movingdir=[1])
            except:
                attempt_count += 1
            else:
                break

        self.release()
        self.obj.WaitForController(0)

    def closeGripper(self,obj,arm="left"):
        self.activate_manip_joints(arm)
        self.initGripper()
        taskmanip = self.interfaces.TaskManipulation(self.obj)
        with self.obj:
            taskmanip.CloseFingers(movingdir=[-1])
        self.grab(obj,arm)
        self.obj.WaitForController(0)

    def tuck_arm(self,arm):
        self.activate_manip_joints(arm)
        self.release(arm)
        solution = getattr(self,"{}_arm_tuck_DOFs".format(arm))
        try:
            self.obj.SetActiveDOFValues(solution)
        except:
            pass
        
        self.openGripper()        
        return True
    
    def initGripper(self):
        """Setup gripper closing direction and tool direction """
        gripperManip = self.obj.GetActiveManipulator()
        gripperIndices = gripperManip.GetGripperIndices()
        closingDirection = np.zeros(len(gripperIndices))

    def grab(self,obj=None,arm="left",delta_mp=False):
        if obj is not None:
            o = self.env.GetKinBody(obj)
            gripper_link = getattr(self,"{}_gripper_link".format(arm))
            robot_t = gripper_link.GetTransform()
            ot = o.GetTransform()
            euclidean_distance = np.linalg.norm(robot_t[:3,3]-ot[:3,3])
            obj_type = obj.split("_")[SimConfig.OBJ_TYPE_IND]
            grab_range = self.grab_range[obj_type]
            if delta_mp or \
               (euclidean_distance<grab_range[1] and euclidean_distance>grab_range[0]):
                self.obj.Grab(o)
                setattr(self,"grabbed_flag_{}".format(self.id),True) 
                setattr(self,"grabbed_object_{}".format(self.id),obj)
            else:
                setattr(self,"grabbed_flag_{}".format(self.id),False) 
                print("object out of grasp range")

    def release(self,arm="left"):
        self.activate_manip_joints(arm)
        # if getattr(self,"grabbed_flag_{}".format(self.id)):
        #     self.openGripper()
        self.obj.ReleaseAllGrabbed()
        setattr(self,"grabbed_flag_{}".format(self.id),False) 
        setattr(self,"grabbed_object_{}".format(self.id),None)

    @blockPrinting
    def get_ik_solutions(self,end_effector_solution,robot_param="left",collision_fn = None,check_collisions=True):
        self.activate_manip_joints(arm=robot_param)
        current_state = self.obj.GetActiveDOFValues()
        collision = True
        if robot_param not in self.manipulator_groups:
            robot_param = "left"
        ik_solver = getattr(self,"{}_ik_solver".format(robot_param))
        ik_count = 0
        
        required_T = np.linalg.pinv(self.obj_init_transform).dot(end_effector_solution)
        pose = pose_from_transform(required_T)
        pos = pose[:3]
        orn = sim.quatFromAxisAngle(pose[3:])

        while collision:
            seed_state = [np.random.uniform(-3.14, 3.14)] * ik_solver.number_of_joints
            joint_values = ik_solver.get_ik(seed_state,
                                                 pos[0], pos[1], pos[2],  # X, Y, Z
                                                 orn[1], orn[2], orn[3], orn[0]  # QX, QY, QZ, QW
                                                 )
            ik_count += 1
            if ik_count<=SimConfig.MAX_IK_ATTEMPTS:
                if joint_values is not None:
                    self.obj.SetActiveDOFValues(joint_values)
                    print(joint_values)        
                    if collision_fn is not None and check_collisions: 
                        if Robot.check_collision(self.obj, joint_values, collision_fn):
                            collision=True
                        else:
                            collision = False
                # else:
                #     print("no joint_values")
            else:
                print("max ik attempts exceeded")
                joint_values = []
                break
        
        self.obj.SetActiveDOFValues(current_state)
        return joint_values