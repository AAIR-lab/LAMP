import Config

class ROSAction(object):
    def __init__(self,id,robot,gripper_pose,base_pose,Type,SubType,grabbed_object,base_frame,desired_ik,is_ik):
        self.id = id
        self.robot = robot
        self.gripper_pose = gripper_pose
        self.base_pose = base_pose
        self.Type = Type
        self.SubType = SubType
        self.base_frame = base_frame
        self.desired_ik = desired_ik 
        self.is_ik = is_ik
        if grabbed_object is None:
            self.grabbed_object = "" 
        else:
            self.grabbed_object = grabbed_object