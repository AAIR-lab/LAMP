import os
import sys
import functools

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

from Simulator import Simulator

@functools.total_ordering
class Object(Simulator):
    def __init__(self,obj):
        self.obj = obj
    
    def __getattr__(self,name):
        if hasattr(self.obj,name) and callable(getattr(self.obj,name)):
            return getattr(self.obj,name)
        else:
            return super(Object,self).__getattr__(name)

    def get_transform(self):
        return self.obj.GetTransform()
    
    def set_transform(self,t):
        return self.obj.SetTransform(t)

    def get_name(self):
        if not hasattr(self,"name"):
            return str(self.obj.GetName())
        else:
            return self.name

    def set_name(self,name):
        self.obj.SetName(name)
        if hasattr(self,"name"):
            self.name = name
    
    def __hash__(self):
        return hash(self.get_name())
    
    def __str__(self):
        return self.get_name()

    def __lt__(self,o):
        return self.__str__() < o.__str__()
    
    def __eq__(self,o):
        if self.obj != o.obj or self.get_name() != o.get_name():
            return False
        
        return True