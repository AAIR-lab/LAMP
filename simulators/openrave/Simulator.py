import os
import sys

import openravepy

class Simulator(object):
    def __getattr__(self,name):
        if hasattr(openravepy,name):
            return getattr(openravepy,name)
        else:
            raise AttributeError("{} attribute not found".format(name))
