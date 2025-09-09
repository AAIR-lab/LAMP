import os
import sys
import numpy as np

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

sim = Simulator()

def create_flat_area(env,name,color=[0.75,0.75,0.75],t=np.eye(4),dims=[0.075,0.075]):
    dims = dims+[0]
    infobox = sim.KinBody.GeometryInfo()
    infobox._type = sim.GeometryType.Box
    infobox._vGeomData = dims
    infobox._bVisible = True
    infobox._vDiffuseColor = color
    # infobox._t[2, 3] = dims[2] / 2

    box = sim.RaveCreateKinBody(env, '')
    box.InitFromGeometries([infobox])
    box.SetName(name)
    box.SetTransform(t)
    env.AddKinBody(box)
    return box

def create_dropbox(env,init_trans=np.eye(4)):
    base = sim.KinBody.GeometryInfo()
    base._type = sim.GeometryType.Box
    base._vGeomData = [0.075, 0.075, 0]
    base._t[0,3] = 0 #x
    base._t[1,3] = 0 #y
    base._t[2,3] = 0
    base._vDiffuseColor = [1,0.8,0]
    
    wall_1 = sim.KinBody.GeometryInfo()
    wall_1._type = sim.GeometryType.Box
    wall_1._vGeomData = [0.005, 0.085, 0.05]
    wall_1._t[0,3] = 0.08 #x
    wall_1._t[1,3] = 0 #y
    wall_1._t[2,3] = 0.05
    wall_1._vDiffuseColor = [1,0.8,0]

    wall_2 = sim.KinBody.GeometryInfo()
    wall_2._type = sim.GeometryType.Box
    wall_2._vGeomData = [0.005, 0.085, 0.05]
    wall_2._t[0,3] = -0.08 #x
    wall_2._t[1,3] = 0 #y
    wall_2._t[2,3] = 0.05
    wall_2._vDiffuseColor = [1,0.8,0]

    wall_3 = sim.KinBody.GeometryInfo()
    wall_3._type = sim.GeometryType.Box
    wall_3._vGeomData = [0.075, 0.005, 0.05]
    wall_3._t[0,3] = 0 #x
    wall_3._t[1,3] = -0.08 #y
    wall_3._t[2,3] = 0.05
    wall_3._vDiffuseColor = [1,0.8,0]

    wall_4 = sim.KinBody.GeometryInfo()
    wall_4._type = sim.GeometryType.Box
    wall_4._vGeomData = [0.075, 0.005, 0.05]
    wall_4._t[0,3] = 0 #x
    wall_4._t[1,3] = 0.08 #y
    wall_4._t[2,3] = 0.05
    wall_4._vDiffuseColor = [1,0.8,0]

    box = sim.RaveCreateKinBody(env, '')
    box.InitFromGeometries([base,wall_1, wall_2, wall_3, wall_4])
    box.SetName("droparea")
    box.SetTransform(init_trans)

    return box

def create_jenga(env,plankname,plank_transform=np.eye(4)):
    HALF_JENGA_LENGTH = 0.0762
    HALF_JENGA_BREADTH = 0.0254
    HALF_JENGA_HEIGHT = 0.01524

    infobox = sim.KinBody.GeometryInfo()
    infobox._type = sim.GeometryType.Box
    infobox._vGeomData = [HALF_JENGA_LENGTH,HALF_JENGA_BREADTH,HALF_JENGA_HEIGHT]
    infobox._bVisible = True
    infobox._vDiffuseColor = [0,0.999,0.999]
    # infobox._t[2, 3] = dims[2] / 2

    box = sim.RaveCreateKinBody(env, '')
    box.InitFromGeometries([infobox])
    box.SetName(plankname)
    box.SetTransform(plank_transform)
    env.AddKinBody(box)
    return box

def create_plank(env,plankname,plank_transform=np.eye(4),color=[0.76,0.60,0.50]):
    HALF_PLANK_LENGTH = 0.05884
    HALF_PLANK_BREADTH = 0.01162
    HALF_PLANK_HEIGHT = 0.003875
    infobox = sim.KinBody.GeometryInfo()
    infobox._type = sim.GeometryType.Box
    infobox._vGeomData = [HALF_PLANK_LENGTH,HALF_PLANK_BREADTH,HALF_PLANK_HEIGHT]
    infobox._bVisible = True
    infobox._vDiffuseColor = color
    # infobox._t[2, 3] = dims[2] / 2

    box = sim.RaveCreateKinBody(env, '')
    box.InitFromGeometries([infobox])
    box.SetName(plankname)
    box.SetTransform(plank_transform)
    env.AddKinBody(box)
    return box

def create_box(env, body_name, t, dims, color=[0,1,1]):
    infobox = sim.KinBody.GeometryInfo()
    infobox._type = sim.GeometryType.Box
    infobox._vGeomData = dims
    infobox._bVisible = True
    infobox._vDiffuseColor = color
    infobox._t[2, 3] = dims[2] / 2

    box = sim.RaveCreateKinBody(env, '')
    box.InitFromGeometries([infobox])
    box.SetName(body_name)
    box.SetTransform(t)

    return box

def create_cylinder(env, body_name, t, dims, color=[0,1,1]):
    infocylinder = sim.KinBody.GeometryInfo()
    infocylinder._type = sim.GeometryType.Cylinder
    infocylinder._vGeomData = dims
    infocylinder._bVisible = True
    infocylinder._vDiffuseColor = color
    infocylinder._t[2, 3] = dims[1] / 2

    cylinder = sim.RaveCreateKinBody(env, '')
    cylinder.InitFromGeometries([infocylinder])
    cylinder.SetName(body_name)
    cylinder.SetTransform(t)

    return cylinder

def create_cafe_table(env,
                      table_name,
                      dim1,
                      dim2,
                      thickness,
                      legdim1,
                      legdim2,
                      legheight,
                      pose,
                      color):

    """
    thickness = 0.1
    legheight = 0.55
    pose = [4, 5, 0.63]
    tables = [[4, 8, 0.63], [6, 8, 0.63], [4, 10, 0.63], [6, 10, 0.63]]
    for i, pose in enumerate(tables):
        env.Add(create_table(env, 'table_'+str(i), 0.90, 0.90, thickness, 0.1, 0.1, legheight, pose))
    """
  
    x, y, z = pose

    tabletop = sim.KinBody.GeometryInfo()
    tabletop._type = sim.GeometryType.Box
    tabletop._vGeomData = [dim1/2, dim2/2, thickness/2]
    tabletop._t[0,3] = 0 #x
    tabletop._t[1,3] = 0 #y
    tabletop._t[2,3] = 0
    tabletop._vDiffuseColor = color

    leg = sim.KinBody.GeometryInfo()
    leg._type = sim.GeometryType.Box
    leg._vGeomData = [legdim1/2, legdim2/2, legheight/2]
    leg._t[0,3] = 0 #x
    leg._t[1,3] = 0 #y
    leg._t[2, 3] = (-legheight/2 - thickness/2)
    leg._vDiffuseColor = [0.5, 0.2, 0.1]

    tablebottom = sim.KinBody.GeometryInfo()
    tablebottom._type = sim.GeometryType.Box
    tablebottom._vGeomData = [dim1/4, dim2/4, thickness/8]
    tablebottom._t[0,3] = 0 #x
    tablebottom._t[1,3] = 0 #y
    tablebottom._t[2, 3] = (-legheight - thickness/2)
    tablebottom._vDiffuseColor = [0.5, 0.2, 0.1]

    table = sim.RaveCreateKinBody(env, '')
    table.InitFromGeometries([tabletop, leg, tablebottom])
    table.SetName(table_name)
    table.SetTransform(sim.matrixFromPose([1,0,0,0,x,y,z]))
    # print(table.GetTransform())
    # print(table.ComputeAABB())

    return table

def create_keva_table(env, table_name, dim1, dim2, thickness, legdim1, legdim2, legheight):
    tabletop = sim.KinBody.GeometryInfo()
    tabletop._type = sim.GeometryType.Box
    tabletop._vGeomData = [dim1/2, dim2/2, thickness/2]
    tabletop._vDiffuseColor = [0.5, 0.2, 0.1]

    leg1 = sim.KinBody.GeometryInfo()
    leg1._type = sim.GeometryType.Box
    leg1._vGeomData = [legdim1/2, legdim2/2, legheight/2]
    leg1._t[0, 3] = dim1/2 - legdim1/2
    leg1._t[1, 3] = dim2/2 - legdim2/2
    leg1._t[2, 3] = -legheight/2 - thickness/2
    leg1._vDiffuseColor = [0.5, 0.2, 0.1]

    leg2 = sim.KinBody.GeometryInfo()
    leg2._type = sim.GeometryType.Box
    leg2._vGeomData = [legdim1/2, legdim2/2, legheight/2]
    leg2._t[0, 3] = dim1/2 - legdim1/2
    leg2._t[1, 3] = -dim2/2 + legdim2/2
    leg2._t[2, 3] = -legheight/2 - thickness/2
    leg2._vDiffuseColor = [0.5, 0.2, 0.1]

    leg3 = sim.KinBody.GeometryInfo()
    leg3._type = sim.GeometryType.Box
    leg3._vGeomData = [legdim1/2, legdim2/2, legheight/2]
    leg3._t[0, 3] = -dim1/2 + legdim1/2
    leg3._t[1, 3] = dim2/2 - legdim2/2
    leg3._t[2, 3] = -legheight/2 - thickness/2
    leg3._vDiffuseColor = [0.5, 0.2, 0.1]

    leg4 = sim.KinBody.GeometryInfo()
    leg4._type = sim.GeometryType.Box
    leg4._vGeomData = [legdim1/2, legdim2/2, legheight/2]
    leg4._t[0, 3] = -dim1/2 + legdim1/2
    leg4._t[1, 3] = -dim2/2 + legdim2/2
    leg4._t[2, 3] = -legheight/2 - thickness/2
    leg4._vDiffuseColor = [0.5, 0.2, 0.1]

    table = sim.RaveCreateKinBody(env, '')
    table.InitFromGeometries([tabletop, leg1, leg2, leg3, leg4])
    table.SetName(table_name)

    return table