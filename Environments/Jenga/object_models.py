import openravepy
from openravepy import *
import numpy as np
from operator import truediv

HALF_PLANK_LENGTH = 0.0762
HALF_PLANK_BREADTH = 0.0254
HALF_PLANK_HEIGHT = 0.01524

def create_dest(env, destname, t, vert=False):
  dim = 0.1
  THICKNESS = 0.001

  surface = openravepy.KinBody.GeometryInfo()
  surface._type = openravepy.GeometryType.Box
  if vert:
    surface._vGeomData = [THICKNESS/2, dim/2, dim/2]
  else:
    surface._vGeomData = [dim/2, dim/2, THICKNESS/2]
  surface._vDiffuseColor = [1, 0, 0]

  dest = openravepy.RaveCreateKinBody(env, '')
  dest.InitFromGeometries([surface])
  dest.SetName(destname)

  dest.SetTransform(t)

  return dest

def create_dropbox(env,init_trans=np.eye(4)):
  base = openravepy.KinBody.GeometryInfo()
  base._type = openravepy.GeometryType.Box
  base._vGeomData = [0.075, 0.075, 0]
  base._t[0,3] = 0 #x
  base._t[1,3] = 0 #y
  base._t[2,3] = 0
  base._vDiffuseColor = [1,0.8,0]
  
  wall_1 = openravepy.KinBody.GeometryInfo()
  wall_1._type = openravepy.GeometryType.Box
  wall_1._vGeomData = [0.005, 0.085, 0.05]
  wall_1._t[0,3] = 0.08 #x
  wall_1._t[1,3] = 0 #y
  wall_1._t[2,3] = 0.05
  wall_1._vDiffuseColor = [1,0.8,0]

  wall_2 = openravepy.KinBody.GeometryInfo()
  wall_2._type = openravepy.GeometryType.Box
  wall_2._vGeomData = [0.005, 0.085, 0.05]
  wall_2._t[0,3] = -0.08 #x
  wall_2._t[1,3] = 0 #y
  wall_2._t[2,3] = 0.05
  wall_2._vDiffuseColor = [1,0.8,0]

  wall_3 = openravepy.KinBody.GeometryInfo()
  wall_3._type = openravepy.GeometryType.Box
  wall_3._vGeomData = [0.075, 0.005, 0.05]
  wall_3._t[0,3] = 0 #x
  wall_3._t[1,3] = -0.08 #y
  wall_3._t[2,3] = 0.05
  wall_3._vDiffuseColor = [1,0.8,0]

  wall_4 = openravepy.KinBody.GeometryInfo()
  wall_4._type = openravepy.GeometryType.Box
  wall_4._vGeomData = [0.075, 0.005, 0.05]
  wall_4._t[0,3] = 0 #x
  wall_4._t[1,3] = 0.08 #y
  wall_4._t[2,3] = 0.05
  wall_4._vDiffuseColor = [1,0.8,0]

  box = openravepy.RaveCreateKinBody(env, '')
  box.InitFromGeometries([base,wall_1, wall_2, wall_3, wall_4])
  box.SetName("droparea")
  box.SetTransform(init_trans)

  return box

def create_flat_area(env,name,color=[0.75,0.75,0.75],t=np.eye(4),dims=[0.075,0.075]):
  dims = dims+[0]
  infobox = KinBody.GeometryInfo()
  infobox._type = GeometryType.Box
  infobox._vGeomData = dims
  infobox._bVisible = True
  infobox._vDiffuseColor = color
  # infobox._t[2, 3] = dims[2] / 2

  box = RaveCreateKinBody(env, '')
  box.InitFromGeometries([infobox])
  box.SetName(name)
  box.SetTransform(t)
  env.AddKinBody(box)
  return box

def create_plank(env,plankname,plank_transform=np.eye(4)):
  infobox = KinBody.GeometryInfo()
  infobox._type = GeometryType.Box
  infobox._vGeomData = [HALF_PLANK_LENGTH,HALF_PLANK_BREADTH,HALF_PLANK_HEIGHT]
  infobox._bVisible = True
  infobox._vDiffuseColor = [0,0.999,0.999]
  # infobox._t[2, 3] = dims[2] / 2

  box = RaveCreateKinBody(env, '')
  box.InitFromGeometries([infobox])
  box.SetName(plankname)
  box.SetTransform(plank_transform)
  env.AddKinBody(box)
  return box

def on_table(obj, table):
  T = obj.GetTransform()

  table_ab = table.ComputeAABB()
  table_top_z = table_ab.pos()[2] + table_ab.extents()[2]

  if "cloth" in obj.GetName():
    T[2, 3] = table_top_z + 0.03
    obj.SetTransform(T)
  else:
    obj_ab = obj.ComputeAABB()
    obj_min_z = obj_ab.pos()[2] - obj_ab.extents()[2]

    diff_z = obj_min_z - table_top_z -0.01
    T[2, 3] -= diff_z-0.001
    obj.SetTransform(T)

def create_box(env, body_name, t, dims, color=[0,1,1]):
  infobox = openravepy.KinBody.GeometryInfo()
  infobox._type = openravepy.GeometryType.Box
  infobox._vGeomData = dims
  infobox._bVisible = True
  infobox._vDiffuseColor = color
  infobox._t[2, 3] = dims[2] / 2

  box = openravepy.RaveCreateKinBody(env, '')
  box.InitFromGeometries([infobox])
  box.SetName(body_name)
  box.SetTransform(t)

  return box

def create_cylinder(env, body_name, t, dims, color=[0,1,1]):
  infocylinder = openravepy.KinBody.GeometryInfo()
  infocylinder._type = openravepy.GeometryType.Cylinder
  infocylinder._vGeomData = dims
  infocylinder._bVisible = True
  infocylinder._vDiffuseColor = color
  infocylinder._t[2, 3] = dims[1] / 2

  cylinder = openravepy.RaveCreateKinBody(env, '')
  cylinder.InitFromGeometries([infocylinder])
  cylinder.SetName(body_name)
  cylinder.SetTransform(t)

  return cylinder

def cylinder_collisionfree_model(env, body_name, pos, dims):
  iv_str = '#Inventor V2.1 ascii\nSeparator {\nCylinder {\nparts ALL\nradius %f\nheight %f\n}\n}'%(dims[0], dims[1]) # hardcode collision-free disk dimensions
  with open("cylinder.iv", "w+") as f:
    f.write(iv_str)
  xml_str = '<KinBody name="%s"> <Body name="surface" type="dynamic"> <Geom type="sphere"> <Render>cylinder.iv</Render> <Radius>0.0001</Radius> \
<Translation>0 0 %f</Translation> <RotationAxis>1 0 0 90></RotationAxis> </Geom> </Body> </KinBody>'%(body_name, dims[1] / 2)
  with open("temp_cylinder.xml", "w+") as f:
    f.write(xml_str)
  env.Load("temp_cylinder.xml")
  cylinder = env.GetKinBody(body_name)
  x, y, z = pos
  cylinder_t = openravepy.matrixFromPose([1, 0, 0, 0, x, y, z])
  cylinder.SetTransform(cylinder_t)

  return cylinder

def create_table(env, table_name, dim1, dim2, thickness, legdim1, legdim2, legheight):
  tabletop = openravepy.KinBody.GeometryInfo()
  tabletop._type = openravepy.GeometryType.Box
  tabletop._vGeomData = [dim1/2, dim2/2, thickness/2]
  tabletop._vDiffuseColor = [0.5, 0.2, 0.1]

  leg1 = openravepy.KinBody.GeometryInfo()
  leg1._type = openravepy.GeometryType.Box
  leg1._vGeomData = [legdim1/2, legdim2/2, legheight/2]
  leg1._t[0, 3] = dim1/2 - legdim1/2
  leg1._t[1, 3] = dim2/2 - legdim2/2
  leg1._t[2, 3] = -legheight/2 - thickness/2
  leg1._vDiffuseColor = [0.5, 0.2, 0.1]

  leg2 = openravepy.KinBody.GeometryInfo()
  leg2._type = openravepy.GeometryType.Box
  leg2._vGeomData = [legdim1/2, legdim2/2, legheight/2]
  leg2._t[0, 3] = dim1/2 - legdim1/2
  leg2._t[1, 3] = -dim2/2 + legdim2/2
  leg2._t[2, 3] = -legheight/2 - thickness/2
  leg2._vDiffuseColor = [0.5, 0.2, 0.1]

  leg3 = openravepy.KinBody.GeometryInfo()
  leg3._type = openravepy.GeometryType.Box
  leg3._vGeomData = [legdim1/2, legdim2/2, legheight/2]
  leg3._t[0, 3] = -dim1/2 + legdim1/2
  leg3._t[1, 3] = dim2/2 - legdim2/2
  leg3._t[2, 3] = -legheight/2 - thickness/2
  leg3._vDiffuseColor = [0.5, 0.2, 0.1]

  leg4 = openravepy.KinBody.GeometryInfo()
  leg4._type = openravepy.GeometryType.Box
  leg4._vGeomData = [legdim1/2, legdim2/2, legheight/2]
  leg4._t[0, 3] = -dim1/2 + legdim1/2
  leg4._t[1, 3] = -dim2/2 + legdim2/2
  leg4._t[2, 3] = -legheight/2 - thickness/2
  leg4._vDiffuseColor = [0.5, 0.2, 0.1]

  table = openravepy.RaveCreateKinBody(env, '')
  table.InitFromGeometries([tabletop, leg1, leg2, leg3, leg4])
  table.SetName(table_name)

  return table