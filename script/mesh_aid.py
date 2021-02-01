#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import open3d as o3d
import copy
import sys
import time
import pprint
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from std_msgs.msg import String
from std_msgs.msg import Float32
from scipy.spatial.transform import Rotation

ORIGINAL_MODEL_FILE_SUFFIX = "_org"

TOPIC_MODEL_LOAD = "/request/model_load"
TOPIC_SCENE_DATA = "/scene/surface/floats"
TOPIC_EXEC_SOLVE = "/request/solve"
TOPIC_CREATE = "~create"
TOPIC_CLEAR = "~clear"
TOPIC_TEST = "~test"

DMY_MODEL_ROT_RANGE_X_DEGREE = 30
DMY_MODEL_ROT_RANGE_Y_DEGREE = 30
DMY_MODEL_ROT_RANGE_Z_DEGREE = 180

Config={
  "proc":0,
  "path":"recipe",
  "scenes":["surface"],
  "solver":"o3d_solver",
  "scene_frame_ids":[],
  "master_frame_ids":[],
  "base_frame_id":"world"}

# norm_rad_scale,ft_mesh_scale,ft_rad_scale
PRESET_PARAM_SCALES = (
    (2,2,5),
    (2,2,6),
    (3,2,4),
    (3,2,5),
    (3,2,6)
)

### Solver Param
#Param={
#  "normal_radius":0.00399,
#  "feature_radius":0.0199,
#  "normal_min_nn":2599,
#  "distance_threshold":0.199,
#  "icp_threshold":0.00199,
#  "rotate":0,
#  "repeat":1,
#  "cutter":{"base":0,"offset":0,"width":0}
#}

Param ={
  "mesh_size" : 5.0,
  "solver_prm_preset_no" : 0
}

Model_org_pcd=o3d.geometry.PointCloud
Model_meshed_pcd=o3d.geometry.PointCloud

def random_pmone():
    return 2.0 * np.random.rand() - 1.0
    
    
def update_param_preset( mesh_size , preset_idx ):
  if preset_idx < 0 or len(PRESET_PARAM_SCALES) <= preset_idx:
    rospy.logerr("preset param scale index out of bounds.")
    return False

  preset_param = PRESET_PARAM_SCALES[preset_idx]
  update_param(mesh_size,preset_param[0],preset_param[1],preset_param[2])
  return True

def update_param( mesh, norm_rad_scale, ft_mesh_scale, ft_rad_scale):
  norm_rad = norm_rad_scale * mesh
  ft_mesh = ft_mesh_scale * mesh
  ft_rad = ft_rad_scale * ft_mesh
  dist_th = 1.5 * ft_mesh
  icp_th = 0.8 * mesh
  
  values={
    "normal_radius": norm_rad,
    "feature_mesh":  ft_mesh,
    "feature_radius": ft_rad,
    "distance_threshold": dist_th,
    "icp_threshold": icp_th
  }
  
  
  try:
    solv_prm = rospy.get_param("~solver_param")
    rospy.loginfo("---------- Solver Param (Before) -----")
    pprint.pprint(solv_prm)
    solv_prm.update(values)
    rospy.set_param("~solver_param",solv_prm)
    
    rospy.loginfo("---------- Solver Param (After) -----")
    pprint.pprint(solv_prm)
    
  except Exception as e:
    rospy.logerr("update param exception:%s",e.args)


def get_model_path(suffix=""):
  try:
    Config.update(rospy.get_param("~config"))
  except Exception as e:
    rospy.logerr("Config get_param exception:%s",e.args)
    return ""
  
  if len(Config["scenes"]) < 1:
      return ""
  return Config["path"] + "/" + Config["scenes"][0] + suffix +  ".ply"

def cb_mesh_create(msg):
  global Model_org_pcd,Model_meshed_pcd,pub_model_load,Param
    
  start_tm = time.time()
  
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print "get_param exception:",e.args
  
  mesh_size = Param["mesh_size"]
  rospy.loginfo("mesh create start. mesh_size=%.2f",mesh_size)
  if mesh_size <= 0:
    rospy.logerr("invalid mesh_size.")
    return
  
  model_org_path  = get_model_path(ORIGINAL_MODEL_FILE_SUFFIX)
  
  rospy.loginfo("original model data load start. path=%s",model_org_path)
  Model_org_pcd = o3d.read_point_cloud(model_org_path)
  if Model_org_pcd.is_empty():
    rospy.logerr("original model data load failed.")
    return
  
  org_point_count = len(np.asarray(Model_org_pcd.points))
  rospy.loginfo("original model data load finished. point count=%d",org_point_count)
  
  rospy.loginfo("downsampling start.")
  Model_meshed_pcd = o3d.voxel_down_sample(Model_org_pcd, voxel_size = mesh_size )
  
  meshed_point_count = len(np.asarray(Model_meshed_pcd.points))
  
  rospy.loginfo("downsampling finished. before=%d, after=%d (%.2f%%)",
    org_point_count,meshed_point_count,meshed_point_count/float(org_point_count)*100.0)
  
  model_meshed_path  = get_model_path()
  rospy.loginfo("meshed model save start. path=%s",model_meshed_path)
  if not o3d.write_point_cloud(model_meshed_path,Model_meshed_pcd):
    rospy.logerr("meshed model save  failed.")
  else:
    rospy.loginfo("meshed model save finished.")
  
  rospy.loginfo("model create finished. proc_tm=%.3f sec" , ( time.time() - start_tm ))
  
  msgModelLoad = Bool()
  msgModelLoad.data = True
  pub_model_load.publish(msgModelLoad)


def cb_mesh_clear(msg):
  global Model_org_pcd, Model_meshed_pcd, Param
  rospy.loginfo("mesh clear.")
  Model_org_pcd = o3d.geometry.PointCloud
  Model_meshed_pcd = o3d.geometry.PointCloud
  

def make_dummy_data( input, mesh_size ):
  # ‰ñ“]Šp“x‚ðƒ‰ƒ“ƒ_ƒ€‚ÉŒˆ’è
  angles = [  DMY_MODEL_ROT_RANGE_X_DEGREE * random_pmone(),  # XŽ²‰ñ“].}30“x‚Ì”ÍˆÍ
              DMY_MODEL_ROT_RANGE_Y_DEGREE * random_pmone(),  # YŽ²‰ñ“].}30“x‚Ì”ÍˆÍ
              DMY_MODEL_ROT_RANGE_Z_DEGREE * random_pmone()]  # ZŽ²‰ñ“].}180“x‚Ì”ÍˆÍ
  rot = Rotation.from_euler('zyx', angles, degrees=True)   # scipy.spatial.transform.RotationŒ^
  rotMat = np.eye(4)
  rotMat[:3, :3] = rot.as_dcm()

  dmy_data = copy.deepcopy(input)
  dmy_data.transform(rotMat)
  return dmy_data
  
def cb_mesh_test(msg):
  global Model_meshed_pcd, Param
  
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print "get_param exception:",e.args
  
  mesh_size = Param["mesh_size"]
  if mesh_size is None:
    rospy.logerr("meshed model data is none.")
    return
  elif Model_meshed_pcd.is_empty():
    rospy.logerr("meshed model data is empty.")
    return
  
  rospy.loginfo("mesh test start. mesh_size=%.2f",mesh_size)
  
  preset_idx = Param["solver_prm_preset_no"]
  rospy.loginfo("solver parameter update start. preset_no=%d",preset_idx)
  update_param_preset(mesh_size,preset_idx)
  rospy.loginfo("solver parameter update finished.")
  
  dmy_pcd = make_dummy_data(Model_meshed_pcd, mesh_size)
  
  mesh_flotas=np.ravel(np.asarray(dmy_pcd.points))
  
  #np.set_printoptions(threshold=np.inf)
  #print(mesh_flotas)
  
  rospy.loginfo("scene data publish start. point count=%d (%d)", len(np.asarray(Model_meshed_pcd.points)), len(mesh_flotas))
  
  pub_scene_floats.publish(mesh_flotas)
  
  rospy.loginfo("scene data publish finished.")
  
  rospy.loginfo("solver publish start.")
  msgSolver = Bool()
  msgSolver.data = True
  pub_solve.publish(msgSolver)
  rospy.loginfo("solver publish finished.")

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key] = tokens[1]
  return args

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f
  
########################################################

rospy.init_node("mesh_aid",anonymous=True)
Config.update(parse_argv(sys.argv))
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  rospy.logerr("Config get_param exception:%s",e.args)

rospy.loginfo("========== Config @mesh_aid ==========")
pprint.pprint(Config)

try:
  if not rospy.has_param("~param"):
    rospy.set_param("~param",Param)
  else:
    Param.update(rospy.get_param("~param"))
except Exception as e:
  print "get_param exception:",e.args
rospy.loginfo("========== Param @mesh_aid ==========")
pprint.pprint(Param)


###I/O
rospy.Subscriber(TOPIC_CREATE, Bool, cb_mesh_create)
rospy.Subscriber(TOPIC_CLEAR, Bool, cb_mesh_clear)
rospy.Subscriber(TOPIC_TEST, Bool, cb_mesh_test)

pub_model_load = rospy.Publisher(TOPIC_MODEL_LOAD, Bool, queue_size=1)
pub_scene_floats = rospy.Publisher(TOPIC_SCENE_DATA, numpy_msg(Floats) ,queue_size=1)
pub_solve = rospy.Publisher(TOPIC_EXEC_SOLVE, Bool, queue_size=1)

try:
  rospy.spin()
except KeyboardInterrupt:
  rospy.loginfo("Shutting down")
