import numpy as np
import open3d as o3
import copy
import time

Param={
  "normal_radius":0.01,
  "normal_min_nn":25,
  "feature_mesh":0,
  "feature_radius":0.025,
  'distance_threshold':0.001,
  'icp_threshold':0.0015
}

modFtArray=[]
modPcArray=[]
scnFtArray=[]
scnPcArray=[]

def toNumpy(pcd):
  return np.reshape(np.asarray(pcd.points),(-1,3))

def fromNumpy(dat):
  d=dat.astype(np.float32)
  pc=o3.PointCloud()
  pc.points=o3.Vector3dVector(d)
  return pc

def _get_features(cloud):
  o3.estimate_normals(cloud,o3.KDTreeSearchParamRadius(radius=Param["normal_radius"]))
  viewpoint=np.array([0.0,0.0,0.0],dtype=float)
  o3.orient_normals_towards_camera_location(cloud, camera_location=viewpoint)
  nfmin=Param["normal_min_nn"]
  if nfmin<=0: nfmin=1
  cl,ind=o3.geometry.radius_outlier_removal(cloud,nb_points=nfmin,radius=Param["normal_radius"])
  nfcl=o3.geometry.select_down_sample(cloud,ind)
  cloud.points=nfcl.points
  cloud.normals=nfcl.normals
  cds=cloud
  if Param["feature_mesh"]>0:
    cds=o3.voxel_down_sample(cloud,voxel_size=Param["feature_mesh"])
  return cds,o3.compute_fpfh_feature(cds,o3.KDTreeSearchParamRadius(radius=Param["feature_radius"]))

def learn(datArray,prm):
  global modFtArray,modPcArray,Param
  Param.update(prm)
  modFtArray=[]
  modPcArray=[]
  for dat in datArray:
    pc=fromNumpy(dat)
    modPcArray.append(pc)
    modFtArray.append(_get_features(pc))
  return modPcArray

def solve(datArray,prm):
  global scnFtArray,scnPcArray,Param
  Param.update(prm)
  scnFtArray=[]
  scnPcArray=[]
  t1=time.time()
  for dat in datArray:
    pc=fromNumpy(dat)
    scnPcArray.append(pc)
    scnFtArray.append(_get_features(pc))
  print "time for calc feature",time.time()-t1
  t1=time.time()
  resft=o3.registration.registration_fast_based_on_feature_matching(
    modFtArray[0][0],scnFtArray[0][0],modFtArray[0][1],scnFtArray[0][1],
    o3.registration.FastGlobalRegistrationOption(
      division_factor = 1.4,
      use_absolute_scale = 1,
      decrease_mu = 1,
      maximum_correspondence_distance = Param["distance_threshold"],
      iteration_number = 64,
      tuple_scale = 0.950000,
      maximum_tuple_count = 1000))
  print "time for feature matching",time.time()-t1
  print "feature matching",resft.transformation,resft.fitness
  resicp=o3.registration_icp(
    modPcArray[0],scnPcArray[0],
    Param["icp_threshold"],
    resft.transformation,o3.TransformationEstimationPointToPlane())
  return {"transform":[resicp.transformation],"fitness":[resicp.fitness],"rmse":[resicp.inlier_rmse]}
