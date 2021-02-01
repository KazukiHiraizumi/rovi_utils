#!/usr/bin/python

import numpy as np
import open3d as o3d
import copy
from scipy import optimize
from scipy.spatial.transform import Rotation as R

def toNumpy(pcd):
  return np.reshape(np.asarray(pcd.points),(-1,3))
def fromNumpy(dat):
  d=dat.astype(np.float32)
  pc=o3d.PointCloud()
  pc.points=o3d.Vector3dVector(d)
  return pc

def fit_func(prm,dat): #fir circle
  fv=np.linalg.norm(dat[:,1:3]-prm[1:3],axis=1)-prm[0]
  return fv

def solve(pcd,xc1,xc2,wid):
  P0=toNumpy(pcd)
  w1=np.where(np.abs(P0.T[0]-xc1)<wid/2)
  P1=P0[w1]
  G1=np.mean(P1,axis=0)
  result=optimize.leastsq(fit_func,G1,args=P1)
  if result[1] not in [1,2,3,4]:
    print "rcalib_solver::scipy::optimize failed"
    return None
  res1=np.asarray(result[0])
  w2=np.where(np.abs(P0.T[0]-xc2)<wid/2)
  P2=P0[w2]
  G2=np.mean(P2,axis=0)
  result=optimize.leastsq(fit_func,G2,args=P2)
  if result[1] not in [1,2,3,4]:
    print "rcalib_solver::scipy::optimize failed"
    return None
  res2=np.asarray(result[0])
#  print "journal",res1,res2
  k=xc1/xc2
  RT=np.eye(4)
  RT[1:3,3]=((res1-k*res2)/(1-k))[1:3]
  res1[0]=xc1
  res2[0]=xc2
  basx=res2-res1
  basx=basx/np.linalg.norm(basx)
  rot=np.cross(np.array([1,0,0]),basx)
  rnm=np.linalg.norm(rot)
  if rnm>0:
    rot=rot/rnm*np.arcsin(rnm)
  r=R.from_rotvec(rot)
  RT[:3,:3]=r.as_dcm()
  print "axis_solver",RT
  return RT

if __name__ == '__main__':
  print "Prepare model"
  pcd=o3d.read_point_cloud("surface.ply")
  rt=solve(pcd,-97,83,2.5)
  print rt
#  s1=fromNumpy(P1)
#  s1.paint_uniform_color([1, 0.706, 0])
#  s2=fromNumpy(P2)
#  s2.paint_uniform_color([1, 0.706, 0])
#  o3d.draw_geometries([s1,s2])
