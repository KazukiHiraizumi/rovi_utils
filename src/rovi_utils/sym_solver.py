#!/usr/bin/python

import numpy as np
import open3d as o3d
import cv2
import copy
import math

def trimRT(mat):
  M=np.hstack((mat,np.array([0,0,0]).reshape((3,1))))
  M=np.vstack((M,np.array([0,0,0,1]).reshape((1,4))))
  return M
def toNumpy(pcd):
  return np.reshape(np.asarray(pcd.points),(-1,3))
def fromNumpy(dat):
  d=dat.astype(np.float32)
  pc=o3d.PointCloud()
  pc.points=o3d.Vector3dVector(d)
  return pc
def getImmovTf(T,pc0,thres):
  pc1=copy.deepcopy(pc0)
  pc1.transform(T)
  reg=o3d.registration_icp(pc1,pc0,thres,np.eye(4,dtype=float),o3d.TransformationEstimationPointToPlane())
#  print reg.fitness
#  print reg.transformation
  return np.dot(reg.transformation,T)
def getwTx(N,pcd,thres):
  wTx=[]
  wTx.append(np.eye(4,dtype=float))
  for i in range(1,N):
    theta=2*np.pi*i/N
    cos=np.cos(theta)
    sin=np.sin(theta)
    wTwi=np.eye(4,dtype=float)
    wTwi[0,0]=cos;wTwi[0,1]=sin
    wTwi[1,0]=-sin;wTwi[1,1]=cos
    wTxi=getImmovTf(wTwi,pcd,thres)
    wTx.append(wTxi)
  return np.asarray(wTx)

def solve(pcd,num,thres):
  P0=toNumpy(pcd)
  g0=np.mean(P0,axis=0)
  sTw=np.eye(4,dtype=float)
  sTw[0,3]=g0[0]
  sTw[1,3]=g0[1]
  sTw[2,3]=g0[2]

  for i in range(2): #iteration
    pcdw=copy.deepcopy(pcd)
    pcdw.transform(np.linalg.inv(sTw))  #move points to axis center
    wTx=getwTx(int(num),pcdw,thres)
    zbase=wTx[:,:,2][:,:3]
#    print "symmetry zbase",zbase
    zmean=np.mean(zbase,axis=0)
    zmean=zmean/np.linalg.norm(zmean)
    zcross=np.cross(wTx[0,:,2][:3],zmean)  #angle would be small enough to aproximate as sin
    rod,jaco=cv2.Rodrigues(zcross)
    rod=trimRT(rod)
    obase=wTx[:,:,3]
    omean=np.mean(obase,axis=0)
    rod[:,3]=omean
    sTw=np.dot(sTw,rod)

  sTs=[]
  sTs.append(sTw)
  for i in range(1,num):
    sTs.append(np.dot(np.dot(sTw,wTx[i]),np.linalg.inv(sTw)))
  return sTs
