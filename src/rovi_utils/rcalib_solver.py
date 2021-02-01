#!/usr/bin/python

import cv2
import numpy as np
import tflib
import copy
from scipy import optimize
import itertools

Weight=1

def fit_func(prm,M1,P1,M2,P2):
  R,jacob=cv2.Rodrigues(np.array([prm[3],prm[4],prm[5]],dtype=float))
  T=np.array([prm[0],prm[1],prm[2]]).reshape((3,1))
  mTc=np.vstack((np.hstack((R,T)),np.array([0,0,0,1]).reshape((1,4))))
  bTm1=tflib.toRTfromVec(M1)
  cTp1=tflib.toRTfromVec(P1)
  bTm2=tflib.toRTfromVec(M2)
  cTp2=tflib.toRTfromVec(P2)
  bTp1=np.dot(np.dot(bTm1,mTc),cTp1)
  bTp2=np.dot(np.dot(bTm2,mTc),cTp2)
  terr=np.linalg.norm(bTp1[:3,3]-bTp2[:3,3])
  xerr=np.linalg.norm(bTp1[:3,0]-bTp2[:3,0])
  yerr=np.linalg.norm(bTp1[:3,1]-bTp2[:3,1])
  zerr=np.linalg.norm(bTp1[:3,2]-bTp2[:3,2])
  return terr+(xerr+yerr+zerr)

def solve(M,P):
  alen=np.linalg.norm(M[:,:3],axis=1)
  Weight=np.mean(alen)
  print "weight",Weight
  Mn=copy.copy(M)
  Pn=copy.copy(P)
  Mn[:,:3]=Mn[:,:3]/Weight
  Pn[:,:3]=Pn[:,:3]/Weight
  mat=np.hstack((Mn,Pn))
  Cmat=np.asarray(list(itertools.combinations(mat,2)))
  Dat1=Cmat[:,0,:]
  Dat2=Cmat[:,1,:]
#  M1=Dat1[0][:7]
#  P1=Dat1[0][7:14]
#  M2=Dat2[0][:7]
#  P2=Dat2[0][7:14]
#  print fit_func([0,0,0,0,0,0],M1,P1,M2,P2)
  M1=Dat1[:,:7].T
  P1=Dat1[:,7:14].T
  M2=Dat2[:,:7].T
  P2=Dat2[:,7:14].T
  result=optimize.leastsq(fit_func,[0,0,0,0,0,0],args=(M1,P1,M2,P2),maxfev=100000,ftol=0.000001)
  if result[1] not in [1,2,3,4]:
    print "rcalib_solver::scipy::optimize failed"
    return None
  result=np.asarray(result[0])
  result[:3]=result[:3]*Weight
  print "solve result",result
  R,jacob=cv2.Rodrigues(result[3:6])
  T=result[0:3].reshape((3,1))
  RT=np.vstack((np.hstack((R,T)),np.array([0,0,0,1]).reshape((1,4))))
  return RT
