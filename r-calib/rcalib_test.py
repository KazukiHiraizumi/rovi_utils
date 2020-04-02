#!/usr/bin/python

import cv2
import numpy as np
import math
from rovi_utils import rcalib_solver as solver
from rovi_utils import tflib

def error(mTc,M,P):
  bT=np.array([]).reshape((3,-1))
  bR=np.array([]).reshape((3,-1))
  for m,p in zip(M,P):
    bTm=tflib.toRTfromVec(m)
    cTp=tflib.toRTfromVec(p)
    bTp=np.dot(np.dot(bTm,mTc),cTp)
    bT=np.hstack((bT,bTp[:3,3]))
    rvec,jacob=cv2.Rodrigues(bTp[:3,:3])
    bR=np.hstack((bR,rvec))
  tcen=np.mean(bT,axis=1).reshape((3,1))
  print "t-mean",tcen
  terr=np.linalg.norm(bT-tcen,axis=0)
  print "t-error mean/max",np.mean(terr),np.max(terr)
  rcen=np.mean(bR,axis=1).reshape((3,1))
  print "r-center",rcen
  rerr=np.linalg.norm(bR-rcen,axis=0)
  print "r-error mean/max",np.mean(rerr),np.max(rerr)
  return np.max(terr),np.max(rerr)

Tcsv=np.loadtxt("rcalib_input.txt").reshape((-1,14))
#print Tcsv

poses=Tcsv[:,:7]
grids=Tcsv[:,7:14]

#print poses
#print grids
mTc=solver.solve(poses,grids)
mtc=tflib.fromRT(mTc)

print mtc

Terr,Rerr=error(mTc,poses,grids)
