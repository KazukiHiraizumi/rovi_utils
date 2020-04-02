#!/usr/bin/python

import cv2
import numpy as np
import math
import roslib
import rospy
import tf
import tf2_ros
import open3d as o3d
import copy
import os
import sys
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from rovi_utils import tflib
from scipy import optimize

Param={"cropZ":0,"cropR":0,"mesh":0.001,"ladle":0,"ladleW":0,"nfrad":0,"nfmin":0}
Config={
  "relay":"/rovi/X1",
  "source_frame_id":"camera/capture",
  "frame_id":"camera/capture0",
  "capture_frame_id":"camera"
}

def P0():
  return np.array([]).reshape((-1,3))

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def voxel(data):
  mesh=Param["mesh"]
  if mesh==0: return data
  if len(data)<10: return data
  d=np.asarray(data).astype(np.float32)
  pc=o3d.geometry.PointCloud()
  pc.points=o3d.utility.Vector3dVector(d)
  rospy.loginfo("vec3d done")
  dwpc=o3d.geometry.voxel_down_sample(pc,voxel_size=mesh)
  rospy.loginfo("down sample done")
  return np.reshape(np.asarray(dwpc.points),(-1,3))

def nf(data):
  d=np.asarray(data).astype(np.float32)
  pc=o3d.geometry.PointCloud()
  pc.points=o3d.utility.Vector3dVector(d)
  nfmin=Param["nfmin"]
  if nfmin<=0: nfmin=1
  cl,ind=o3d.geometry.radius_outlier_removal(pc,nb_points=nfmin,radius=Param["nfrad"])
  dwpc=o3d.geometry.select_down_sample(pc,ind)
  return np.reshape(np.asarray(dwpc.points),(-1,3))

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("cropper::getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def pTr(RT,pc):
  return np.dot(RT[:3],np.vstack((pc.T,np.ones((1,len(pc)))))).T

def arrange(pc,n):
  capc=Config["source_frame_id"]
  outc=Config["frame_id"]
  RT=getRT(outc+str(n),capc+str(n))
  if RT is None:
    RT=getRT(outc+str(n),capc)
    if RT is None:
      RT=getRT(outc,capc+str(n))
      if RT is None:
        RT=getRT(outc,capc)
        if RT is None:
          RT=np.eye(4)
          rospy.logwarn("cropper::arrange::TF not found")
  return pTr(RT,pc)

def crop():
  pn=P0()
#camera cropping and merge points
  for n,pc in enumerate(srcArray):
    pt=pc.T
    w1=None
#    if cropZ>0:
#      w1=np.where(pt[2]<cropZ)
    w2=None
    if Param["cropR"]>0:
      w2=np.where(np.linalg.norm(pt[:2],axis=0)<Param["cropR"])
    if w1 is not None and w2 is not None:
      w=np.intersect1d(w1,w2)
      pa=arrange(pc[w],n)
    elif w1 is not None:
      pa=arrange(pc[w1],n)
    elif w2 is not None:
      pa=arrange(pc[w2],n)
    else:
      pa=arrange(pc,n)
    pn=np.vstack((pn,pa))
#ladle cropping(camera)
  pn=voxel(pn)
  if Param["ladC"]>0 and len(pn)>Param["ladC"]:
    d=np.linalg.norm(pn,axis=1)
    pn=pn[d.argsort(),:]
    pn=pn[:Param["ladC"],:]
#world z-crop
  if len(pn)>0:
    RT=getRT("world",Config["frame_id"])
    pw=pTr(RT,pn)
    pw=pw[np.ravel(pw[:,2]>Param["cropZ"])]
#ladle cropping(world)
    if Param["ladW"]>0 and len(pw)>Param["ladW"]:
      d=pw.T[2]
      pw=pw[np.ravel(d).argsort(),:]
      pw=pw[len(pw)-Param["ladW"]:,:]
#back to camera coordinate
    pn=pTr(np.linalg.inv(RT),pw)
    rospy.loginfo("ladle done")
#Noise eliminator
  if Param["nfrad"]>Param["mesh"]:
    pn=nf(pn)
    rospy.loginfo("noise filter done")
  pub_crop.publish(np2F(pn))
  return len(pn)

def raw():
  pn=P0()
  for n,pc in enumerate(srcArray):
    pa=arrange(pc,n)
    pn=np.vstack((pn,pa))
#  pub_raw.publish(np2F(pn))
  return

def cb_ps(msg): #callback of ps_floats
  global srcArray
  pc=np.reshape(msg.data,(-1,3))
#  pc=voxel(pc)
  srcArray.append(pc)
  raw()
  crop()
  pub_capture.publish(mTrue)
  return

def cb_param(msg):
  global Param
  prm=Param.copy()
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print "get_param exception:",e.args
  if prm!=Param:
    print "Param changed",Param
    crop()
  rospy.Timer(rospy.Duration(1),cb_param,oneshot=True) #Param update itself
  return

def cb_clear(msg):
  global srcArray,tfArray
  srcArray=[]
  tfArray=[]
  keep=Config["capture_frame_id"]
  try:
    keeptf=tfBuffer.lookup_transform("world",keep,rospy.Time())
    keeptf.header.stamp=rospy.Time.now()
    keeptf.header.frame_id="world"
    keeptf.child_frame_id=keep+"/capture0"
    broadcaster.sendTransform([keeptf])
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pub_msg.publish("cropper::clear::lookup failure world->"+keep)
  raw()
  crop()
  pub_clear.publish(mTrue)

def cb_capture(msg):
  global tfArray
  keep=Config["capture_frame_id"]
  try:
    keeptf=tfBuffer.lookup_transform("world",keep,rospy.Time())
    keeptf.header.stamp=rospy.Time.now()
    keeptf.header.frame_id="world"
    keeptf.child_frame_id=keep+"/capture"+str(len(srcArray))
    if len(srcArray)==0: tfArray=[]
    tfArray.append(keeptf)
    broadcaster.sendTransform(tfArray)
#    rospy.loginfo("tf2 broadcast %s",tfArray[0].child_frame_id)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    rospy.loginfo("cropper::capture::TF lookup failure world->"+keep)
  if pub_relay is not None: pub_relay.publish(mTrue)

def cb_ansback(msg):
  if msg.data is False: pub_capture.publish(mFalse)

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key]=tokens[1]
  return args

########################################################
rospy.init_node("cropper",anonymous=True)
Config.update(parse_argv(sys.argv))
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  print "get_param exception:",e.args
print "Config",Config
try:
  Param.update(rospy.get_param("~param"))
except Exception as e:
  print "get_param exception:",e.args
print "Param",Param

###Input topics
rospy.Subscriber("~in/floats",numpy_msg(Floats),cb_ps)
rospy.Subscriber("~clear",Bool,cb_clear)
rospy.Subscriber("~capture",Bool,cb_capture)
if "ansback" in Config:
  rospy.Subscriber(Config["ansback"],Bool,cb_ansback)
###Output topics
pub_crop=rospy.Publisher("~out/floats",numpy_msg(Floats),queue_size=1)
pub_raw=rospy.Publisher("~raw/floats",numpy_msg(Floats),queue_size=1)
pub_relay=None
if "relay" in Config:
  pub_relay=rospy.Publisher(Config["relay"],Bool,queue_size=1)
pub_clear=rospy.Publisher("~cleared",Bool,queue_size=1)
pub_capture=rospy.Publisher("~captured",Bool,queue_size=1)
pub_msg=rospy.Publisher("/message",String,queue_size=1)

###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()
rospy.sleep(1)
cb_clear(mTrue)

rospy.Timer(rospy.Duration(1),cb_param,oneshot=True) #Param update itself

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"

