#!/usr/bin/python

import cv2
import numpy as np
import math
import roslib
import rospy
import tf
import tf2_ros
import time
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from rovi_utils import rcalib_solver as solver
from rovi_utils import tflib

Config={
  "config_tf":"/config_tf",
  "camera_frame_id":"camera",
  "mount_frame_id":"mount",
  "flange_frame_id":"flange",
  "board_frame_id":"gridboard",
}

def cb_X0(f):
  global cTsAry,bTmAry
  print "cbX0"
  pb_msg.publish("rcalib::clear")
  cTsAry=[]
  bTmAry=[]
  count=Int32(); count.data=len(cTsAry); pb_count.publish(count)

def cb_X1(f):
  global cTsAry,bTmAry
  try:
    cTs=tfBuffer.lookup_transform(Config["camera_frame_id"], Config["board_frame_id"], rospy.Time())
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pb_err.publish("rcalib::gridboard lookup failed")
    done=Bool(); done.data=False; pb_Y1.publish(done)
    return
  try:
    if Config["mount_frame_id"] == "world":
      bTm=tfBuffer.lookup_transform("world",Config["flange_frame_id"], rospy.Time())  #The board may be held on the flange
    else:
      bTm=tfBuffer.lookup_transform("world",Config["mount_frame_id"], rospy.Time())  #The camera mount may be attached on some link
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pb_err.publish("rcalib::robot lookup failed")
    done=Bool(); done.data=False; pb_Y1.publish(done)
    return
  tf=bTm.transform
  pb_msg.publish("rcalib::robot["+str(len(cTsAry))+"]=("+"%.4f"%tf.translation.x+", "+"%.4f"%tf.translation.y+", "+"%.4f"%tf.translation.z+",    "+"%.4f"%tf.rotation.x+", "+"%.4f"%tf.rotation.y+", "+"%.4f"%tf.rotation.z+", "+"%.4f"%tf.rotation.w+")")
  cTsAry.append(cTs.transform)
  bTmAry.append(bTm.transform)
  done=Bool(); done.data=True; pb_Y1.publish(done)
  count=Int32(); count.data=len(cTsAry); pb_count.publish(count)
  return

def save_input(name):
  print "save input",len(bTmAry),len(cTsAry)
  Tcsv=np.array([]).reshape((-1,14))
  for M,S in zip(bTmAry,cTsAry):
    btm=tflib.fromRTtoVec(tflib.toRT(M))
    cts=tflib.fromRTtoVec(tflib.toRT(S))
    alin=np.hstack((btm,cts))
    Tcsv=np.vstack((Tcsv,alin))
  np.savetxt(name,Tcsv)
  return

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

def set_param_tf(name,tf):
  rospy.set_param(name,tflib.tf2dict(tf))
  if "copy" in Config:
    rospy.set_param(Config["copy"],tflib.tf2dict(tf))
  return

def call_visp():
  global cTsAry,bTmAry,mTc,Terr,Rerr
  poses=np.array([]).reshape((-1,7))
  for tf in bTmAry:
    if Config["mount_frame_id"] == "world":  #fixed camera
      tf=tflib.inv(tf)
    vec=tflib.fromRTtoVec(tflib.toRT(tf))
    poses=np.vstack((poses,vec))
  grids=np.array([]).reshape((-1,7))
  for tf in cTsAry:
    vec=tflib.fromRTtoVec(tflib.toRT(tf))
    grids=np.vstack((grids,vec))
  mTc=solver.solve(poses,grids)
  mtc=tflib.fromRT(mTc)
  set_param_tf(Config["config_tf"]+"/"+Config["camera_frame_id"]+"/transform",mtc)
  pb_msg.publish("rcalib::visp solver success")
  Terr,Rerr=error(mTc,poses,grids)
  err=Float64(); err.data=Terr; pb_stats.publish(err)

def cb_X2(f):
  global cTsAry,bTmAry
  save_input('rcalib_input.txt')
  call_visp()
  return


###############################################################
rospy.init_node('rcalib',anonymous=True)
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  print "get_param exception:",e.args
print "Config",Config

pb_msg=rospy.Publisher('/message',String,queue_size=1)
pb_err=rospy.Publisher('/error',String,queue_size=1)
pb_stats=rospy.Publisher('~error',Float64,queue_size=1)
pb_count=rospy.Publisher('~count',Int32,queue_size=1)
pb_Y0=rospy.Publisher('~cleared',Bool,queue_size=1)    #X0 done
pb_Y1=rospy.Publisher('~captured',Bool,queue_size=1)    #X1 done
pb_Y2=rospy.Publisher('~solved',Bool,queue_size=1)    #X2 done

rospy.Subscriber('~clear',Bool,cb_X0)
rospy.Subscriber('~capture',Bool,cb_X1)
rospy.Subscriber('~solve',Bool,cb_X2)

cb_X0(Bool())

tfBuffer = tf2_ros.Buffer()
tfListener = tf2_ros.TransformListener(tfBuffer)

config_tf=rospy.get_param(Config["config_tf"])
camera_info=config_tf[Config["camera_frame_id"]]
print "camera_info",camera_info
Config["mount_frame_id"]=camera_info["parent_frame_id"]
print "mount_frame",Config["mount_frame_id"]

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
