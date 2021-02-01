#!/usr/bin/python

import cv2
import numpy as np
import math
import copy
import roslib
import rospy
import tf
import tf2_ros
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from rovi_utils import tflib

Config={
  "config_tf":"/config_tf",
  "camera_frame_id":"camera",
  "mount_frame_id":"mount",
  "flange_frame_id":"flange",
  "board_frame_id":"gridboard",
  "solve_frame_id":"camera/capture0",
  "degs":[[15,0,0],[-15,0,0],[0,15,0],[0,-15,0],
          [15,15,0],[-15,15,0],[15,-15,0],[-15,-15,0],
          [0,0,60],[0,0,120],[0,0,-60],[0,0,-120]],
}

def get_pos():
  global bTs,cTm,sTc,bTc,cTs,sTm,wTb,count,pb_Y2
  if count<1:
    # not start yet.
    pb_err.publish("planner::captures are not start yet")
    done=Bool(); done.data=False; pb_Y2.publish(done)
    return

  if count>len(Config['degs']):
    # done all captures.
    pb_err.publish("planner::captures done")
    done=Bool(); done.data=False; pb_Y2.publish(done)
    return

  if count==1:
    key="/config_tf/camera/transform"
    rcalib=Transform(); rcalib.rotation.w=1.;
    if rospy.has_param(key):
      rcalib=tflib.dict2tf(rospy.get_param(key))
    else:
      pb_err.publish("planner::default calib value is not exists!")
      done=Bool(); done.data=False; pb_Y2.publish(done)
    try:
      cts=tfBuffer.lookup_transform(Config["camera_frame_id"], Config["board_frame_id"], rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pb_err.publish("planner::gridboard lookup failed")
      done=Bool(); done.data=False; pb_Y2.publish(done)
      return
    try:
      if Config["mount_frame_id"] == "world":
        btm=tfBuffer.lookup_transform("world",Config["flange_frame_id"], rospy.Time())  #The board may be held on the flange
      else:
        btm=tfBuffer.lookup_transform("world",Config["mount_frame_id"], rospy.Time())  #The camera mount may be attached on some link
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pb_err.publish("planner::robot lookup failed")
      done=Bool(); done.data=False; pb_Y2.publish(done)
      return
    try:
      wTb=tfBuffer.lookup_transform("world","base", rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pb_err.publish("planner::base lookup failed")
      done=Bool(); done.data=False; pb_Y2.publish(done)
      return
    if Config["mount_frame_id"]=="world":
      cTs=tflib.toRT(cts.transform)
      sTc=np.linalg.inv(cTs)
      bTm=tflib.toRT(btm.transform)
      bTc=tflib.toRT(rcalib)
      cTb=np.linalg.inv(bTc)
      sTm=np.dot(np.dot(sTc,cTb),bTm)
    else:
      cTs=tflib.toRT(cts.transform)
      mTc=tflib.toRT(rcalib)
      bTm=tflib.toRT(btm.transform)
      bTs=np.dot(np.dot(bTm,mTc),cTs)
      cTm=np.linalg.inv(mTc)
      sTc=np.linalg.inv(cTs)

  idx=count-1
  deg=Config['degs'][idx]
  print 'deg[{}]={}'.format(idx,deg)
  rad=np.array([map(lambda x: math.radians(x),deg)])
  mat,jac=cv2.Rodrigues(rad)
  rmat=np.eye(4)
  rmat[:3,:3]=mat
  if Config["mount_frame_id"]=="world":
    cts=np.dot(rmat,cTs)
    cts[:3,3]=cTs[:3,3]
    btm=np.dot(np.dot(bTc,cts),sTm)
  else:
    stc=np.dot(rmat,sTc)
    btm=np.dot(np.dot(bTs,stc),cTm)

  dtnow=rospy.Time.now()
  tfs=[]
  tf=TransformStamped()
  tf.header.stamp=dtnow
  tf.header.frame_id="world"
  tf.child_frame_id=Config["solve_frame_id"]+"/solve0"
  tf.transform=tflib.fromRT(btm)
  tfs.append(copy.deepcopy(tf))
  tf=TransformStamped()
  tf.header.stamp=dtnow
  tf.header.frame_id="world"
  tf.child_frame_id="camera/master0"   # dummy
  tf.transform=wTb.transform
  tfs.append(copy.deepcopy(tf))
  broadcaster.sendTransform(tfs)
  done=Bool(); done.data=True; pb_Y2.publish(done)
  return

def cb_X2(f):
  get_pos()
  return

def cb_count(f):
  global count
  count=f.data
  return

###############################################################
rospy.init_node('planner',anonymous=True)

pb_err=rospy.Publisher('/error',String,queue_size=1)
pb_Y2=rospy.Publisher('~solved',Bool,queue_size=1)    #X2 done
rospy.Subscriber('~solve',Bool,cb_X2)
rospy.Subscriber('~count',Int32,cb_count)

tfBuffer = tf2_ros.Buffer()
tfListener = tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

config_tf=rospy.get_param(Config["config_tf"])
camera_info=config_tf[Config["camera_frame_id"]]
print "camera_info",camera_info
Config["mount_frame_id"]=camera_info["parent_frame_id"]
print "mount_frame",Config["mount_frame_id"]

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
