#!/usr/bin/python

import numpy as np
import roslib
import rospy
import open3d as o3d
import os
import sys
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform
from rovi_utils import tflib

Args={"wd":""}

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def cb_save_ply(msg):
  d=outPn.astype(np.float32)
  pc=o3d.PointCloud()
  pc.points=o3d.Vector3dVector(d)
  print 'save model PC',d.dtype,d.shape
  o3d.write_point_cloud(Args["wd"],pc,True,False)
  return

def cb_load_ply(msg):
  global outPn
  pcd=o3d.read_point_cloud(Args["wd"])
  outPn=np.reshape(np.asarray(pcd.points),(-1,3))
  print "load",len(outPn)
  pub.publish(np2F(outPn))
  f=Bool();f.data=True;pub_ld.publish(f)

def cb_ps(msg):
  global outPn
  outPn=np.reshape(msg.data,(-1,3))
  return

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key]=tokens[1]
  return args

########################################################

rospy.init_node("filer",anonymous=True)
###Input topics
rospy.Subscriber("~in/floats",numpy_msg(Floats),cb_ps)
rospy.Subscriber("~save",Bool,cb_save_ply)
rospy.Subscriber("~load",Bool,cb_load_ply)
pub_ld=rospy.Publisher("~loaded",Bool,queue_size=1)
pub=rospy.Publisher("~out/floats",numpy_msg(Floats),queue_size=1)

Args.update(parse_argv(sys.argv))
print "Arg",Args

#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
