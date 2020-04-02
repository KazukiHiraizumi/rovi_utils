#!/usr/bin/python

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import sys
import json
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from rovi_utils import tflib
from rovi_utils.srv import TextFilter,TextFilterRequest,TextFilterResponse

def lookup(s):
  tokens=s.split(" ")
  if len(tokens)==0:
    return ""
  elif len(tokens)==1:
    base=None
    source="world"
    target=tokens[0]
  elif len(tokens)==2:
    base=None
    source=tokens[0]
    target=tokens[1]
  elif len(tokens)>=3:
    base=tokens[0]
    source=tokens[1]
    target=tokens[2]
  if base is None:
    try:
      tfs=tfBuffer.lookup_transform(source,target,rospy.Time(0))
    except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
      return ""
    else:
      return json.dumps(tflib.tf2dict(tfs.transform))
  else:
    try:
      bft=tfBuffer.lookup_transform(base,target,rospy.Time(0))
      sfb=tfBuffer.lookup_transform(source,base,rospy.Time(0))
    except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
      return ""
    else:
      bTt=tflib.toRT(bft.transform)
      sTb=tflib.toRT(sfb.transform)
      T=np.dot(bTt,sTb)
      return json.dumps(tflib.tf2dict(tflib.fromRT(T)))

def query(req):
  res=TextFilterResponse()
  res.data=lookup(req.data)
  print "tf_lookup req",req.data
  print "tf_lookup res",res.data
  return res

########################################################
rospy.init_node('tf_lookup',anonymous=True)
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)

if filter(lambda s: s.startswith("__name:="),sys.argv):
  print "tf_lookup may be launched by roslaunch"
  s=rospy.Service('/tf_lookup/query', TextFilter, query)
  rospy.spin()
else:
  rospy.sleep(1)
  while not rospy.is_shutdown():
#  line=sys.stdin.readline()
    line=raw_input()
    stf=lookup(line)
    if len(stf)==0: continue
    else: sys.stdout.write(stf+"\n")
