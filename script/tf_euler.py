#!/usr/bin/python

import numpy as np
import rospy
from rovi_utils.srv import TextFilter,TextFilterRequest,TextFilterResponse
import tf
import sys
import cv2
from geometry_msgs.msg import Transform
from rovi_utils import tflib

axes="1"
unit="rad"

def euler_to_quat(toks):
  p=np.array(map(float,toks))
  if axes.startswith("rvec"):
    rmat,jac=cv2.Rodrigues(p)
    m=np.eye(4)
    m[:3,:3]=rmat
    q=tf.transformations.quaternion_from_matrix(m)
  else:
    if not unit.startswith("rad"): p=p*np.pi/180
    q=tf.transformations.quaternion_from_euler(p[0],p[1],p[2],axes=axes)
  return q

def quat_to_euler(toks):
  p=map(float,toks)
  if axes.startswith("rvec"):
    rmat=tf.transformations.quaternion_matrix(p)
    vec,jac=cv2.Rodrigues(rmat[:3,:3])
    e=np.ravel(vec)
  else:
    e=np.array(tf.transformations.euler_from_quaternion(p,axes=axes))
    if not unit.startswith("rad"): e=e*180/np.pi
  return e

def query(req):
  global axes,unit
  res=TextFilterResponse()
  msgs=req.data.strip().split('\n')
  for msg in msgs:
    elm=msg.split(',')
    if len(elm)==1:
      elm=msg.split(' ')
      elm=[a for a in elm if len(a)>0]
    if len(elm)==1:  #set axes
      axes=elm[0]
      resadd='ok'
    elif len(elm)==2: #set axes and unit of angle
      axes=elm[0]
      unit=elm[1]
      resadd='ok'
    elif len(elm)==3: #euler(3)->quat(4)
      q=euler_to_quat(elm)
      resadd=','.join(map(str,q))
    elif len(elm)==4: #quat(4)->euler(3)
      e=quat_to_euler(elm)
      resadd=','.join(map(str,e))
    else:
      resadd=''
    if len(resadd)>0:
      if len(res.data)>0: res.data=res.data+'\n'
      res.data=res.data+resadd
  return res

if filter(lambda s: s.startswith("__name:="),sys.argv):
  print "tf_euler may be launched by roslaunch"
  axes="sxyz"
  unit="deg"
  rospy.init_node('tf_euler')
  s=rospy.Service('/tf_euler/query', TextFilter, query)
  rospy.spin()

else:
  if len(sys.argv)>=2:
    axes=sys.argv[1]
  if len(sys.argv)>=3:
    unit=sys.argv[2]

  try:
    loop=int(axes)
  except Exception as e:
    loop=-1

  while loop<0:
    try:
      line=raw_input()
    except Exception as e:
      break
    tokens=line.split(",")

    if len(tokens)==3:
      q=euler_to_quat(tokens)
      sys.stdout.write(','.join(map(str,q))+"\n")
      sys.stdout.flush()
    elif len(tokens)==4:
      e=quat_to_euler(tokens)
      sys.stdout.write(','.join(map(str,e))+"\n")
      sys.stdout.flush()

  while loop>0:
    q=tf.transformations.random_quaternion()
    sys.stdout.write(str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+"\n")
    loop=loop-1
