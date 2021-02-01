#!/usr/bin/python

import numpy as np
import roslib
import rospy
import copy
import os
import sys

Config={
  "set":[]
}

def endkey(key,dic,n):
  for k in dic:
    if type(dic[k]) is list:
      if n<1: n=1
      if n>len(dic[k]): n=len(dic[k])
      rospy.set_param(key+'/'+k,dic[k][n-1])
    else:
      endkey(key+'/'+k,dic[k],n)

def cb_param(msg):
  global Params
  for n,rc in enumerate(Config["set"]):  #to make "paramset" list
    try:
      param=rospy.get_param(rc)
      if "paramsetN" in param:
        setn=param["paramsetN"]
        if "paramset" in param:
          endkey(rc,param["paramset"],setn)
    except Exception as e:
      rospy.logwarn("Exception in parameset "+str(e))
  rospy.Timer(rospy.Duration(1),cb_param,oneshot=True)
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
rospy.init_node("paramset",anonymous=True)
Config.update(parse_argv(sys.argv))
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  print "get_param exception:",e.args
print "Config",Config

Config["set"]=eval(Config["set"].replace('[','["').replace(']','"]').replace(',','","'))

rospy.Timer(rospy.Duration(1),cb_param,oneshot=True) #to check change of param

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"

