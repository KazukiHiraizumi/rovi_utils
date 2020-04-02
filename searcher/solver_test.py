#!/usr/bin/python

import numpy as np
import open3d as o3
import copy
from rovi_utils import tflib
#from rovi_utils import ransac_solver as solver

Param={
  "normal_max_nn":30,
  "normal_radius":0.002,
  "feature_max_nn":100,
  "feature_radius":0.005,
  'distance_threshold':0.1,
  'icp_threshold':0.001
}

print "Prepare model"
model=o3.io.read_point_cloud("../data/model.ply")
model.estimate_normals(o3.geometry.KDTreeSearchParamHybrid(radius=Param["normal_radius"],max_nn=Param["normal_max_nn"]))
print "model points",np.asarray(model.points)
print "model normals",np.asarray(model.normals)
o3.io.write_point_cloud("learn.ply",model)

#solver.learn([np.asarray(model)],{})
#scene=o3.read_point_cloud("../data/sample.ply")
#result=solver.solve([np.asarray(scene)],{})
#Tmat=result["transform"]
#score=result["fitness"]
#print "feature matching result",Tmat[0],score[0]

#P=copy.deepcopy(toNumpy(solver.modPcArray[0]))
#source=o3.transform(Tmat,P)
#target=solver.scnPcArray[0]
#source.paint_uniform_color([1, 0.706, 0])
#target.paint_uniform_color([0, 0.651, 0.929])
#o3.draw_geometries([source, target])
