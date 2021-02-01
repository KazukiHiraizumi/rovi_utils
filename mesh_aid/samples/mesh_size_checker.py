#!/usr/bin/env python
# -*- coding: utf-8 -*-

import open3d as o3d
import numpy as np
import copy
import ransac_solver as solver
from scipy.spatial.transform import Rotation
from degraded import degraded_copy_point_cloud
import pprint
import time
import sys

DMY_MODEL_TRANS_RANGE_X = 100
DMY_MODEL_TRANS_RANGE_Y = 100
DMY_MODEL_TRANS_RANGE_Z = 100
DMY_MODEL_ROT_RANGE_X_DEGREE = 30
DMY_MODEL_ROT_RANGE_Y_DEGREE = 30
DMY_MODEL_ROT_RANGE_Z_DEGREE = 180

TEST_PARAM = [2,2,5]

def make_param(mesh, norm_rad_scale, ft_mesh_scale, ft_rad_scale):
    norm_rad = norm_rad_scale * mesh
    ft_mesh = ft_mesh_scale * mesh
    ft_rad = ft_rad_scale * ft_mesh
    dist_th = 1.5 * ft_mesh
    icp_th = 0.8 * mesh
    
    prms={
        "distance_threshold": dist_th,
        "feature_mesh":  ft_mesh,
        "feature_radius": ft_rad,
        "icp_threshold": icp_th,
        "normal_radius": norm_rad,
        "normal_min_nn": 0,
        "eval_threshold":0,        
        "rotate":0,
        "repeat":1}
    return prms

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    #o3d.visualization.draw_geometries([source_temp, target_temp],top=30,left=1380)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def random_pmone():
    return 2.0 * np.random.rand() - 1.0

def get_test_prm_key(s_normal_radius, s_feature_mesh, s_feature_radius):
    return '{:d}'.format(s_normal_radius) + "-" + \
           '{:d}'.format(s_feature_mesh) + "-" + \
           '{:d}'.format(s_feature_radius)

def main(model_path, mesh_size, param_scales):
    # 点群データ読み込み
    master_org = o3d.io.read_point_cloud(model_path)
    
    work_start_tm = time.time()
    test_results = {}

    master = copy.deepcopy(master_org)
    before_count = len(np.asarray(master_org.points))
    master = master_org.voxel_down_sample(voxel_size=mesh_size)
    print("master downsampling leaf_size=%f, before=%d, after=%d" % (mesh_size, before_count,len(np.asarray(master.points))) )
    
    master_count = len(np.asarray(master.points))
    
    
    norm_rad_scale = param_scales[0]
    ft_mesh_scale = param_scales[1]
    ft_rad_scale = param_scales[2]

    test_param = make_param(mesh_size , norm_rad_scale, ft_mesh_scale, ft_rad_scale)

    # ターゲットデータを作成
    target = degraded_copy_point_cloud(master, test_param["normal_radius"] , master_count )
    #target = degraded_copy_point_cloud(master, mesh_size/10 , master_count )

    # 回転角度をランダムに決定
    angles = [  DMY_MODEL_ROT_RANGE_X_DEGREE * random_pmone(),  # X軸回転.±30度の範囲
                DMY_MODEL_ROT_RANGE_Y_DEGREE * random_pmone(),  # Y軸回転.±30度の範囲
                DMY_MODEL_ROT_RANGE_Z_DEGREE * random_pmone()]  # Z軸回転.±180度の範囲
    rot_ans = Rotation.from_euler('zyx', angles, degrees=True)   # scipy.spatial.transform.Rotation型

    # 移動ベクトル.
    t = [ DMY_MODEL_TRANS_RANGE_X * random_pmone(), DMY_MODEL_TRANS_RANGE_Y * random_pmone(), DMY_MODEL_TRANS_RANGE_Z * random_pmone() ]
    
    ansRT = np.eye(4)
    ansRT[:3, :3] = rot_ans.as_matrix()
    ansRT[:3,  3] = t
        
    # ターゲットデータのコピーをとって、正解RTで変換
    target_tmp = copy.deepcopy(target)
    target_tmp.transform(ansRT)
                    
    # master_tmp = copy.deepcopy(master)
    solver.learn([solver.toNumpy(master)],test_param)

    test_start_tm = time.time()

    result=solver.solve([solver.toNumpy(target_tmp)],test_param)
            
    retRT = result["transform"][0]
    
    test_elapsed = time.time() - test_start_tm
            
    ## 結果の剛体変換行列から回転だけ取り出してscipy.spatial.transform.Rotation型へ
    rot_rsl = Rotation.from_matrix(retRT[:3, :3])
    #
    ## 誤差を計算
    ## 姿勢はクォータニオン(4次元ベクトル)に変えて差の二乗ノルムを誤差とする
    error_pose = np.linalg.norm(rot_ans.as_quat() - rot_rsl.as_quat(), ord=2)
    ## 位置は位置ベクトル(三次元ベクトル)の差の二乗ノルムを誤差とする
    error_position = np.linalg.norm(t - retRT[:3, 3], ord=2)
    
    draw_registration_result(master, target_tmp, retRT)  # 点群表示
    
    print("finished. elapsed=%f sec" % ( time.time() - work_start_tm ))
    print("================= Params =================")
    pprint.pprint(test_param)
    
    print("================= Results =================")
    print("mesh size: %.1f" % ( mesh_size ))
    print("param scales: %d-%d-%d" % (param_scales[0], param_scales[1], param_scales[2]))
    print("fitness: %f" % (result["fitness"][0]))
    print("rmse: %f" % (result["rmse"][0]))
    print("error_position: %f" % (error_position) )
    print("error_pose: %f" % (error_pose) )
    print("transform:")
    pprint.pprint(retRT)


if __name__ == '__main__':
    model_path = ""
    mesh_size = 1.0
    param_scales = TEST_PARAM
    
    if len(sys.argv) == 3:
       model_path = sys.argv[1]
       mesh_size = float(sys.argv[2])
    elif len(sys.argv) == 6:
       model_path = sys.argv[1]
       mesh_size = float(sys.argv[2])
       param_scales[0] = int(sys.argv[3])
       param_scales[1] = int(sys.argv[4])
       param_scales[2] = int(sys.argv[5])
    else:
       print("usage: %s <model path> <mesh size>" % (sys.argv[0]))
       print("  or : %s <model path> <mesh size> <normal radius scale> <feature mesh scale> <feature radius scale>" % (sys.argv[0]))
       exit(-1)
       
    main(model_path, mesh_size, param_scales)
