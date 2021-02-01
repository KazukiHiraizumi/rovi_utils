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
import os
from contextlib import redirect_stdout

DMY_MODEL_TRANS_RANGE_X = 100
DMY_MODEL_TRANS_RANGE_Y = 100
DMY_MODEL_TRANS_RANGE_Z = 100
DMY_MODEL_ROT_RANGE_X_DEGREE = 30
DMY_MODEL_ROT_RANGE_Y_DEGREE = 30
DMY_MODEL_ROT_RANGE_Z_DEGREE = 180

MESH_SIZES = [ 1 ,1.5 , 2, 2.5, 3 , 4, 5, 10 ]
MESH_SIZES.reverse()

TEST_PARAMS = [
    [2,2,5],
    [2,2,6],
    [3,2,4],
    [3,2,5],
    [3,2,6]
]

PASS_RATIO_TH = 0.999

POSITION_ERR_TH = 1
POSE_ERR_TH = 0.002
POSE_ERR_TH_EXCLUDE = 2.0

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

def draw_registration_result(source, target, transformation,passed):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    if passed:
        target_temp.paint_uniform_color([0, 0.651, 0.929])
    else:
        target_temp.paint_uniform_color([231/255, 76/255, 60/255])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def random_pmone():
    return 2.0 * np.random.rand() - 1.0

def get_test_prm_key(s_normal_radius, s_feature_mesh, s_feature_radius):
    return '{:d}'.format(s_normal_radius) + "-" + \
           '{:d}'.format(s_feature_mesh) + "-" + \
           '{:d}'.format(s_feature_radius)

def is_exclude_pose_error(val):
    return round(val,4) == POSE_ERR_TH_EXCLUDE


def main( model_path , repeat ):
    # 点群データ読み込み
    master_org = o3d.io.read_point_cloud(model_path)
    
    work_start_tm = time.time()
    test_results = {}

    test_passed = False
    
    select_mesh_size = None
    select_test_param = []
    select_param = {}
    
    for mesh_size in MESH_SIZES:
        master = copy.deepcopy(master_org)
        before_count = len(np.asarray( master_org.points ))
        master = master_org.voxel_down_sample( voxel_size = mesh_size )
        after_count = len(np.asarray( master.points ))
        
        print("[%3.1f] test start. master downsampling. before=%d, after=%d (%.1f%%)" % ( mesh_size, before_count,after_count,after_count/before_count*100))
        master_count = len( np.asarray(master.points) )
        
        for test_param in TEST_PARAMS:
            norm_rad_scale = test_param[0]
            ft_mesh_scale  = test_param[1]
            ft_rad_scale   = test_param[2]

            param = make_param( mesh_size , norm_rad_scale, ft_mesh_scale, ft_rad_scale )
            print("  <%d-%d-%d> start." % ( norm_rad_scale, ft_mesh_scale, ft_rad_scale ))
            
            # master_tmp = copy.deepcopy(master)
            solver.learn([solver.toNumpy(master)],param)
            ok_num = 0
            ng_num = 0
            test_passed = True
            for n in range(repeat):
                # ターゲットデータを作成
                target = degraded_copy_point_cloud(master, param["normal_radius"], master_count )
                #target = degraded_copy_point_cloud(master, mesh_size/10, master_count )

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
                
                test_start_tm = time.time()
                with redirect_stdout(open(os.devnull, 'w')):
                    result= solver.solve([solver.toNumpy(target_tmp)],param)
                retRT = result["transform"][0]
                
                test_elapsed = time.time() - test_start_tm
                        
                ## 結果の剛体変換行列から回転だけ取り出してscipy.spatial.transform.Rotation型へ
                rot_rsl = Rotation.from_matrix(copy.deepcopy(retRT[:3, :3]))
                
                ## 誤差を計算
                ## 姿勢はクォータニオン(4次元ベクトル)に変えて差の二乗ノルムを誤差とする
                error_pose = np.linalg.norm(rot_ans.as_quat() - rot_rsl.as_quat(), ord=2)
                #print("-------- answer quat----------")
                #print(rot_ans.as_quat())
                #print("-------- result quat----------")
                #print(rot_rsl.as_quat())
                
                ## 位置は位置ベクトル(三次元ベクトル)の差の二乗ノルムを誤差とする
                error_position = np.linalg.norm(t - retRT[:3, 3], ord=2)
                
                passed = False
                if error_position > POSITION_ERR_TH:
                    print("        error: position unmatching. val=%.5f over %.5f" % (error_position,POSITION_ERR_TH))
                    #pass
                elif error_pose > POSE_ERR_TH and not is_exclude_pose_error(error_pose):
                    print("        error: pose unmatching. val=%.5f over %.5f" % (error_pose,POSE_ERR_TH))
                    #pass
                else:
                    passed = True
                
                if passed:
                    ok_num += 1
                else:
                    ng_num += 1
                
                estim_max_pass_ratio = ( repeat - n - 1 + ok_num )/repeat
                
                #print("finished. elapsed=%f sec" % ( time.time() - work_start_tm ))
                ##print("================= Params =================")
                ##pprint.pprint(param)
                
                print("    %3d/%d ok=%3d, ng=%3d, pass=%4.1f%%, Results: fitness=%.3f, rmse=%.1f, err_position=%.3f, error_pose=%.4f" % 
                    ( n + 1, repeat , ok_num, ng_num , ok_num/repeat * 100, result["fitness"][0] , result["rmse"][0] , error_position, error_pose ))
                #print("transform:")
                #pprint.pprint(retRT)
                
                #if passed:
                #if is_exclude_pose_error(error_pose):
                #if error_position > POSITION_ERR_TH:
                #    draw_registration_result(master, target_tmp, retRT)  # 点群表示
                #    draw_registration_result(master, target_tmp, retRT, passed)
                
                if estim_max_pass_ratio < PASS_RATIO_TH:
                    print("        !!! ABORT !!!: estimate pass ratio is under threshold.  th=%.3f%% > estim_pass_ratio=%.3f%%" % 
                    ( PASS_RATIO_TH *100,estim_max_pass_ratio * 100) )
                    test_passed = False
                    break
                    
            
            
            if test_passed:
                select_param = param
                select_test_param = test_param
                select_mesh_size = mesh_size
                break
        if test_passed:
            break


    print("================= Results =================")
    if not select_mesh_size:
        print("mesh size calc failed. elapsed=%d sec" % ( time.time() - work_start_tm ) )
    else:
        print("elapsed: %d sec" % ( time.time() - work_start_tm ) );
        print("param scales: %d-%d-%d" % (test_param[0], test_param[1], test_param[2]))
        print("recommend mesh size: %.1f" % (select_mesh_size))
        print("Params:")
        pprint.pprint(select_param)

if __name__ == '__main__':
    model_path = ""
    mesh_size = 1.0
    if len(sys.argv) == 3:
       model_path = sys.argv[1]
       repeat = int(sys.argv[2])
    else:
       print("usage: %s <model path> <repeat>" % (sys.argv[0]))
       exit(-1)
       
    main( model_path, repeat )
