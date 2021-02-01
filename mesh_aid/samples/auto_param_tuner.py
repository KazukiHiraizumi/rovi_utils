import open3d as o3d
import numpy as np
import copy
import ransac_solver as solver
from scipy.spatial.transform import Rotation
from degraded import degraded_copy_point_cloud
import pprint
import time
import sys

WORLD_SCALE  = 1

DMY_MODEL_TRANS_RANGE_X = 100 * WORLD_SCALE
DMY_MODEL_TRANS_RANGE_Y = 100 * WORLD_SCALE
DMY_MODEL_TRANS_RANGE_Z = 100 * WORLD_SCALE
DMY_MODEL_ROT_RANGE_X_DEGREE = 30
DMY_MODEL_ROT_RANGE_Y_DEGREE = 30
DMY_MODEL_ROT_RANGE_Z_DEGREE = 180

MESHES = [2,5,10]
NORMAL_RADIUS_SCALES = ( 2, 3 , 4 ) # * mesh
FEATURE_MESH_SCALES = [ 2 ] # * mesh
FEATURE_RADIUS_SCALES = ( 4, 5 , 6 ) # * feature_mesh

OUT_OF_RANK_VAL = 9999
RANK_VAL_SMALL_DIV_NUM = 5
FITNESS_MIN = 0.7
FITNESS_MAX = 1.0
FITNESS_RANK_VALUES = (0.99,0.98,0.97,0.95,0.90,0.85,0.8,FITNESS_MIN)

INLIER_RSME_MAX = 100
INLIER_RANK_VALUES = (1,2,3,5,10,15,20,50,INLIER_RSME_MAX)

MOVE_ERR_MAX = 200
MOVE_ERR_RANK_VALUES = (0.1,0.3,0.5,1,5,10,20,30,50,100,MOVE_ERR_MAX)

POSE_ERR_MAX = 200
POSE_ERR_RANK_VALUES = (0.1,0.3,0.5,1,5,10,20,30,50,100,POSE_ERR_MAX)

PROC_TIME_MAX = 20
PROC_TIME_RANK_VALUES = (0.3,0.5,1.0,2.0,3,4,5,8,10,PROC_TIME_MAX)


def make_param(mesh, norm_rad_scale, ft_mesh_scale, ft_rad_scale):
    norm_rad = norm_rad_scale * mesh
    ft_mesh = ft_mesh_scale * mesh
    ft_rad = ft_rad_scale * ft_mesh
    dist_th = 1.5 * ft_mesh
    icp_th = 0.5 * mesh
    
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
    o3d.visualization.draw_geometries([source_temp, target_temp],top=0,left=1480)


def random_pmone():
    return 2.0 * np.random.rand() - 1.0

def get_test_prm_key(s_normal_radius, s_feature_mesh, s_feature_radius):
    return '{:d}'.format(s_normal_radius) + "-" + \
           '{:d}'.format(s_feature_mesh) + "-" + \
           '{:d}'.format(s_feature_radius)

def calc_rank_val_higher(rank_values, val, max):
    result = OUT_OF_RANK_VAL
    for i in range(len(rank_values)):
        if rank_values[i] < val:
            max_val = max
            if i >= len(rank_values) :
                max_val = rank_values[i-1]
            span = ( max_val - rank_values[i]) / RANK_VAL_SMALL_DIV_NUM
            rank_val_small = ( val  - rank_values[i] ) / span / 10
            result = i + rank_val_small
            break
    return result

def calc_rank_val_lower(rank_values, val):
    result = OUT_OF_RANK_VAL
    for i in range(len(rank_values)):
        if rank_values[i] > val:
            min = 0
            rank_val_small = 0
            if i >= 1:
                min = rank_values[i-1]
            span = (rank_values[i] - min) / RANK_VAL_SMALL_DIV_NUM
            rank_val_small = ( val  - min ) / span / 10
            result = i + rank_val_small
            break
    return result
    
def evaluation (test_results,sample_num):
    passed_results = {}
    failure_resuls = []
    
    i = 0
    for k, v in test_results.items():
        passed = False
        
        i = i + 1
        
        if len(v) > 0 :
            passed = True
            for result in v:
                if FITNESS_MIN > result["fitness"]:
                   print("fitness: key=%s is failure. min=%f val=%f" % (k,FITNESS_MIN,result["fitness"] ) )
                   passed=False
                   break;
                elif INLIER_RSME_MAX < result["rmse"]:
                   print("rmse: key=%s is failure. max=%f val=%f" % (k,INLIER_RSME_MAX,result["rmse"] ) )
                   passed=False
                   break;
                elif MOVE_ERR_MAX < result["move_err"]:
                   print("move_err: key=%s is failure. max=%f val=%f" % (k,MOVE_ERR_MAX,result["move_err"] ) )
                   passed=False
                   break;
                elif POSE_ERR_MAX < result["pose_err"]:
                   print("pose_err: key=%s is failure. max=%f val=%f" % (k,POSE_ERR_MAX,result["pose_err"] ) )
                   passed=False
                   break;
                elif PROC_TIME_MAX < result["proc_time"]:
                   print("proc_time: key=%s is failure. max=%f val=%f" % (k,PROC_TIME_MAX,result["proc_time"] ) )
                   passed=False
                   break;
        
        print("key=%s : %s" % (k, "passed" if passed else "failure") )
        for result in v:
            print("    [%2d] fitness=%.3f, rmse=%5.2f, move_err=%7.4f, pose_err=%9.4f, proc_time=%6.3f" % 
                (i, result["fitness"], result["rmse"], result["pose_err"], result["move_err"], result["proc_time"]) )

        if not passed:
            failure_resuls.append(k)
        else:
            passed_results[k] = v
    
    #if len(passed_results) == 0:
    #    print("No evaluation-worthy results were obtained...")
    #    return
    
    eval_results = {}
    eval_sums = {}
    for k, v in test_results.items():
        fitness_avg  = 0
        rmse_avg     = 0
        move_err_avg = 0
        pose_err_avg = 0
        proc_time_avg= 0
        
        for result in v:
            fitness_avg  += result["fitness"]
            rmse_avg     += result["rmse"]
            move_err_avg += result["move_err"]
            pose_err_avg += result["pose_err"]
            proc_time_avg+= result["proc_time"]
        
        dmy_num = len(v)
        
        fitness_avg   = fitness_avg / dmy_num
        rmse_avg      = rmse_avg / dmy_num
        move_err_avg  = move_err_avg / dmy_num
        pose_err_avg  = pose_err_avg / dmy_num 
        proc_time_avg = proc_time_avg / dmy_num 
        
        if k in passed_results.keys():
            fitness_eval =calc_rank_val_higher(FITNESS_RANK_VALUES, fitness_avg, FITNESS_MAX)
            rmse_eval = calc_rank_val_lower(INLIER_RANK_VALUES,rmse_avg)
            move_err_eval = calc_rank_val_lower(MOVE_ERR_RANK_VALUES,move_err_avg)
            pose_err_eval = calc_rank_val_lower(POSE_ERR_RANK_VALUES,pose_err_avg)
            proc_time_eval = calc_rank_val_lower(PROC_TIME_RANK_VALUES,proc_time_avg)
            eval_sum = fitness_eval + rmse_eval+  move_err_eval + pose_err_eval + proc_time_eval
        else:
            fitness_eval = OUT_OF_RANK_VAL
            rmse_eval = OUT_OF_RANK_VAL
            move_err_eval = OUT_OF_RANK_VAL
            pose_err_eval = OUT_OF_RANK_VAL
            proc_time_eval = OUT_OF_RANK_VAL
            eval_sum = OUT_OF_RANK_VAL
        
        eval_sums[k] = eval_sum
        
        eval_results[k] = {
            "mesh"          : result["mesh"],
            "params"        : result["params"],
            "eval_sum"      : eval_sum,
            "fitness_avg"   : fitness_avg,
            "fitness_eval"  : fitness_eval,
            "rmse_avg"      : rmse_avg,
            "rmse_eval"     : rmse_eval,
            "move_err_avg"  : move_err_avg,
            "move_err_eval" : move_err_eval,
            "pose_err_avg"  : pose_err_avg,
            "pose_err_eval" : pose_err_eval,
            "proc_time_avg" : proc_time_avg,
            "proc_time_eval": proc_time_eval,
        }
        
        print("key=%s mesh=%.1f, params=%s, eval_sum=%1.f" % ( k , result["mesh"], result["params"] ,eval_sums[k] ) )
        
        
    # pprint.pprint(eval_results)
    #print(eval_sums)
    eval_sums_sorted = sorted(eval_sums.items(), key=lambda x:x[1])
    #print(eval_sums_sorted)
    #print(failure_resuls)
    
    no = 0
    total_num = len(test_results)
    with open("result_eval_%03d.csv" % ( sample_num ) , mode='w') as f:
        f.write("mesh,params,rank,eval_sum,fitness_avg,fitness_eval,rmse_avg,rmse_eval,move_err_avg,move_err_eval,pose_err_avg,pose_err_eval,proc_time_avg,proc_time_eval\n")
        no = 0
        for eval_sum in eval_sums_sorted :
            no += 1
            k = eval_sum[0]
            eval_result=eval_results[k]
            f.write("%.1f,'%s,%d,%.1f,%f,%.1f,%f,%.1f,%f,%.1f,%f,%.1f,%f,%.1f\n" % \
                (eval_result["mesh"] , eval_result["params"], total_num - no,eval_result["eval_sum"],eval_result["fitness_avg"],eval_result["fitness_eval"], \
                 eval_result["rmse_avg"], eval_result["rmse_eval"],eval_result["move_err_avg"], eval_result["move_err_eval"], \
                 eval_result["pose_err_avg"], eval_result["pose_err_eval"], eval_result["proc_time_avg"], eval_result["proc_time_eval"]))
    
    for eval_sum in eval_sums_sorted :
        #print(eval_sum)
        k = eval_sum[0]
        eval_result=eval_results[k]
        print("mesh=%.1f params=%s, eval_sum=%3f, fitness_avg=%f(%.1f), rmse_avg=%f(%1.f), move_err_avg=%f(%.1f), pose_err_avg=%f(%1.f), proc_time_avg=%f(%.1f)" % 
            ( eval_result["mesh"], eval_result["params"], eval_result["eval_sum"], eval_result["fitness_avg"], eval_result["fitness_eval"], eval_result["rmse_avg"], eval_result["rmse_eval"], eval_result["move_err_avg"], eval_result["move_err_eval"], eval_result["pose_err_avg"], eval_result["pose_err_eval"], eval_result["proc_time_avg"], eval_result["proc_time_eval"] ))
        
     
def main(model_path, count):
    # 点群データ読み込み
    master_org = o3d.io.read_point_cloud(model_path)
    
    try_num = 0
    
    start_tm = time.time()
    test_results = {}
    try_max_num = len(MESHES) * len(NORMAL_RADIUS_SCALES) * len(FEATURE_MESH_SCALES) * len(FEATURE_RADIUS_SCALES) * count
    
    for mesh in MESHES:
        master = copy.deepcopy(master_org)
        before_count = len(np.asarray(master_org.points))
        master = master_org.voxel_down_sample(voxel_size=mesh)
        print("master downsampling leaf_size=%f, before=%d, after=%d" % (mesh, before_count,len(np.asarray(master.points))) )
        
        master_count = len(np.asarray(master.points))
        
        for norm_rad_scale in NORMAL_RADIUS_SCALES:
            norm_rad = norm_rad_scale * mesh
            
            # ターゲットデータを作成
            target = degraded_copy_point_cloud(master, norm_rad , master_count )
            
            # ターゲットデータを指定された回数だけランダムに移動して位置決めしてみる
            for n in range(count):
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
                
                for ft_mesh_scale in FEATURE_MESH_SCALES:
                    for ft_rad_scale in FEATURE_RADIUS_SCALES:
                        ft_mesh = ft_mesh_scale * mesh
                        ft_rad = ft_rad_scale * mesh
                        dist_th = 5 * mesh
                        icp_th = 1.5 * mesh
                        
                        test_params=make_param(mesh, norm_rad_scale, ft_mesh_scale, ft_rad_scale)
                        
                        print("===== Param =====")
                        pprint.pprint(test_params)
                        
                        try_num = try_num + 1
                        
                        master_tmp = copy.deepcopy(master)
                        solver.learn([solver.toNumpy(master_tmp)],test_params)
        
                        test_start_tm = time.time()
                        print("%05d: mesh=%.1f dmy_no=%3d [%d][%d][%d] start. %d/%d(%.2f)" %
                            (try_num, mesh, n, norm_rad_scale, ft_mesh_scale, ft_rad_scale, try_num,try_max_num,try_num/try_max_num * 100.0) )
           
                        result=solver.solve([solver.toNumpy(target_tmp)],test_params)
        
                        retRT = result["transform"][0]
                        #print("===== Transform =====")
                        #pprint.pprint(retRT)
                        
                        test_elapsed = time.time() - test_start_tm
                        
                        ## 結果の剛体変換行列から回転だけ取り出してscipy.spatial.transform.Rotation型へ
                        rot_rsl = Rotation.from_matrix(retRT[:3, :3])
                        #
                        ## 誤差を計算
                        ## 姿勢はクォータニオン(4次元ベクトル)に変えて差の二乗ノルムを誤差とする
                        error_pose = np.linalg.norm(rot_ans.as_quat() - rot_rsl.as_quat(), ord=2)
                        ## 位置は位置ベクトル(三次元ベクトル)の差の二乗ノルムを誤差とする
                        error_position = np.linalg.norm(t - retRT[:3, 3], ord=2)
                        
                        params_str = get_test_prm_key( norm_rad_scale, ft_mesh_scale, ft_rad_scale );
                        
                        test_key = "%.1f-%s" % ( mesh, params_str )
                        test_result ={
                            "mesh":mesh,
                            "params":params_str,
                            "fitness":result["fitness"][0],
                            "rmse":result["rmse"][0],
                            "pose_ans":rot_ans.as_quat(),
                            "pose_rsl":rot_rsl.as_quat(),
                            "pose_err":error_pose,
                            "move_ans":t,
                            "move_rsl":retRT[:3, 3],
                            "move_err":error_position,
                            "proc_time":test_elapsed 
                        }
                        if test_key not in test_results:
                            test_results[test_key] = []
                        
                        test_results[test_key].append(test_result)
                        
                        print("    finished. elapsed=%.3f s, fitness=%.3f, rmse=%5.2f, move_err=%6.3f, pose_err=%6.3f" % 
                            ( test_elapsed,
                              test_result["fitness"],
                              test_result["rmse"],
                              test_result["move_err"],
                              test_result["pose_err"]) )
                        
                        #pprint.pprint(result)
                        
                        #draw_registration_result(master, target_tmp, retRT)  # 点群表示    
    
    with open("result_test_%03d.csv" % (count) , mode='w') as f:
        f.write("mesh,params,dmy_no,fitness,rmse,move_err,pose_err,proc_time\n")
        for k, v in test_results.items():
            no = 0
            for test_result in v:
                no += 1
                f.write("%.1f,'%s,%d,%f,%f,%f,%f,%f\n" % \
                    ( mesh, test_result["params"] , no, test_result["fitness"] ,test_result["rmse"], test_result["move_err"], test_result["pose_err"], test_result["proc_time"]))

    
    evaluation(test_results,count)
    
    print("finished. elapsed=%f sec" %( time.time() - start_tm ))

if __name__ == '__main__':
    if len(sys.argv) != 3:
       print("usage: %s <model path> <repeat>" % (sys.argv[0]))
       exit(-1)
       
    model_path = sys.argv[1]
    repeat = int(sys.argv[2])
    
    main(model_path, repeat )
