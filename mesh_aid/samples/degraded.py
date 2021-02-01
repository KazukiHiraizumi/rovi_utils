import open3d as o3d
import numpy as np


def degraded_copy_point_cloud(cloud, normal_radius, n_newpc):
    """
    与えられた点群データからPoisson表面を再構成し、頂点データからランダムに点を取り出して
    新たな点群を作成する.

    Parameters
    ----------
    cloud : open3d.geometry.PointCloud
        入力点群
    normal_radius : float
        法線ベクトル計算時の近傍半径(Poisson表面構成では必要なので).既に法線ベクトルが
        計算されていれば無視される.
    n_newpc : int
        新しい点群データに含まれる点の数
    """
    if np.asarray(cloud.normals).shape[0] == 0:
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=30))

    # Poisson表面作成    
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(cloud, depth=9)

    # 密度値が低い所の面を削除
    mesh.remove_vertices_by_mask(densities < np.quantile(densities, 0.1))

    # メッシュの頂点データから入力の点群と同じ数の点を取り出す(randomで)
    n_vertic = np.asarray(mesh.vertices).shape[0]
    if n_vertic > n_newpc:
        indices = np.random.choice(np.arange(n_vertic), size=n_newpc, replace=False)
        points = np.asarray(mesh.vertices)[indices]
    else:
        print("Warning: Mesh vertices is {} (< {}).".format(n_vertic, n_newpc)) 
        points = np.asarray(mesh.vertices)
    
    # 新たな点群を作成する
    newcl = o3d.geometry.PointCloud()
    newcl.points = o3d.utility.Vector3dVector(points)
    return newcl


if __name__ == '__main__':
    
    cloud = o3d.io.read_point_cloud("surface.ply")
    print("Original Point Cloud")
    print(cloud)
    print(np.asarray(cloud.normals).shape)

    degcl = degraded_copy_point_cloud(cloud, 0.2, 10000)

    degcl.paint_uniform_color([1, 0.706, 0])
    o3d.visualization.draw_geometries([cloud, degcl])

    
