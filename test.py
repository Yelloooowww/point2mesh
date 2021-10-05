import open3d as o3d
import numpy as np
# print("Load a ply point cloud, print it, and render it")
# pcd0 = o3d.io.read_point_cloud("Urban_Circuit_LowRes_Scan_alpha_LowerFloor.pcd")
# pcd1 = o3d.io.read_point_cloud("Urban_Circuit_LowRes_Scan_alpha_UpperFloor.pcd")
# pcd2 = o3d.io.read_point_cloud("Urban_Circuit_LowRes_Scan_beta_LowerFloor.pcd")
# pcd3 = o3d.io.read_point_cloud("Urban_Circuit_LowRes_Scan_beta_UpperFloor.pcd")

# print(pcd)
# print(np.asarray(pcd.points))
# o3d.visualization.draw_geometries([pcd0])
# o3d.visualization.draw_geometries([pcd1])
# o3d.visualization.draw_geometries([pcd2])
# o3d.visualization.draw_geometries([pcd3])

# o3d.io.write_point_cloud("Urban_Circuit_LowRes_Scan_alpha_UpperFloor.xyz", pcd1)
# pcd = o3d.io.read_point_cloud("Urban_Circuit_LowRes_Scan_alpha_LowerFloor.pts")
# print(pcd)
# print(np.asarray(pcd.points))
# o3d.visualization.draw_geometries([pcd])
print("Testing IO for meshes ...")
mesh = o3d.io.read_triangle_mesh("catkin_ws/left.ply")
print(mesh)
o3d.visualization.draw_geometries([mesh])
