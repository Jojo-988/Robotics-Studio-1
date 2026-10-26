import open3d as o3d

mesh = o3d.io.read_triangle_mesh("Environment Reconstruction.ply")
mesh.compute_vertex_normals()

o3d.visualization.draw_geometries(
    [mesh],
    window_name="Forest 3D Reconstruction",
    mesh_show_back_face=True
)