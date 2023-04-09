import open3d as o3d
import os
from os.path import exists, join
def pclViewer(pclQ):
    vis = o3d.visualization.Visualizer()
    vis.create_window('PCD', width=1280, height=720)
    ctr = vis.get_view_control()
    #ctr.set_zoom(1.5)
    pcd = None
    #ctr.scale(1.5)
    while True:
        if pcd is not None:

            vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        if pclQ.empty():
            continue
        else:
            xx = pclQ.get()
            pcd = o3d.io.read_point_cloud(join(os.getcwd(), "log", "pcd", "originawithpoint.pcd"))
            #pcd.transform([[3, 0, 0, 0], [0, -3, 0, 0], [0, 0, -3, 0], [0, 0, 0, 3]])
            pcd.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            #pcd.scale(5.2, center=pcd.get_center())
            #pcd.scale *= 1.5
            #ctr.set_zoon(1.5)e
            vis.clear_geometries()
            vis.add_geometry(pcd)
            pcd.scale(1.5, center=pcd.get_center())
            #param = vis.get_view_control().convert_to_pinhole_camera_parameters()
            #print(param)
