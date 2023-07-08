import numpy as np
import open3d
import sys
import os
import glob


def save_view_point(pcd, filename):
    vis = open3d.visualization.Visualizer()
    vis.create_window(window_name='Open3D', width=1920, height=1080)
    vis.add_geometry(pcd)

    # vis.get_render_option().load_from_json('renderoption.json')
    vis.run()  # user changes the view and press "q" to terminate
    param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    open3d.io.write_pinhole_camera_parameters(filename, param)
    # vis.destroy_window(
        
def load_view_point(pcd, filename):
    vis = open3d.visualization.Visualizer()
    vis.create_window(window_name='Open3D')
    ctr = vis.get_view_control()
    param = open3d.io.read_pinhole_camera_parameters(filename)
    vis.add_geometry(pcd)
    vis.get_render_option().load_from_json('renderoption.json')
    ctr.convert_from_pinhole_camera_parameters(param)
    vis.run()
    vis.destroy_window()

def pick_points(file):
    vis = open3d.visualization.VisualizerWithVertexSelection()
    vis.create_window(window_name='Open3D')
    # vis.create_window(window_name='Open3D', width=1080, height=720, left=300, top=150, visible=True)
    ctr = vis.get_view_control()
    point_cloud = open3d.geometry.PointCloud()
    pcd = open3d.io.read_point_cloud(file)   #此处读取的pcd文件,也可读取其他格式的
    point_cloud.points = open3d.utility.Vector3dVector(np.asarray(pcd.points).reshape((-1, 3)))
    vis.add_geometry(point_cloud)
    vis.get_render_option().point_size = 1                              # 点云大小
    vis.get_render_option().background_color = np.asarray([0, 0, 0])    # 背景颜色
    # 激活窗口。此函数将阻止当前线程，直到窗口关闭。
    vis.run()  # 等待用户拾取点
    vis.destroy_window()
    return vis.get_picked_points()

def read_pcds(pcdFilesPath):
    filepath = glob.glob(pcdFilesPath + '/*.pcd')
    filepath.sort()
    return filepath

def main():
    
    pcdFilesPath = "/media/sky/files/slam_study/calibration/data/livox_camera/pcdFiles"
    pcdFiles = read_pcds(pcdFilesPath)
    
    corners_savefile = "/media/sky/files/slam_study/calibration/data/livox_camera/lidar_photo.txt"
    cornersN = []

    for file in pcdFiles:
        cornersN.append(pick_points(file))
        
    with(open(corners_savefile, "w", encoding="utf-8")) as f:
        for i,corners in enumerate(cornersN):
            f.write("lidar{0}".format(i+1))
            for i, point in enumerate(corners):
                f.write(str(1+1))
                f.write(str(point.coord[0]) + " " + str(point.coord[1]) + " " + str(point.coord[2]))
       
        
if __name__ == '__main__':
    main()