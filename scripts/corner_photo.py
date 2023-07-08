import sys
import os
import cv2
import numpy as np
import glob



corners = []

class Params(object):
    def __init__(self):
        # 左相机内参数
        self.cam_matrix_left = np.asarray([[1.01371452e+03, 0.00000000e+00, 9.46057581e+02],
                                           [0.00000000e+00, 1.01374887e+03,
                                               5.18433343e+02],
                                           [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        # 右相机内参数
        self.cam_matrix_right = np.asarray([[1.00961974e+03, 0.00000000e+00, 9.67193243e+02],
                                            [0.00000000e+00, 1.01001406e+03,
                                                4.93110992e+02],
                                            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        # 左右相机畸变系数:[k1, k2, p1, p2, k3]
        self.distortion_l = np.asarray([[-3.69051939e-01,  1.61718172e-01, -7.83087722e-05,  2.58040137e-04,
                                         -3.75137954e-02]])
        self.distortion_r = np.asarray([[-3.67265340e-01,  1.55719616e-01, -1.88885026e-04,  4.45146376e-04,
                                         -3.39251782e-02]])
        # 旋转矩阵

        self.R = np.asarray([[9.99947132e-01, -9.92989790e-03,  2.67023488e-03],
                             [9.93049946e-03,  9.99950669e-01, -2.12118829e-04],
                             [-2.66799684e-03,  2.38624381e-04,  9.99996412e-01]])
        # 平移矩阵
        self.T = np.asarray([[-9.16059047e+01],
                             [-7.31339482e-03],
                             [1.68750703e+00]])

        self.baseline = self.T[0]


def undistortion(image, camera_matrix, dist_coeff):
    # 消除畸变
    undistortion_image = cv2.undistort(image, camera_matrix, dist_coeff)

    return undistortion_image

# 获取畸变校正和立体校正的映射变换矩阵、重投影矩阵
# @param：config是一个类，存储着双目标定的参数:config = stereoconfig.stereoCamera()


def getRectifyTransform(height, width, config):
    # 读取内参和外参
    left_K = config.cam_matrix_left
    right_K = config.cam_matrix_right
    left_distortion = config.distortion_l
    right_distortion = config.distortion_r
    R = config.R
    T = config.T

    # 计算校正变换
    height = int(height)
    width = int(width)
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        left_K, left_distortion, right_K, right_distortion, (width, height), R, T, alpha=-1)

    map1x, map1y = cv2.initUndistortRectifyMap(
        left_K, left_distortion, R1, P1, (width, height), cv2.CV_16SC2)
    map2x, map2y = cv2.initUndistortRectifyMap(
        right_K, right_distortion, R2, P2, (width, height), cv2.CV_16SC2)
    # print(width, height)

    return map1x, map1y, map2x, map2y, Q


# 畸变校正和立体校正
def rectifyImage(image1, image2, map1x, map1y, map2x, map2y):
    rectifyed_img1 = cv2.remap(image1, map1x, map1y, cv2.INTER_LINEAR)
    rectifyed_img2 = cv2.remap(image2, map2x, map2y, cv2.INTER_LINEAR)

    return rectifyed_img1, rectifyed_img2


def read_images(cal_path):
    filepath = glob.glob(cal_path + '/*.jpg')
    filepath.sort()
    return filepath


# 鼠标回调函数
def onMouse(event, x, y, flags, param):
    global corners
    if event == cv2.EVENT_MBUTTONDOWN:
        corners.append(np.array([x, y]).reshape(-1,2))


def main():
    global corners
    
    imgPath = "/media/sky/files/slam_study/calibration/data/livox_camera/photo"
    imagesL = read_images(os.path.join(imgPath))
    corners_savdir = "/media/sky/files/slam_study/calibration/data/livox_camera/corner_photo.txt"
    # imgLSavePath = os.path.join(imgPath, "rectify/left")
    # if os.path.exists(imgLSavePath) is False:
    #     os.makedirs(imgLSavePath)
    cornersN = []

    height, width = 1080, 1920
    config = Params()    # 读取相机内参和外参
    
    # criteria:角点精准化迭代过程的终止条件(阈值)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for i in range(len(imagesL)):
        imgL = cv2.imread(imagesL[i])
        # 去畸变
        imgL = undistortion(imgL, config.cam_matrix_left, config.distortion_l)
        
        gray = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)   # 转灰度
        
        while True:

            cv2.namedWindow("undistortion after", cv2.WINDOW_NORMAL)
            cv2.setMouseCallback('undistortion after', onMouse)
            cv2.imshow("undistortion after", imgL)
            keyValue = cv2.waitKey(0) & 0xFF
            if keyValue == ord(' '):        # 空格键实现暂停与开始
                cv2.waitKey(0)
            elif keyValue == ord('n'):      # ‘q’键实现退出
                #np.asarray(corners, dtype=np.float32) 先转为浮点数，不然就会报错
                corners2 = cv2.cornerSubPix(gray, np.asarray(corners, dtype=np.float32), (11, 11), (-1, -1), criteria)
                cornersN.append(corners2)
                corners.clear()
                break
            elif keyValue == ord('q'):      # ‘q’键实现退出
                exit(0)
    
    with(open(corners_savdir, "w", encoding="utf-8")) as f:
        for i, corners in enumerate(cornersN):
            f.write("{0s}.jpg\n".format(i+1))
            for i,corner in enumerate(corners):
                f.write(str(i+1)+"\n")
                f.write(str(corner[0,0]) + " " + str(corner[0,1])+"\n")


if __name__ == '__main__':
    main()
