import io
import os
from os.path import exists, join
import base64
import numpy as np
import json
import cv2
import time
from io import BytesIO
# from util.scorpioUtil import *
import colorsys
import random
# from util import scorpioUtil
import open3d as o3d
import math
from multiprocessing import Process
import random
import math
import pandas as pd
from threading import Thread
# import scipy
# from scipy.spatial.transform import Rotation as R
from util.convertUtils import calNormToPolar, conCamToRobot3d, convRealRobot3d
#      til.convertUtils import calNormToPolar, conCamToRobot3d,convRealRobot3d
import copy


class PointEstimator():
    region = None
    regionJig = None
    flybyZone = [[370, 666], [-138, 366], [263, 420]]

    # flybyPlaceZone = [[250, 670], [-320, 200], [157, 400]]

    def __init__(self, robotStatus, algoStatus):
        # def __init__(self):
        super().__init__()
        self.robotStatus = robotStatus
        self.algoStatus = algoStatus
        self.pcd = None
        self.geom_added = False
        self.cp = None
        self.cpnormal = None
        self.affine = None
        self.intrinsics = None
        self.ph_intr = None
        self.norm_y_map = None
        self.norm_x_map = None
        self.centerROI = [0, 0]
        self.setAngleMap()
        self.prev_target = list()
        self.instance_config = dict()
        self.radius = 0
        print("PICKINGPOINT INIT")

    def reset(self, robotStatus, algoStatus):
        self.robotStatus = robotStatus
        self.algoStatus = algoStatus
        self.pcd = None
        self.geom_added = False
        self.affine = None
        self.intrinsics = None
        self.ph_intr = None
        self.norm_y_map = None
        self.norm_x_map = None
        self.centerROI = [0, 0]
        self.setAngleMap()
        self.prev_target = list()
        self.instance_config = dict()
        self.radius = 0
        print("PICKINGPOINT INIT")

    def setAngleMap(self):
        roi = self.algoStatus["roi"]
        self.region = roi
        self.centerROI = [(roi[0][0] + roi[1][0]) // 2, (roi[0][1] + roi[1][1]) // 2]
        # self.regionJig = self.algoStatus["jigroi"]
        print("SETANGLEMAP roi:", self.region)
        print("SETANGLEMAP jig roi:", self.regionJig)
        # h,w = 300,450
        h, w = roi[1][1] - roi[0][1], roi[1][0] - roi[0][0]
        marginH = int(h * 0.25)
        marginW = int(w * 0.25)
        self.norm_x_map = np.zeros((h, w))
        self.norm_y_map = np.zeros((h, w))

        self.norm_x_map[:, 0:marginW].fill(-1)
        self.norm_x_map[:, w-marginW:w].fill(1)

        self.norm_y_map[0:marginH, :].fill(-1)
        self.norm_y_map[h-marginH:h, :].fill(1)

    def pclViewer(self):
        vis = o3d.visualization.Visualizer()
        vis.create_window('PCD', width=1280, height=720)

        while True:
            if self.pcd is None:
                continue

            if self.geom_added == False:
                vis.add_geometry(self.pcd)
                self.geom_added = True

            vis.update_geometry(self.pcd)
            vis.poll_events()
            vis.update_renderer()

    def setAffine(self):
        affinePath = join(os.getcwd(), "module", "vision", "affine", "affine_abb_RS415_0519.npy")
        self.affine = np.load(affinePath)

    def setIntrinsics(self, resolution):
        self.intrinsics = self.getIntrinsic(resolution)
        self.ph_intr = o3d.camera.PinholeCameraIntrinsic(self.intrinsics["width"],
                                                         self.intrinsics["height"],
                                                         self.intrinsics["fx"],
                                                         self.intrinsics["fy"],
                                                         self.intrinsics["ppx"],
                                                         self.intrinsics["ppy"])

    """
    def makePCL(self, color, depth, mask):
        h, w, _ = color.shape
        mask = mask[0:h, 0:w]
        np.where(mask == True, 1, mask)
        m = np.ma.masked_equal(mask, True)
        arr = np.ma.array(m.filled(1) * depth, mask=(m.mask))
        depth = arr.data
        color_Image = o3d.geometry.Image(color)
        depth_Image = o3d.geometry.Image(depth)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_Image, depth_Image)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.ph_intr)
        pcd = pcd.voxel_down_sample(voxel_size=1 / 500)
        return pcd
    """

    def GetInstConfig(self):
        with open("config/instconfig.json", "r") as f:
            self.instance_config = json.load(f)
        return 0

    def estimatePoint(self, masks, boxes, scores, classes, num_instances):
        # set parameters

        self.GetInstConfig()

        self.radius = 10 / 1000
        cannymin = 100
        cannymax = 150
        cannykernel = 7
        epsilon = 0.1 / 1000
        radius = self.radius
        swell = radius / np.sqrt(2)
        n_swell_sample = 8
        swell_sample_near_k = 30
        depth_scale = 1 / 0.0001

        # set param END

        mode = self.algoStatus["status"]
        self.algoStatus["status"] = "HOLD"
        color = self.algoStatus["color"]
        gray_image = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
        h, w, _ = color.shape
        curpos = self.algoStatus["curpos"]

        depth = self.algoStatus["depth"]

        time_s = time.time()

        maskArea = []
        pickpoint = []
        # region = np.array(self.region)

        mask = np.zeros((h, w), dtype=np.uint16)
        mask_canny = np.zeros((h, w), dtype=np.uint16)
        mask_scalexy = np.zeros((h, w), dtype=np.uint16)

        #
        skip_mask = 0
        for i, m in enumerate(masks):
            bbox = boxes[i]
            centerbbox = [(bbox[0] + bbox[2]) // 2, (bbox[1] + bbox[3]) // 2]

            dist = math.sqrt((self.centerROI[0] - centerbbox[0]) ** 2 + (self.centerROI[1] - centerbbox[1]) ** 2)
            border = 5

            # if bbox[0] < (self.region[0][0] - border) or bbox[1] < (self.region[0][1] - border) \
            #         or bbox[2] > (self.region[1][0] + border) or bbox[3] > (self.region[1][1] + border) \
            #         or len(np.nonzero(m)[0]) < 10 \
            #         or (scores[i] < 0.947) or (num_instances == 1 and scores[i] < 0.99) \
            #         or (num_instances > 3 and scores[i] < 0.99) \
            #         or (num_instances < 3 and np.std(depth[m[0:h, 0:w] == True]) < 1.9):
            if len(np.nonzero(m)[0]) < 10 \
                    or (scores[i] < 0.947) or (num_instances == 1 and scores[i] < 0.94) \
                    or (num_instances > 3 and scores[i] < 0.93) \
                    or (num_instances < 3 and np.std(depth[m[0:h, 0:w] == True]) < 1.9):
                # print("not feasible instance", num_instances)
                # # print(bbox, self.region)
                # print("score :", scores[i])
                # print("depth diff :", np.std(depth[m[0:h, 0:w] == True]))
                # print("len(np.nonzero(m)[0]):", len(np.nonzero(m)[0]))
                # print("SKIP")
                skip_mask += 1
                # print("non-selectable np.std:", np.std(depth[m == True]))
                maskArea.append([i, 0, 9999, 9999])
            else:
                m = m[0:h, 0:w]
                mask |= (m == True)
                a = depth[m == True]
                ma = np.ma.masked_equal(a, 0, copy=False)
                maskArea.append([i, len(np.nonzero(m)[0]), dist, np.min(ma[ma > 0])])
                #
                # if self.instance_config['option_scale_xy'][classes[i]]:
                #     mask_scalexy[tuple(np.argwhere(m == True).T)] = 255

                if self.instance_config['option_surface_small_instance'][classes[i]]:
                    mask_canny |= (m==True)
                    obj_img = gray_image[bbox[1]:bbox[3], bbox[0]:bbox[2]]

                    size = 5
                    kernel_motion_blur = np.zeros((size, size))
                    kernel_motion_blur[int((size - 1) / 2), :] = np.ones(size)
                    kernel_motion_blur = kernel_motion_blur / size
                    obj_img = cv2.filter2D(obj_img, -1, kernel_motion_blur)

                    v = obj_img.mean()
                    x = 0.3
                    cannykernel = 5
                    kernel = np.ones((cannykernel, cannykernel), np.uint8)
                    obj_img = cv2.Canny(obj_img, v*(1.0-x), v*(1.0+x))
                    # canny_edge = cv2.dilate(cv2.Canny(obj_img, m*(1.0-x), m*(1.0+x)), kernel, iterations=1)
                    # canny_edge = cv2.erode(canny_edge, kernel, iterations=1)
                    # canny_edge = mask_canny * canny_edge

                    dk = 5
                    dkernel = np.ones((dk, dk), np.uint8)
                    ek = 3
                    ekernel = np.ones((ek, ek), np.uint8)
                    canny_edge = cv2.dilate(obj_img, dkernel, iterations=1)
                    canny_edge = cv2.erode(canny_edge, ekernel, iterations=1)
                    canny_edge = cv2.erode(canny_edge, ekernel, iterations=1)
                    canny_edge = cv2.dilate(canny_edge, dkernel, iterations=1)

                    mask_canny[bbox[1]:bbox[3], bbox[0]:bbox[2]] = canny_edge

                    #gray_image[bbox[1]:bbox[3], bbox[0]:bbox[2]] = canny_edge


                    # color = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
                    # color[tuple(np.argwhere(canny_edge == 255).T)] = [255, 0, 0]


                    #mask_canny[bbox[0]:bbox[2], bbox[1]:bbox[3]] = 1

        # # 마스크 사이즈가 큰 순서대로 정렬 (화면 상 가려진 영역이 적은 물체 우선)
        maskArea = np.array(maskArea, dtype=np.uint16)
        xx = maskArea[maskArea[:, 1].argsort()][-10:][::-1]  # 면적이 넓은 10개를 고르고
        xx = xx[xx[:, 3].argsort()][:6]  # 높은 순서대로 10개를 고르고
        xx = xx[xx[:, 1].argsort()]  # 한번더 면적 순으로 작은것부터 정렬
        searchOrder = xx[:, 0]
        print("searchorder",searchOrder)
        print(list([classes[x] for x in searchOrder]))
        # print(list([x for x in searchOrder]))
        # classes_sorted = classes[searchOrder]
        print("classes", classes)

        if np.max(maskArea[searchOrder, 1]) <= 0:
            return "INIT", None

        np.where(mask == True, 1, mask)
        m = np.ma.masked_equal(mask, True)
        arr = np.ma.array(m.filled(1) * depth, mask=(m.mask))
        depth = arr.data

        # gray_image = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
        #
        # kernel = np.ones((cannykernel, cannykernel), np.uint8)
        # canny_edge = cv2.dilate(cv2.Canny(gray_image, cannymin, cannymax), kernel, iterations=1)
        # canny_edge = cv2.erode(canny_edge, kernel, iterations=1)
        # canny_edge = mask_canny * canny_edge

        color = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
        #color[tuple(np.argwhere(canny_edge == 255).T)] = [255, 0, 0]
        color[tuple(np.argwhere(mask_canny == 255).T)] = [255, 0, 0]
        # color[tuple(np.argwhere(mask_scalexy == 255).T)] = [0, 255, 0]

        cv2.imwrite("log/canny.jpg", color)
        color_Image = o3d.geometry.Image(color)
        depth_Image = o3d.geometry.Image(depth)

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_Image, depth_Image, depth_scale=depth_scale, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.ph_intr)

        pcd = pcd.voxel_down_sample(voxel_size=1 / 973)
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=20),
                             fast_normal_computation=False)

        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        colors = np.asarray(pcd.colors)

        x = np.argwhere(normals[:, 2] > 0)
        normals[x] = normals[x] * -1.0

        # pcd.orient_normals_to_align_with_direction([0, 0, -1])

        pcd_tree = o3d.geometry.KDTreeFlann(pcd)

        # normal이 수직 단위벡터와 45도 이상 차이나면 dead point
        normals_dot = np.abs(np.dot(normals, [0, 0, -1]))
        normal_criteria = self.Angle2Cos(45)
        dead_points = np.zeros_like(normals_dot)
        angle_dead_idx = np.argwhere(normals_dot <= normal_criteria)
        dead_points[angle_dead_idx] = 1

        self.prev_target = self.prev_target[-6:]
        for i, prev_point in enumerate(self.prev_target[::-1]):
            [prev_target_n, prev_target_idx, _] = pcd_tree.search_radius_vector_3d(prev_point, radius)
            if prev_target_idx:
                prev_target_idx = np.asanyarray(prev_target_idx)[np.random.randint(0, prev_target_n, prev_target_n *int(1 - (i / 10)))]
                dead_points[prev_target_idx] = 1

        # tool reachable check: spatial roi 안에 normal의 projection이 들어가는지?

        depth_floor = self.region[2] / depth_scale

        # bin height 설정 나중에 다시 Check값 설정은 나중에 바꿀 것
        height_bin = 0.1
        depth_bin = depth_floor - height_bin - 0.2
        depth_epsilon = 0.01

        flybox_min = self.pc_from_2d2(self.region[0][0], self.region[0][1], depth_floor, self.intrinsics)
        flybox_max = self.pc_from_2d2(self.region[1][0], self.region[1][1], depth_floor, self.intrinsics)

        # bin box 만들기 (spatial ROI, depth는 floor to bin + epsilon마진)
        flybox_points = [[flybox_min[0], flybox_min[1], depth_floor],
                         [flybox_max[0], flybox_max[1], (depth_bin - depth_epsilon)]]
        flybox_points = o3d.utility.Vector3dVector(flybox_points)
        flybox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(flybox_points)

        # neg or pos 확인해보기
        projection = normals * np.reshape((depth_bin - points[:, 2]) / normals_dot, (-1, 1))
        points_proj = points - projection

        pcd_proj = copy.deepcopy(pcd)
        np.asarray(pcd_proj.points)[:] = points_proj

        flyable_idx = flybox.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(points_proj))

        flyable_points = np.zeros_like(dead_points)
        flyable_points[flyable_idx] = 1

        dead_points = np.logical_or(dead_points, np.logical_not(flyable_points))
        dead_points_idx = np.argwhere(dead_points == 1)

        # (신설)dead point hard: 좀 더 빡빡한 기준 적용
        dead_points_hard = np.zeros_like(dead_points)
        dead_points_hard[np.argwhere(colors[:, 0] != colors[:, 2])] = 1
        dead_points_hard_idx = np.argwhere(dead_points_hard == 1)
        not_dead_hard_idx = np.argwhere(dead_points_hard == 0)

        np.asarray(pcd.colors)[dead_points_idx] = [1, 0.5, 0]
        np.asarray(pcd.colors)[angle_dead_idx] = [1, 0.5, 0]
        np.asarray(pcd.colors)[dead_points_hard_idx] = [1, 0, 0]

        # np.asarray(pcd_proj.colors)[:] = [0, 0, 1]
        # o3d.visualization.draw_geometries([pcd, flybox])

        o3d.io.write_point_cloud(join(os.getcwd(), "log", "pcd", "original.pcd"), pcd)

        angle_cp_criteria = self.Angle2Cos(35)
        angle_cp_idx = np.argwhere(normals_dot >= angle_cp_criteria)

        angle_cp_idx = angle_cp_idx.reshape(-1, )
        cp_pickable_idx = np.unique(np.concatenate((flyable_idx, angle_cp_idx)))

        print(f"instance: {len(masks)}, search: {len(searchOrder)}, skipped: {skip_mask}, MODE: {mode}")

        search_total = 0
        search = 0

        df_pickable = pd.DataFrame()

        depth_temp = self.DepthFill(self.algoStatus["depth"])

        for i, ind in enumerate(searchOrder):
            roi = boxes[ind]
            margin = 0  # 0.25
            marginx = (int)(np.abs(roi[0] - roi[2]) * margin)
            marginy = (int)(np.abs(roi[1] - roi[3]) * margin)

            # print("roi:",roi)
            posmin_depth = depth_temp[(roi[1] + marginy), (roi[0] + marginx)] / depth_scale
            posmax_depth = depth_temp[(roi[3] - marginy), (roi[2] - marginx)] / depth_scale

            posmin = self.pc_from_2d2((roi[0] + marginx), (roi[1] + marginy), posmin_depth, self.intrinsics)
            posmax = self.pc_from_2d2((roi[2] - marginx), (roi[3] - marginy), posmax_depth, self.intrinsics)
            bounding_points = [[posmin[0], posmin[1], 1], [posmax[0], posmax[1], -1]]

            bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(bounding_points))

            # bounding points z 바닥 flooring
            option_scale_xy = self.instance_config["option_scale_xy"][classes[ind]]
            roi_bbox = self.GetRoiBox(pcd, bbox, margin_xy=0.05, margin_floor=0.1, scale_xy=option_scale_xy)
            roi_bbox_pc_idx = np.asarray(roi_bbox.get_point_indices_within_bounding_box(pcd.points))
            roi_bbox_pc_idx = np.intersect1d(roi_bbox_pc_idx, cp_pickable_idx)
            roi_bbox_pc_idx = np.intersect1d(roi_bbox_pc_idx, not_dead_hard_idx)

            depth_mean = np.mean(depth[roi[1]:roi[3],roi[0]:roi[2]]) / depth_scale
            if abs(depth_mean - depth_floor) < 0.01:
                continue

            num_roi_rand_idx = 50

            if len(roi_bbox_pc_idx) > (num_roi_rand_idx * 2):
                roi_bbox_pc_idx = roi_bbox_pc_idx[np.random.randint(0, len(roi_bbox_pc_idx), num_roi_rand_idx)]
            elif 10 < len(roi_bbox_pc_idx) <= (num_roi_rand_idx * 2):
                roi_bbox_pc_idx = roi_bbox_pc_idx[np.random.randint(0, len(roi_bbox_pc_idx), int(len(roi_bbox_pc_idx)/2))]
            else:
                continue

            roi_bbox_pc = o3d.geometry.PointCloud()
            roi_bbox_pc.points = o3d.utility.Vector3dVector(points[roi_bbox_pc_idx])
            roi_bbox_distance = np.asarray(roi_bbox_pc.compute_mahalanobis_distance())

            # roi_bbox_distance_crit = np.percentile(roi_bbox_distance, 90)

            object_type = self.instance_config["object_type"][classes[ind]]

            for idx_roi, idx in enumerate(roi_bbox_pc_idx):
                search_total += 1
                cp_point = points[idx]
                cp_normal = normals[idx]

                [n, np_idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[idx], radius)

                np_idx = np.asarray(np_idx)

                near_points = copy.deepcopy(points[np_idx])
                near_normals = copy.deepcopy(normals[np_idx])

                # OUT조건
                outcrit = list()
                outcrit.append(np.intersect1d(dead_points_idx, np_idx).shape[0] >= (n * (2 / 3)))
                outcrit.append(np.intersect1d(dead_points_hard_idx, np_idx).shape[0] >= (n * (1 / 30)))
                outcrit.append(n < 10)
                # outcrit.append(roi_bbox_distance[idx_roi] > roi_bbox_distance_crit)
                if np.any(outcrit):
                    continue

                # epsilon = 0.0001
                if np.min(near_points[:, 2]) == np.max(near_points[:, 2]):
                    near_points[0][2] += epsilon

                near_box = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(near_points))

                # nearbox vector와 centerpoint normal의 max. dot product
                # near_box_dot = np.max(np.abs(np.dot(near_box.R, cp_normal)))

                # nearbox의 장방형 비율
                near_box_edge, _ = self.GetBoxEdgeNorm(near_box)
                near_box_edge_ratio = 1 - np.abs(1 - (near_box_edge[1] / near_box_edge[0]))

                # nearbox의 높이
                near_box_height = near_box_edge[2]

                # nearbox 대표 edge 길이와 radius 길이 비율
                if object_type == 2 or object_type == 3:
                    near_box_radius_ratio = 1 - np.abs(1 - (near_box_edge[0] * near_box_edge_ratio /
                                                            (np.sqrt(np.power(radius, 2) - np.power(near_box_height, 2)) * 2)))
                else:
                    near_box_radius_ratio = 1 - np.abs(1 - (near_box_edge[0] / (radius * 2)))

                # swell surface sample similarity
                # swell = 5/1000
                pcd_np = o3d.geometry.PointCloud()
                pcd_np.points = o3d.utility.Vector3dVector(near_points)
                pcd_np_tree = o3d.geometry.KDTreeFlann(pcd_np)

                pcd_np_proj = o3d.geometry.PointCloud()
                pcd_np_proj.points = o3d.utility.Vector3dVector(near_points + near_normals * swell)
                pcd_np_proj_tree = o3d.geometry.KDTreeFlann(pcd_np_proj)

                rand_sample = np.random.randint(0, n, n_swell_sample)
                swell_similarity = 0

                for rand_idx in rand_sample:
                    [_, np_idx_1, _] = pcd_np_tree.search_knn_vector_3d(np.asarray(pcd_np.points)[rand_idx], swell_sample_near_k)
                    [_, np_idx_2, _] = pcd_np_proj_tree.search_knn_vector_3d(np.asarray(pcd_np_proj.points)[rand_idx], swell_sample_near_k)
                    swell_similarity += np.intersect1d(np_idx_1, np_idx_2).shape[0] / swell_sample_near_k

                swell_similarity_score = swell_similarity/n_swell_sample

                mahalanobis_distance = roi_bbox_distance[idx_roi]

                pick_point_z = cp_point[2]

                cp_normal_dot = np.abs(np.dot(cp_normal, [0, 0, -1]))

                pickpoint.append([idx, near_box_edge_ratio, near_box_radius_ratio, near_box_height,
                                  swell_similarity_score, mahalanobis_distance, ind, pick_point_z,
                                  near_points, cp_normal_dot])

                search += 1

            if len(pickpoint) < 10:
                md = 1
                nbh = 1
            else:
                md = 0.5
                nbh = 0.5

            _, df_sorted = self.GetSortedIdx(data=pickpoint, nbe=0.80, nbr=0.95, ss=0.90, nbh=nbh, md=md,
                                             object_type=object_type)
            if df_sorted.shape[0] <= 1:
                _, df_sorted = self.GetSortedIdx(data=pickpoint, nbe=0.75, nbr=0.85, ss=0.85, nbh=nbh*1.5, md=md*1.5,
                                                 object_type=object_type)

            if df_sorted.shape[0] == 0:
                pass
            else:
                df_pickable = pd.concat([df_pickable, df_sorted.iloc[:2, :]], axis=0, ignore_index=True, join='outer')

        # if len(pickpoint) <= 0:
        #     print("no instance -> ESTIMATE SUCTION TO INIT")
        #     return "INIT", None
        print(f"Point total: {search_total} skip: {search_total - search}")

        if df_pickable.shape[0] < 1:
            print('*** No pickable point exist -> return to INIT')
            # print("-> ESTIMATE SUCTION TO INIT")
            return "INIT", None

        print("# of pickable points:", df_pickable.shape[0])

        # instance당 하나의 pickable
        print(df_pickable['search_order_index'])
        print(df_pickable.value_counts(['search_order_index']))
        print(df_pickable.value_counts(['search_order_index']).shape[0])
        if df_pickable.value_counts(['search_order_index']).shape[0] > 1:
            df_pickable = df_pickable.drop_duplicates(['search_order_index'], keep='first')
            print("pickable")
            print(df_pickable)
            df_pickable = df_pickable.sort_values(by=['pick_point_z'], axis=0, ascending=True).reset_index(drop=True)
            print(df_pickable['search_order_index'])

        res_list = list()
        # self.prev_target = list()

        for i, cand in enumerate(np.asarray(df_pickable['idx'])[:2]):
            color = [0, 0, 0]
            color[i] = 1
            np.asarray(pcd.colors)[cand] = color

            camerapos = points[cand]
            self.prev_target.append(camerapos)
            camerapos = self.coordconvert_world_cam(camerapos[0], camerapos[1], camerapos[2], self.intrinsics)

            norm = normals[cand]
            norm = self.ReliefNormal(norm, angle=10)
            # norm = [(norm[0] * -1), (norm[1]* -1), norm[2]]
            norm = [norm[1], norm[0], norm[2]]
            # norm = [ 0,0,-1]

            print("camerapos:", int(camerapos[0]), int(camerapos[1]), int(camerapos[2]))
            candd = self.pc_from_2d2(camerapos[0], camerapos[1], camerapos[2], self.intrinsics)
            candd = candd.tolist()

            print("CANDD:", candd)

            candX0, candY0, candZ0 = self.cam2robot(curpos, [int(candd[1][0]), -int(candd[0][0]), int(candd[2][0])])
            # 544             263(720, 1280)            393
            # 548             280(720, 1280)            397

            # curpos: [455.7698059082031, 38.0, 461.99407958984375, 4.408522880083865e-09, 179.99990844726562,
            #          4.408522880083865e-09]
            # TARGET: [array([[541.],
            #                 [266.],
            #                 [400.00000596]]), [-0.50525643270433, 0.002339658394357185, -0.8629660846229665],
            #          0.985323713316466, 0.13372216502511058,
            #          [515.4537810528547, 83.91811195619749, 92.19789255164758, 180, 180, 180], 5.691184184639585e-07, 1]
            # Estimate
            # point
            # Time: 0.2753
            # RECOG
            # COMPLETE - ROBUTpreset_string = str(json.load(open('preset_density.json'))).replace("'", '\"')
            # STATUS: MOVE
            # SENDDATA: {'ORDER': 'MOVE2',
            #            'DEST_POS': [515.4537810528547, 83.91811195619749, 92.19789255164758, 180, 180, 180],
            #                        [495.770263671875, 83.00000000000004, 85.99395751953125
            #            'VERT_POS': [array([525.79075381]), array([-143.44959642]), array([873.20862233]), 45.0,
            #                         179.99918971528191, 0],
            #            'SECOND_POS': [515.4067233635863, 86.63978277743819, 97.22612460513307, 180, 180, 180],
            #            'ESCAPE_POS': [array([525.79075381]), array([-143.44959642]), array([873.20862233]), 45.0,
            #                           179.99918971528191, 0], 'TIMESTAMP': 'Mon Sep 21 16:41:04 2020'}
            #

            # ccpp = [402.770263671875, -52.99999999999996, 95.99395751953125, 180, 180, 180]
            # candd = self.pc_from_2d2(ccpp[0], ccpp[1], ccpp[2], self.intrinsics)

            # TARGET
            # NORMAL: [-0.1130347233511727, 0.0235390067033975, -0.993312169703131]
            # RECOG
            # COMPLETE - ROBUT
            # STATUS: MOVE
            # SENDDATA: {'ORDER': 'MOVE2',
            #            'DEST_POS': [525.562177002197, 81.00889164682029, 104.78515201382834, 101.76348233145217,
            #                         173.3698616569089, 180],
            #            'VERT_POS': [454.7675074288854, -148.7266689669507, 881.0585124512527, 45.0, 179.99918971528191,
            #                         0],
            #            'SECOND_POS': [525.562177002197, 81.00889164682029, 104.78515201382834, 101.76348233145217,
            #                           173.3698616569089, 180],
            #            'ESCAPE_POS': [454.7675074288854, -148.7266689669507, 881.0585124512527, 45.0,
            #                           179.99918971528191, 0], 'TIMESTAMP': 'Mon Sep 21 15:37:10 2020'}
            # WAIT
            # curpos: [455.7972717285156, 37.96531677246094, 462.00408935546875, 134.23696899414062, 179.99368286132812,
            #          134.23756408691406]
            # TARGET: [array([[541.],
            #                 [252.],
            #                 [395.99999785]]), [0.3259938180633651, -0.03522298040458685, -0.9447154980394296],
            #          0.9615343359905641, 0.15617412722484592,
            #          [492.38748809629783, 84.19812108662148, 100.06913178684879, -6.166770978746001, 160.85917124750114,
            #           0], 1.4825808700213491e-06, 2]
            # Estimate
            # point
            # Time: 0.4348
            # RECOG
            # COMPLETE - ROBUT
            # STATUS: MOVE
            # SENDDATA: {'ORDER': 'MOVE2',
            #            'DEST_POS': [492.38748809629783, 84.19812108662148, 100.06913178684879, -6.166770978746001,
            #                         160.85917124750114, 0],
            #            'VERT_POS': [array([510.50171838]), array([-143.1831381]), array([877.1857301]), 45.0,
            #                         179.99918971528191, 0],
            #            'SECOND_POS': [509.90613799635173, 102.72399488999156, 98.89027689212176, -90.71212303993707,
            #                           165.50667122271904, 180],
            #            'ESCAPE_POS': [array([510.50171838]), array([-143.1831381]), array([877.1857301]), 45.0,
            #                           179.99918971528191, 0], 'TIMESTAMP': 'Mon Sep 21 16:28:00 2020'}
            # WAIT
            # [498.770263671875, 81.00000000000004, 88.99395751953125, 180, 180, 180]

            polar = calNormToPolar(norm)
            ang_z1, ang_y, ang_z2 = polar
            # ang_z1, ang_y, ang_z2 = 180,180,180
            offset_amt = 10
            offset_pos = [offset_amt * norm[0] * -1, offset_amt * norm[1] * -1, offset_amt * norm[2] * -1, 0, 0, 0]
            # offset_pos = [offset_amt * norm[0] * 1, offset_amt * norm[1] * 1, offset_amt * norm[2] * -1, 0, 0, 0]
            print("cand:", candX0, candY0, candZ0)
            pose_target_on = self.trans([candX0, candY0, candZ0, ang_z1, ang_y, ang_z2], offset_pos, "DR_BASE")

            print("pose_target_on:", pose_target_on)

            # 비행금지구역 체크
            if mode == "RECOG":
                if not self.checkFlybyZone(pose_target_on[0:3]):
                    continue

            res_list.append([camerapos, norm, df_pickable['near_box_radius_ratio'][i],
                             df_pickable['swell_similarity_score'][i], pose_target_on,
                             df_pickable['near_box_height'][i], df_pickable['search_order_index'][i],
                             df_pickable['near_points'][i], df_pickable['cp_normal'][i],
                             classes[df_pickable['search_order_index'][i]]])

            # 0 xyd, 1 centerNorm, 2 nearbox radius ratio, 3 swell surface similarity score,
            # 4 pose_target_on 5 nearbox volume, 6 searchorder index, 7 near_points, 8 near_normals, 9 class

        print("pick points: ", len(res_list))
        target = res_list[0]
        secondTarget = res_list[0] if len(res_list) < 2 else res_list[1]
        print("curpos:", curpos)
        # print("TARGET:", target)

        o3d.io.write_point_cloud(join(os.getcwd(), "log", "pcd", "target.pcd"), pcd)

        print("Estimate point Time: %.4f" % (time.time() - time_s))

        if len(target) <= 0:
            print("ESTIMATE NO TARGET")
            return "RESET", None

        else:
            # print("TARGET NORMAL :", target[1])
            self.robotStatus["center"] = [target[0][0], target[0][1]]
            self.robotStatus["secondcenter"] = [secondTarget[0][0], secondTarget[0][1]]

            # ind = searchOrder[target[6]]
            # roi = boxes[ind]
            #
            # posmin_depth = self.algoStatus["depth"][(roi[1]), (roi[0])]/1000
            # posmax_depth = self.algoStatus["depth"][(roi[3]), (roi[2])]/1000
            #
            # posmin = self.pc_from_2d2((roi[0]), (roi[1]), posmin_depth, self.intrinsics)
            # posmax = self.pc_from_2d2((roi[2]), (roi[3]), posmax_depth, self.intrinsics)
            #
            # bounding_points = [[posmin[0], posmin[1], -1000],
            #                    [posmax[0], posmax[1], 1000]]
            #
            # target_bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(bounding_points))
            # target_bbox_pc_idx = np.asarray(target_bbox.get_point_indices_within_bounding_box(pcd.points))
            #
            # target_pcd = o3d.geometry.PointCloud()
            # print("target_bbox_pc_idx:", target_bbox_pc_idx)
            # target_pcd.points = o3d.utility.Vector3dVector(copy.deepcopy(points[target_bbox_pc_idx]))
            # target_pcd.normals = o3d.utility.Vector3dVector(copy.deepcopy(normals[target_bbox_pc_idx]))
            #
            # target_pcd_tree = o3d.geometry.KDTreeFlann(target_pcd)
            # [k, nn, _] = target_pcd_tree.search_hybrid_vector_3d(target[0], 0.01, 1)
            #
            # np.asarray(target_pcd.colors)[nn] = [0, 1, 0]
            # np.asarray(target_pcd.colors)[nn[0]] = [1, 0, 0]
            #
            # o3d.io.write_point_cloud(join(os.getcwd(), "log", "pcd", "target.pcd"), target_pcd)

            return "MOVE", [target, self.getDeginatedPosition(target[0], [0, 0, -1]), secondTarget]

    def checkFlybyZone(self, pose):
        if pose[0] < self.flybyZone[0][0] or pose[0] > self.flybyZone[0][1]:
            print("X exceed", pose[0])
            # self.robotStatus["status"] = "RESET"
            return False
        elif pose[1] < self.flybyZone[1][0] or pose[1] > self.flybyZone[1][1]:
            print("Y exceed", pose[1], self.flybyZone[1][0], self.flybyZone[1][1])
            # self.robotStatus["status"] = "RESET"
            return False
        elif pose[2] < self.flybyZone[2][0] or pose[2] > self.flybyZone[2][1]:
            print("Z exceed :", pose[2])
            # self.robotStatus["status"] = "RESET"
            return False

        return True

    def getIntrinsic(self, res):
        intrinsics = {}
        if res == (720, 1280):
            # intrinsics = {
            #     "width": 1280,
            #     "height": 720,
            #     "ppx": 640.2010498046875,
            #     "ppy": 366.087890625,
            #     "fx": 610.1175537109375,
            #     "fy": 609.68475341796875
            # }
            # D415 SETTING
            intrinsics = {
                "width": 1280,
                "height": 720,
                "ppx": 647.222,
                "ppy": 365.037,
                "fx": 922.15,
                "fy": 921.341,
                "model": 2,
                "coeffs": [0, 0, 0, 0, 0]
            }
        elif res == (1080, 1920):
            # WFOV UNBINNED
            # 915.17633056640625,
            # 0.0,
            # 0.0,
            # 0.0,
            # 914.527099609375,
            # 0.0,
            # 960.55157470703125,
            # 549.3818359375,
            # 1.0
            intrinsics = {
                "width": 1920,
                "height": 1080,
                "ppx": 960.55157470703125,
                "ppy": 549.3818359375,
                "fx": 915.17633056640625,
                "fy": 914.527099609375
            }
        elif res == (1536, 2048):
            intrinsics = {
                "width": 2048,
                "height": 1536,
                "ppx": 1024.6217041015625,
                "ppy": 778.0406494140625,
                "fx": 976.1881103515625,
                "fy": 975.49560546875
            }
        return intrinsics

    def pc_from_2d(self, x, y, depth, intrinsics):
        XYZ = np.zeros((3, 1))
        # print(intrincsics.ppx, intrincsics.ppy, intrincsics.fx, intrincsics.fy)
        # XYZ[0] = ((x - self.principle[0]) / self.focal[0] * depth / 1000)
        # XYZ[1] = ((y - self.principle[1]) / self.focal[1] * depth / 1000)
        XYZ[0] = ((x - intrinsics["ppx"]) / intrinsics["fx"] * depth / 1000)
        XYZ[1] = ((y - intrinsics["ppy"]) / intrinsics["fy"] * depth / 1000)
        XYZ[2] = depth / 1000
        return XYZ

    def pc_from_2d2(self, x, y, depth, intrinsics):

        XYZ = np.zeros((3, 1))
        # print(intrincsics.ppx, intrincsics.ppy, intrincsics.fx, intrincsics.fy)
        # XYZ[0] = ((x - self.principle[0]) / self.focal[0] * depth / 1000)
        # XYZ[1] = ((y - self.principle[1]) / self.focal[1] * depth / 1000)
        XYZ[0] = ((x - intrinsics["ppx"]) / intrinsics["fx"] * depth)
        XYZ[1] = ((y - intrinsics["ppy"]) / intrinsics["fy"] * depth)
        XYZ[2] = depth
        return XYZ

    def coordconvert_world_cam(self, x, y, depth, intrinsics):
        XYZ = np.zeros([3, 1])
        XYZ[0] = (x * intrinsics["fx"]) / depth + intrinsics["ppx"]
        XYZ[1] = (y * intrinsics["fy"]) / depth + intrinsics["ppy"]
        XYZ[2] = depth * 1000
        return XYZ

    def rs2_deproject_pixel_to_point(self, px, py, pz, intr):
        coeffs = [0, 0, 0, 0, 0]
        point = [0, 0, 0]

        fx = intr["fx"]
        fy = intr["fy"]

        ppx = intr["ppx"]
        ppy = intr["ppy"]

        x = (px - ppx) / fx
        y = (py - ppy) / fy
        r2 = x * x + y * y
        f = 1 + coeffs[0] * r2 + coeffs[1] * r2 * r2 + coeffs[4] * r2 * r2 * r2
        ux = x * f + 2 * coeffs[2] * x * y + coeffs[3] * (r2 + 2 * x * x)
        uy = y * f + 2 * coeffs[3] * x * y + coeffs[2] * (r2 + 2 * y * y)
        x = ux
        y = uy
        point[0] = pz * x
        point[1] = pz * y
        point[2] = pz
        print("POINT:", point)
        return point

    # def get_point2pixelpos(self, camera_intrinsics, real_pos):
    #     return rs.rs2_project_point_to_pixel(camera_intrinsics, [real_pos[0], real_pos[1], real_pos[2]])

    def campos_from_pc(self, x, y, z, intrinsics):
        XYZ = np.zeros((3, 1))
        XYZ[0] = int(x * intrinsics["fx"] / z + intrinsics["ppx"])
        XYZ[1] = int(y * intrinsics["fy"] / z + intrinsics["ppy"])
        XYZ[2] = int(1000 * z)
        return XYZ

    def Convert_Camera2Robot3d(self, CameraPosx, CameraPosy, CameraDepthZ, affine3dMatrix):
        Robot_Posx = affine3dMatrix[0][0] * CameraPosx + affine3dMatrix[0][1] * CameraPosy + \
                     affine3dMatrix[0][2] * CameraDepthZ + affine3dMatrix[0][3]

        Robot_Posy = affine3dMatrix[1][0] * CameraPosx + affine3dMatrix[1][1] * CameraPosy + \
                     affine3dMatrix[1][2] * CameraDepthZ + affine3dMatrix[1][3]

        Robot_Posz = affine3dMatrix[2][0] * CameraPosx + affine3dMatrix[2][1] * CameraPosy + \
                     affine3dMatrix[2][2] * CameraDepthZ + affine3dMatrix[2][3]

        Robot_Posz -= 70  # 50

        return Robot_Posx, Robot_Posy, Robot_Posz

    def Convert_Camera2Robot3d_t(self, CameraPosx, CameraPosy, CameraDepthZ, affine3dMatrix):
        theta = CameraDepthZ / 40
        CameraDepthZ_t = CameraDepthZ - (CameraDepthZ * theta) / 1000.0

        Robot_Posx = affine3dMatrix[0][0] * CameraPosx + affine3dMatrix[0][1] * CameraPosy + \
                     affine3dMatrix[0][2] * CameraDepthZ_t + affine3dMatrix[0][3]

        Robot_Posy = affine3dMatrix[1][0] * CameraPosx + affine3dMatrix[1][1] * CameraPosy + \
                     affine3dMatrix[1][2] * CameraDepthZ_t + affine3dMatrix[1][3]

        Robot_Posz = affine3dMatrix[2][0] * CameraPosx + affine3dMatrix[2][1] * CameraPosy + \
                     affine3dMatrix[2][2] * CameraDepthZ_t + affine3dMatrix[2][3]

        # Robot_Posz += 40
        Robot_Posz += 20

        return Robot_Posx, Robot_Posy, Robot_Posz

    def calculaion_nor_2_polar(self, nor_vec):
        # print(nor_vec)
        if (nor_vec[0] == 0):
            nor_vec[0] = 0.00001
        if (nor_vec[1] == 0):
            nor_vec[1] = 0.00001
        if (nor_vec[2] == 0):
            nor_vec[2] = 0.00001

        a = math.acos(nor_vec[0] / math.sqrt(nor_vec[0] * nor_vec[0] + nor_vec[1] * nor_vec[1]))
        b = math.acos(
            nor_vec[2] / math.sqrt(nor_vec[0] * nor_vec[0] + nor_vec[1] * nor_vec[1] + nor_vec[2] * nor_vec[2]))
        # y방향이 양수이면, x는 0~180을 가지면 된다.
        if (nor_vec[1] < 0):
            a = a * -1
        a = a * 180 / math.pi
        b = b * 180 / math.pi

        # if(a > 90):
        #     #b = b * -1
        #     a = a - 180

        # if b > 90:
        #     a = abs(a) - 180

        # return [a, b, 0]
        return [a, b, 0]

    def getDeginatedPosition(self, camPos, norm):
        normal_offset = 186  # 176
        tool_offset = 100
        sp = [0, 0, -normal_offset - tool_offset]

        candX0, candY0, candZ0 = self.Convert_Camera2Robot3d(camPos[0], camPos[1], camPos[2], self.affine)
        ang_z1, ang_y, ang_z2 = self.calculaion_nor_2_polar([norm[0], norm[1], norm[2]])

        Rz1 = ([[math.cos(math.radians(ang_z1)), -math.sin(math.radians(ang_z1)), 0],
                [math.sin(math.radians(ang_z1)), math.cos(math.radians(ang_z1)), 0],
                [0, 0, 1]])
        Ry = ([[math.cos(math.radians(ang_y)), 0, math.sin(math.radians(ang_y))],
               [0, 1, 0],
               [-math.sin(math.radians(ang_y)), 0, math.cos(math.radians(ang_y))]])

        Rz2 = ([[math.cos(math.radians(ang_z2)), -math.sin(math.radians(ang_z2)), 0],
                [math.sin(math.radians(ang_z2)), math.cos(math.radians(ang_z2)), 0],
                [0, 0, 1]])
        # 90도 회전 추가
        angle_rot = 90
        Rz3 = ([[math.cos(math.radians(angle_rot)), -math.sin(math.radians(angle_rot)), 0],
                [math.sin(math.radians(angle_rot)), math.cos(math.radians(angle_rot)), 0],
                [0, 0, 1]])
        R_z1_y = np.matmul(Rz1, Ry)
        R = np.matmul(R_z1_y, Rz2)
        R = np.matmul(R, Rz3)
        s = np.matmul(R, sp)

        # from scipy.spatial.transform import Rotation as RR
        # sciR = RR.from_euler('zyz',[ang_z1,ang_y,ang_z2],degrees=True)
        # # print("R :", R)
        # # print("sciR :", sciR.as_matrix())
        # ss = np.matmul(sciR.as_matrix(), sp)
        # print("DOTTTT :", np.dot(sciR.as_matrix(),sp))
        # print("new:",s)
        return [candX0 + s[0], candY0 + s[1], candZ0 - tool_offset + s[2], ang_z1, ang_y, ang_z2]

    def trans(self, base, offset, mode):
        # print("OFFSET:",offset)
        if mode == "DR_TOOL":
            offset = [- offset[0], offset[1], - offset[2], offset[3], offset[4], offset[5]]
        # print("OFFSET:", offset)
        # print("TRANS:",[ base[i] + offset[i] for i in range(len(base)) ])
        return [base[i] + offset[i] for i in range(len(base))]

    def cam2robot(self, curpos, pos):
        print("CURPOS:", curpos)
        refpos = pos
        # tcp2camera = [-75, 35, 32]
        tcp2camera = [0, 0, 0]
        tcp2tool = [0, 0, 280]
        grip_rotation = 0.0
        offset = [refpos[i] + tcp2camera[i] if i < 3 else 0 for i in range(6)]
        refpos = [refpos[i] if i < 3 else 0 for i in range(6)]
        tcp2camera = [tcp2camera[i] if i < 3 else 0 for i in range(6)]
        curpos = self.trans(curpos, refpos, mode="DR_TOOL")
        return self.trans(curpos, tcp2camera, mode="DR_TOOL")[0:3]
        # return self.trans(curpos,offset, mode="DR_TOOL")[0:3]
        #
        # camera2obj = [refpos[0], refpos[1], 0, 0, 0, 0]
        # temp_pos = self.trans(curpos, camera2obj, "DR_TOOL")
        # tool_offset = [tcp2camera[0], tcp2camera[1], 0, 0, 0, 0]
        # temp_pos = self.trans(temp_pos, tool_offset, "DR_TOOL")
        # tool_offset = [0, 0, refpos[2] + tcp2camera[2] + 20, 0, 0, 0]
        # #temp_pos2 = self.trans(temp_pos, tool_offset, "DR_TOOL")
        # return self.trans(temp_pos, tool_offset, "DR_TOOL")[0:3]

    def movePos2(self, curpos, pos):
        # self.pos = []
        # refpos = recv_msg["DEST_POS"]
        refpos = pos
        # tcp2camera = [ -58,-39,170 ]
        tcp2camera = [-75, 35, 30]
        tcp2tool = [0, 0, 280]
        grip_rotation = 0.0
        # curr_pos = get_current_posx()[0]
        print('curr_pos ' + str(curpos))
        # curr_pos[297.404, -19.180, 476.727, 176.310, 179.960, 176.090]
        print('refpos' + str(refpos))
        # refpos[-35.98163633526756, 71.31509095551931, 430.0, 0.1085040041688638, 0.5701682322140159, -0.8143310555623355, 79.22534801745716, 144.52127178293222, 0]
        camera2obj = [refpos[0], refpos[1], 0, 0, 0, 0]
        print('camera2obj ' + str(camera2obj))
        # camera2obj[-35.98163633526756, 71.31509095551931, 0, 0, 0, 0]
        temp_pos = self.trans(curpos, camera2obj, "DR_TOOL")
        print('temp_pos ' + str(temp_pos))
        # temp_pos[333.111, 52.272, 476.706, 176.310, 179.960, 176.090]

        tool_offset = [tcp2camera[0], tcp2camera[1], 0, 0, 0, 0]
        print('tool_camera_offset ' + str(tool_offset))
        # tool_camera_offset[-75, 35, 0, 0, 0, 0]

        temp_pos = self.trans(temp_pos, tool_offset, "DR_TOOL")
        print('temp_pos ' + str(temp_pos))
        # temp_pos[407.976, 87.560, 476.655, 176.310, 179.960, 176.090]
        tool_offset = tcp2tool
        tool_offset = [tool_offset[0], tool_offset[1], 0, 0, 0, 0]
        print('tool_tool_offset ' + str(tool_offset))
        # tool_tool_offset[0, 0, 0, 0, 0, 0]
        temp_pos = self.trans(temp_pos, tool_offset, "DR_TOOL")
        print('temp_pos ' + str(temp_pos))
        # temp_pos[407.976, 87.560, 476.655, 176.310, 179.960, 176.090]

        # seg11 = posb(DR_LINE, temp_pos, radius=40)
        # 대상 물체의 좌표
        tool_offset = [0, 0, refpos[2] + tcp2camera[2] + 20, 0, 0, 0]
        print('tool_offset ' + str(tool_offset))
        # tool_offset[0, 0, 480.0, 0, 0, 0]
        temp_pos2 = self.trans(temp_pos, tool_offset, "DR_TOOL")
        print('temp_pos2' + str(temp_pos2))
        # temp_pos2[407.642, 87.582, -3.345, 176.310, 179.960, 176.090]

        # 대상 물체의 좌표에서 벡터 적용
        # zyz = recv_msg['RXYZ']
        zyz = [refpos[6], refpos[7], refpos[8]]
        print('zyz ' + str(zyz))
        # zyz[79.22534801745716, 144.52127178293222, 0]
        nor_vec = [refpos[3], refpos[4], refpos[5]]
        # temp_pos5 = posx([temp_pos2[0], temp_pos2[1], temp_pos2[2], zyz[0], zyz[1], zyz[2]])
        temp_pos5 = [temp_pos2[0], temp_pos2[1], temp_pos2[2], zyz[0], zyz[1], zyz[2]]
        print('temp_pos5 ' + str(temp_pos5))
        # temp_pos5[407.642, 87.582, -3.345, 79.225, 144.521, 0.000]

        # 툴크기를 감안한 로봇 위치 계산.
        temp_pos5 = self.trans(temp_pos5, [0, 0, 0, 0, 0, grip_rotation - 90], "DR_TOOL")
        print('temp_pos5' + str(temp_pos5))
        # temp_pos5[407.642, 87.582, -3.345, 79.225, 144.521, -90.000]
        obj2contect = [tcp2tool[0] * -1, tcp2tool[1] * -1, tcp2tool[2] * -1]

        offset_pos = [obj2contect[2] * nor_vec[0], obj2contect[2] * nor_vec[1], obj2contect[2] * nor_vec[2], 0, 0,
                      0]
        print('offset_pos ' + str(offset_pos))
        # offset_pos[-30.381121167281865, -159.64710501992445, 228.01269555745395, 0, 0, 0]
        contect_pos = self.trans(temp_pos5, offset_pos, "DR_BASE")
        print('contect_pos ' + str(contect_pos))
        # contect_pos[377.261, -72.066, 224.668, 79.225, 144.521, -90.000]

        # contact시작 포인트의 위치 계산.
        # obj2precontect = self.m_config.mod_path["OBJ2PRECONTACTPOS"][2] * -1
        obj2precontect = -30

        print('nor_vec' + str(nor_vec))
        # nor_vec[0.1085040041688638, 0.5701682322140159, -0.8143310555623355]
        offset_pos = [obj2precontect * nor_vec[0], obj2precontect * nor_vec[1], obj2precontect * nor_vec[2], 0, 0,
                      0]
        precontect_pos = self.trans(contect_pos, offset_pos, "DR_BASE")
        preprecontect_pos = self.trans(precontect_pos, offset_pos, "DR_BASE")
        ####
        # preprecontect_pos = [370.751, -106.276, 273.528, 79.225, 144.521, -90.000]
        ####
        print('curpos(in robot_cood)' + str(curpos))
        print('precontact_pos(in robot_cood)' + str(precontect_pos))
        print('contact_pos(in robot_cood)' + str(contect_pos))
        print('target_pos(in robot_cood)' + str(temp_pos5))
        # curpos( in robot_cood)[297.404, -19.180, 476.727, 176.310, 179.960, 176.090]
        # precontact_pos( in robot_cood)[374.006, -89.171, 249.098, 79.225, 144.521, -90.000]
        # contact_pos( in robot_cood)[377.261, -72.066, 224.668, 79.225, 144.521, -90.000]
        # target_pos( in robot_cood)[407.642, 87.582, -3.345, 79.225, 144.521, -90.000]

        # self.target_dict = {'READY':True,'POS':[preprecontect_pos, contect_pos], 'GRIP_SIZE':recv_msg['GRIP_SIZE']}
        # self.state_command = True
        print(str(preprecontect_pos))
        self.pos = preprecontect_pos
        self.state_move = True

    def movePos2_temp(self, curpos, pos):
        # self.pos = []
        # refpos = recv_msg["DEST_POS"]
        refpos = pos
        # tcp2camera = [ -58,-39,170 ]
        tcp2camera = [-75, 35, 30]
        tcp2tool = [0, 0, 280]
        grip_rotation = 0.0
        # curr_pos = get_current_posx()[0]
        print('curr_pos ' + str(curpos))
        # curr_pos[297.404, -19.180, 476.727, 176.310, 179.960, 176.090]
        print('refpos' + str(refpos))
        # refpos[-35.98163633526756, 71.31509095551931, 430.0, 0.1085040041688638, 0.5701682322140159, -0.8143310555623355, 79.22534801745716, 144.52127178293222, 0]
        camera2obj = [refpos[0], refpos[1], 0, 0, 0, 0]
        print('camera2obj ' + str(camera2obj))
        # camera2obj[-35.98163633526756, 71.31509095551931, 0, 0, 0, 0]
        temp_pos = self.trans(curpos, camera2obj, "DR_TOOL")
        print('temp_pos ' + str(temp_pos))
        # temp_pos[333.111, 52.272, 476.706, 176.310, 179.960, 176.090]

        tool_offset = tcp2camera
        tool_offset = [tool_offset[0], tool_offset[1], 0, 0, 0, 0]
        print('tool_camera_offset ' + str(tool_offset))
        # tool_camera_offset[-75, 35, 0, 0, 0, 0]

        temp_pos = self.trans(temp_pos, tool_offset, "DR_TOOL")
        print('temp_pos ' + str(temp_pos))
        # temp_pos[407.976, 87.560, 476.655, 176.310, 179.960, 176.090]
        tool_offset = tcp2tool
        tool_offset = [tool_offset[0], tool_offset[1], 0, 0, 0, 0]
        print('tool_tool_offset ' + str(tool_offset))
        # tool_tool_offset[0, 0, 0, 0, 0, 0]
        temp_pos = self.trans(temp_pos, tool_offset, "DR_TOOL")
        print('temp_pos ' + str(temp_pos))
        # temp_pos[407.976, 87.560, 476.655, 176.310, 179.960, 176.090]

        # seg11 = posb(DR_LINE, temp_pos, radius=40)
        # 대상 물체의 좌표
        tool_offset = [0, 0, refpos[2] + tcp2camera[2] + 20, 0, 0, 0]
        print('tool_offset ' + str(tool_offset))
        # tool_offset[0, 0, 480.0, 0, 0, 0]
        temp_pos2 = self.trans(temp_pos, tool_offset, "DR_TOOL")
        print('temp_pos2' + str(temp_pos2))
        # temp_pos2[407.642, 87.582, -3.345, 176.310, 179.960, 176.090]

        # 대상 물체의 좌표에서 벡터 적용
        # zyz = recv_msg['RXYZ']
        zyz = [refpos[6], refpos[7], refpos[8]]
        print('zyz ' + str(zyz))
        # zyz[79.22534801745716, 144.52127178293222, 0]
        nor_vec = [refpos[3], refpos[4], refpos[5]]
        # temp_pos5 = posx([temp_pos2[0], temp_pos2[1], temp_pos2[2], zyz[0], zyz[1], zyz[2]])
        temp_pos5 = [temp_pos2[0], temp_pos2[1], temp_pos2[2], zyz[0], zyz[1], zyz[2]]
        print('temp_pos5 ' + str(temp_pos5))
        # temp_pos5[407.642, 87.582, -3.345, 79.225, 144.521, 0.000]

        # 툴크기를 감안한 로봇 위치 계산.
        temp_pos5 = self.trans(temp_pos5, [0, 0, 0, 0, 0, grip_rotation - 90], "DR_TOOL")
        print('temp_pos5' + str(temp_pos5))
        # temp_pos5[407.642, 87.582, -3.345, 79.225, 144.521, -90.000]
        obj2contect = [tcp2tool[0] * -1, tcp2tool[1] * -1, tcp2tool[2] * -1]

        offset_pos = [obj2contect[2] * nor_vec[0], obj2contect[2] * nor_vec[1], obj2contect[2] * nor_vec[2], 0, 0, 0]
        print('offset_pos ' + str(offset_pos))
        # offset_pos[-30.381121167281865, -159.64710501992445, 228.01269555745395, 0, 0, 0]
        contect_pos = self.trans(temp_pos5, offset_pos, "DR_BASE")
        print('contect_pos ' + str(contect_pos))
        # contect_pos[377.261, -72.066, 224.668, 79.225, 144.521, -90.000]

        # contact시작 포인트의 위치 계산.
        # obj2precontect = self.m_config.mod_path["OBJ2PRECONTACTPOS"][2] * -1
        obj2precontect = -30

        print('nor_vec' + str(nor_vec))
        # nor_vec[0.1085040041688638, 0.5701682322140159, -0.8143310555623355]
        offset_pos = [obj2precontect * nor_vec[0], obj2precontect * nor_vec[1], obj2precontect * nor_vec[2], 0, 0, 0]
        precontect_pos = self.trans(contect_pos, offset_pos, "DR_BASE")
        preprecontect_pos = self.trans(precontect_pos, offset_pos, "DR_BASE")
        ####
        # preprecontect_pos = [370.751, -106.276, 273.528, 79.225, 144.521, -90.000]
        ####
        print('curpos(in robot_cood)' + str(curpos))
        print('precontact_pos(in robot_cood)' + str(precontect_pos))
        print('contact_pos(in robot_cood)' + str(contect_pos))
        print('target_pos(in robot_cood)' + str(temp_pos5))
        # curpos( in robot_cood)[297.404, -19.180, 476.727, 176.310, 179.960, 176.090]
        # precontact_pos( in robot_cood)[374.006, -89.171, 249.098, 79.225, 144.521, -90.000]
        # contact_pos( in robot_cood)[377.261, -72.066, 224.668, 79.225, 144.521, -90.000]
        # target_pos( in robot_cood)[407.642, 87.582, -3.345, 79.225, 144.521, -90.000]

        # self.target_dict = {'READY':True,'POS':[preprecontect_pos, contect_pos], 'GRIP_SIZE':recv_msg['GRIP_SIZE']}
        # self.state_command = True
        print(str(preprecontect_pos))
        self.pos = preprecontect_pos
        self.state_move = True

    def Draw_line(self, vector, mid_points, factor=0.005):
        vector = vector / np.linalg.norm(vector, ord=2) * factor
        endpoint = [0, 0, 0]
        startpoint = vector

        points = [
            endpoint,
            startpoint,
        ]
        lines = [
            [0, 1],
        ]
        colors = [[0, 0, 0] for i in range(len(lines))]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(points),
            lines=o3d.utility.Vector2iVector(lines),

        )
        line_set.translate(mid_points)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        # o3d.visualization.draw_geometries([line_set])  # ,
        return line_set


    def GetBoxEdgeNorm(self, box):
        """
        GetBoxEdgeNorm
        * PCD상 지정된 Box의 주된 Edge 길이 반환
        * 입력
            box: o3d.geometry.AxisAlignedBoundingBox | o3d.geometry.OrientedBoundingBox
        * 출력
            주축 edge 길이 3개 list
        """
        points = np.asarray(box.get_box_points())
        res = np.asarray([np.linalg.norm(points[1] - points[0]), np.linalg.norm(points[2] - points[0]),
                          np.linalg.norm(points[3] - points[0])])
        sort = np.argsort(res)[::-1]
        return res[sort], np.asarray([points[[0, 1]], points[[0, 2]], points[[0, 3]]])[sort]

    def Angle2Cos(self, angle):
        return np.cos((angle / 180) * np.pi)

    def GetSortedIdx(self, data, nbe: float = 0.985, nbr: float = 0.985,
                     ss: float = 0.95, nbh: float = 0.5, md: float = 0.5, object_type: int = 0):
        '''GetSortedIndex
        * 지정된 Criteria에 따라 Filtered & Sorted Index Array 반환
        ['idx', 'near_box_edge_ratio', 'near_box_radius_ratio', 'near_box_volume',
         'swell_similarity_score', 'mahalanobis_distance', 'search_order_index', 'pick_point_z',
         'near_points', 'near_normals']
        * 입력
            data: candidates 정보
            nbe: nearbox가 정방형인 정도, max 1, default 0.985
            nbr: nearbox와 search radius의 비율, max 1, default 0.985
            nbv: nearbox의 부피 남기는 비율, sort 후 일정비율만큼만 남겨 자르고 normal parallel ratio로 다시 정렬, default 0.15
            ss: 정해진 두께만큼 sweel 시켰을 때 표면 point 간 관계의 유사도
            md: instance에서의 mahalanobis distance
        * 출력
            sort된 point의 cloud상 index, nparray
            sort된 raw DataFrame
        '''

        columns = ['idx', 'near_box_edge_ratio', 'near_box_radius_ratio', 'near_box_height',
                   'swell_similarity_score', 'mahalanobis_distance', 'search_order_index', 'pick_point_z',
                   'near_points', 'cp_normal']

        df = pd.DataFrame(data=data, columns=columns)

        if nbh >= 1:
            nbh = 1
        if md >= 1:
            md = 1

        if object_type == 1:
            nbe = nbe + (1 - nbe) / 2
            nbr = nbr + (1 - nbr) / 2
            ss = ss + (1 - ss) / 2
        is_nberatio = df['near_box_edge_ratio'] >= nbe
        is_nbrratio = df['near_box_radius_ratio'] >= nbr
        is_ssscore = df['swell_similarity_score'] >= ss
        df_sorted = df[is_nberatio & is_nbrratio & is_ssscore]

        # # mahalanobis distance로 cut 하고 nearbox volume으로 정렬
        # if df_sorted.shape[0] > 3:
        #     is_md = df_sorted['mahalanobis_distance'] < np.percentile(df_sorted['mahalanobis_distance'], md*100)
        #     df_sorted = df_sorted[is_md]
        #
        # df_sorted = df_sorted.sort_values(by=['near_box_volume'], axis=0, ascending=True).reset_index(drop=True)

        # nearbox volume로 cut 하고 mahalanobis distance로 cut
        if df_sorted.shape[0] > 3:
            is_nbh = df_sorted['near_box_height'] < np.percentile(df_sorted['near_box_height'], nbh * 100)
            df_sorted = df_sorted[is_nbh]

        if df_sorted.shape[0] > 3:
            is_md = df_sorted['mahalanobis_distance'] < np.percentile(df_sorted['mahalanobis_distance'], md * 100)
            df_sorted = df_sorted[is_md]

        # cp_normal 순으로 정렬
        df_sorted = df_sorted.sort_values(by=['cp_normal'], axis=0, ascending=True).reset_index(drop=True)
        idx_sorted = np.asarray(df_sorted['idx'])

        return idx_sorted, df_sorted

    def GetRoiBox(self, pcd, bbox, margin_xy: float = 0, margin_floor: float = 0.2, scale_xy: float = 0):
        """GetRoiBox
        * 입력된 bounding box에 xy margin 및 floor margin 적용해서 return
        """
        assert(0 <= margin_xy < 0.5)
        assert(0 <= margin_floor < 1)
        assert(0 <= scale_xy <= 1)

        colors = np.asarray(pcd.colors)

        pcd_points_idx = np.asarray(bbox.get_point_indices_within_bounding_box(pcd.points))
        if len(pcd_points_idx) < 3:
            return bbox

        points = np.asarray(pcd.points)[pcd_points_idx]
        # if scale_xy:
        #     points = np.asarray(pcd.points)[np.intersect1d(pcd_points_idx, np.argwhere(colors[:, 0] != colors[:, 1]))]
        bbox = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(points))
        bbox_edge_norms, bbox_points = self.GetBoxEdgeNorm(bbox)

        # 길이순 edge 0-1-2
        # cutting은 1-0-2 순
        new_bbox_points = np.asarray([bbox_points[0, 0], bbox_points[0, 1], bbox_points[1, 1], bbox_points[2, 1]])
        new_bbox_directions = [new_bbox_points[1] - new_bbox_points[0], new_bbox_points[2] - new_bbox_points[0],
                               new_bbox_points[3] - new_bbox_points[0]]

        # cut edge1
        bbox_edge1_margin = margin_xy * new_bbox_directions[1]
        new_bbox_points[0] = new_bbox_points[0] + bbox_edge1_margin
        new_bbox_points[2] = new_bbox_points[2] - bbox_edge1_margin

        new_bbox_points[1] = new_bbox_points[1] + bbox_edge1_margin
        new_bbox_points[3] = new_bbox_points[3] + bbox_edge1_margin


        # cut edge0
        bbox_edge_scale_margin = (1 - bbox_edge_norms[1]/bbox_edge_norms[0])/2 * scale_xy
        bbox_edge0_margin = (bbox_edge_scale_margin + margin_xy) * new_bbox_directions[0]
        new_bbox_points[0] = new_bbox_points[0] + bbox_edge0_margin
        new_bbox_points[1] = new_bbox_points[1] - bbox_edge0_margin

        new_bbox_points[2] = new_bbox_points[2] + bbox_edge0_margin
        new_bbox_points[3] = new_bbox_points[3] + bbox_edge0_margin

        # cut floor
        bbox_edge2_margin = margin_floor * new_bbox_directions[2]
        new_bbox_points[3] = new_bbox_points[3] - bbox_edge2_margin

        new_bbox = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(new_bbox_points))

        return new_bbox

    def ReliefNormal(self, normal, standard=np.array([0, 0, -1]), angle=15):
        """ReliefNormal
        * 너무 누워있는 노멀벡터를 완화
          입력 angle 보다 큰 노멀벡터를 standard에서 기준점에서 angle만큼 각을 줄여 반환, 방향은 동일
        """
        if angle:
            dot = np.dot(standard, normal)
            angle_relief = np.deg2rad(np.rad2deg(np.arccos(dot)) - angle)
            if angle_relief > 0:
                sinvec = normal - standard*dot
                normal_relief = (standard * np.cos(angle_relief)) + \
                                (sinvec / np.linalg.norm(sinvec) * np.sin(angle_relief))
                normal_relief = normal_relief / np.linalg.norm(normal_relief)
                #assert np.arccos(np.dot(standard, normal_relief)) == angle_relief
                return normal_relief
            else:
                return standard
        return normal

    def DepthFill(self, array):
        df = pd.DataFrame(np.asarray(array))
        df.replace(0, np.NaN, inplace=True)
        df.fillna(self.region[2], inplace=True)
        df0 = df.interpolate(axis = 0).to_numpy()
        df1 = df.interpolate(axis = 1).to_numpy()
        return (df0 + df1)/2