import io
import os
from os.path import exists, join
import base64
import numpy as np
import json
import cv2
import time
from io import BytesIO
import colorsys
import random
import open3d as o3d
import math
from multiprocessing import Process
import random
import math
import pandas as pd
from threading import Thread
from util.convertUtils import calNormToPolar
import copy
from datetime import datetime
import pickle

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):

            return int(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

class PointEstimator():
    region = None
    regionJig = None
    flybyZone = [[370, 666], [-138, 366], [263, 420]]

    def __init__(self, robotStatus, algoStatus,pclQ):
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
        self.instance_depth = list()
        self.pclQ = pclQ
        print("PICKINGPOINT INIT")

    def setAngleMap(self):
        roi = self.algoStatus["roi"]
        self.region = roi
        self.centerROI = np.array([(roi[0][0] + roi[1][0]) // 2, (roi[0][1] + roi[1][1]) // 2])
        print("SETANGLEMAP roi:", self.region)
        print("SETANGLEMAP jig roi:", self.regionJig)

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
        with open("config/instconfig_pipe.json", "r") as f:
            self.instance_config = json.load(f)
        return 0

    def estimatePoint(self, masks, boxes, scores, classes, num_instances):

        ddtt = datetime.now().strftime("%H%M%S")
        with open(join(os.getcwd(), "log", "mask"+ddtt+".pkl"), 'wb') as f:
            pickle.dump(json.dumps(masks,cls=NumpyEncoder), f, pickle.HIGHEST_PROTOCOL)
        with open(join(os.getcwd(), "log", "boxes"+ddtt+".pkl"), 'wb') as f:
            pickle.dump(json.dumps(boxes,cls=NumpyEncoder), f, pickle.HIGHEST_PROTOCOL)

        # set parameters
        self.GetInstConfig()

        # suction tooltip radius
        self.radius = 10 / 1000
        radius = self.radius

        # realsense depth scale
        depth_scale = 1 / 0.0001

        # boundingbox 생성 시 height 0을 피하기위한 epsilon
        epsilon = 0.1 / 1000

        # swell similarity score 계산을 위한 param
        swell = radius / np.sqrt(2)
        n_swell_sample = 12
        swell_sample_near_k = 30

        # 추후 normal cut 을 위한 기준
        normal_cut_criteria = 40
        normal_pick_criteria = 40

        # total 서치할 point 수
        total_search_points = 480

        # set param END

        mode = self.algoStatus["status"]
        self.algoStatus["status"] = "HOLD"
        color = np.array(self.algoStatus["color"])
        depth = np.array(self.algoStatus["depth"])
        gray_image = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)
        h, w, _ = color.shape
        curpos = self.algoStatus["curpos"]

        time_s = time.time()

        depth_instance = []
        mask_info = []
        class_info = []
        pick_point = []

        mask_canny = np.zeros((h, w), dtype=np.uint16)
        skip_mask = 0

        for i, m in enumerate(masks):
            bbox = boxes[i]
            instance_class = classes[i]

            if len(np.nonzero(m)[0]) < 10 \
                    or (scores[i] < 0.947) or (num_instances == 1 and scores[i] < 0.94) \
                    or (num_instances > 3 and scores[i] < 0.93) \
                    or (num_instances < 3 and np.std(depth[m[0:h, 0:w] == True]) < 1.9):
                skip_mask += 1

                mask_info.append([i, 0, 9999, 9999,  instance_class])
                depth_instance.append(o3d.geometry.Image(o3d.np.zeros([h, w]).astype("uint16")))

            else:
                m = m[0:h, 0:w]
                depth_temp = m * depth

                depth_1d = depth_temp.reshape(-1)
                depth_1d = depth_1d[depth_1d != 0]

                center_bbox = np.array([(bbox[0] + bbox[2]) // 2, (bbox[1] + bbox[3]) // 2])
                dist = np.linalg.norm(center_bbox - self.centerROI)
                mask_info.append([i, depth_1d.shape[0], dist, depth_1d.min(), instance_class])
                depth_instance.append(o3d.geometry.Image(depth_temp.astype("uint16")))

                # 표면의 양각음각 영역 처리
                if self.instance_config['option_surface_small_instance'][instance_class]:
                    mask_canny |= (m==True)
                    obj_img = gray_image[bbox[1]:bbox[3], bbox[0]:bbox[2]]

                    size = 5
                    kernel_motion_blur = np.zeros((size, size))
                    kernel_motion_blur[int((size - 1) / 2), :] = np.ones(size)
                    kernel_motion_blur = kernel_motion_blur / size
                    obj_img = cv2.filter2D(obj_img, -1, kernel_motion_blur)

                    v = obj_img.mean()
                    x = 0.3
                    obj_img = cv2.Canny(obj_img, v*(1.0-x), v*(1.0+x))

                    dk = 5
                    dkernel = np.ones((dk, dk), np.uint8)
                    ek = 3
                    ekernel = np.ones((ek, ek), np.uint8)

                    canny_edge = cv2.dilate(obj_img, dkernel, iterations=1)
                    canny_edge = cv2.erode(canny_edge, ekernel, iterations=1)
                    canny_edge = cv2.erode(canny_edge, ekernel, iterations=1)
                    canny_edge = cv2.dilate(canny_edge, dkernel, iterations=1)

                    mask_canny[bbox[1]:bbox[3], bbox[0]:bbox[2]] = canny_edge

        # # 마스크 사이즈가 큰 순서대로 정렬 (화면 상 가려진 영역이 적은 물체 우선)
        mask_info = np.array(mask_info, dtype=np.uint16)
        xx = mask_info[mask_info[:, 1].argsort()][-10:][::-1]  # 면적이 넓은 10개를 고르고
        xx = xx[xx[:, 3].argsort()][:6]  # 높은 순서대로 10개를 고르고
        xx = xx[xx[:, 1].argsort()]  # 한번더 면적 순으로 작은것부터 정렬
        search_order = xx[:, 0]
        class_order = xx[:, 4]

        if np.max(mask_info[search_order, 1]) <= 0:
            return "INIT", None

        # print(search_order)

        # canny mask 적용 (color가 grayscale이 아니면 canny dead)
        color = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
        color[tuple(np.argwhere(mask_canny != 0).T)] = [255, 0, 0]
        cv2.imwrite("log/canny.jpg", color)

        color_image = o3d.geometry.Image(color)
        pcd_instance = [0] * len(masks)
        pcd_total = o3d.geometry.PointCloud()

        search_order_remove = list()
        for _, i in enumerate(search_order):
            depth_image = depth_instance[i]
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, depth_scale=depth_scale, convert_rgb_to_intensity=False)
            pcd_temp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.ph_intr)
            if np.asarray(pcd_temp.points).shape[0] == 0:
                search_order_remove.append(_)
                continue

            pcd_temp.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=20),
                                      fast_normal_computation=False)
            pcd_instance[i] = pcd_temp
            pcd_total += pcd_temp

        search_order = np.delete(search_order, search_order_remove)
        class_order = np.delete(class_order, search_order_remove)

        print(f"instance: {len(masks)}, search: {len(search_order)}, skipped: {skip_mask}, MODE: {mode}")

        # bin height 설정 나중에 다시 Check /// height_bin: 추후 수정필요
        depth_floor = self.region[2] / depth_scale
        height_bin = 0.1
        depth_bin = depth_floor - height_bin - 0.1
        depth_epsilon = 0.01
        flybox_min = self.pc_from_2d2(self.region[0][0], self.region[0][1], depth_floor, self.intrinsics)
        flybox_max = self.pc_from_2d2(self.region[1][0], self.region[1][1], depth_floor, self.intrinsics)

        # bin box 만들기 (spatial ROI, depth는 floor to bin + epsilon마진)
        flybox_points = [[flybox_min[0], flybox_min[1], depth_floor],
                         [flybox_max[0], flybox_max[1], (depth_bin - depth_epsilon)]]
        flybox_points = o3d.utility.Vector3dVector(flybox_points)
        roi_spatial = o3d.geometry.AxisAlignedBoundingBox.create_from_points(flybox_points)

        # 전체 PCD 저장
        pcd_total_tree = o3d.geometry.KDTreeFlann(pcd_total)
        points_total = np.asarray(pcd_total.points)
        normals_total = np.asarray(pcd_total.normals)
        x = np.argwhere(normals_total[:, 2] > 0)
        normals_total[x] = normals_total[x] * -1.0
        now = datetime.now()

        o3d.io.write_point_cloud(join(os.getcwd(), "log", "pcd", "original"+now.strftime("%H%M%S")+".pcd"), pcd_total)

        num_roi_rand_idx = int(total_search_points / len(search_order))
        assert len(class_order) == len(search_order)

        search = 0
        df_pickable = pd.DataFrame()

        for i_order, i in enumerate(search_order):
            instance_class = class_order[i_order]
            pcd = pcd_instance[i]

            # voxel down / normal 정렬
            pcd = pcd.voxel_down_sample(voxel_size=1 / 973)
            pcd_tree = o3d.geometry.KDTreeFlann(pcd)
            points = np.asarray(pcd.points)
            normals = np.asarray(pcd.normals)
            colors = np.asarray(pcd.colors)

            x = np.argwhere(normals[:, 2] > 0)
            normals[x] = normals[x] * -1.0

            # dead point 지정
            # normal이 수직 단위벡터와 normal_cut_criteria 이상 차이나면 dead point
            normals_dot = np.abs(np.dot(normals, [0, 0, -1]))
            dead_points = np.zeros_like(normals_dot)
            angle_dead_idx = np.argwhere(normals_dot <= np.abs(self.Angle2Cos(normal_cut_criteria)))
            angle_pickable_idx = np.argwhere(normals_dot >= np.abs(self.Angle2Cos(normal_pick_criteria)))

            dead_points[angle_dead_idx] = 1

            plane_model, inliers = pcd.segment_plane(distance_threshold=0.0037, ransac_n=3, num_iterations=500)
            inlier_cloud = pcd.select_by_index(inliers)
            [t_n, t_idx, _] = pcd_tree.search_radius_vector_3d(inlier_cloud.get_center(), radius)

            # 이전 타겟 6개까지 dead point
            self.prev_target = self.prev_target[-6:]
            for i, prev_point in enumerate(self.prev_target[::-1]):
                [prev_target_n, prev_target_idx, _] = pcd_tree.search_radius_vector_3d(prev_point, radius)
                if prev_target_idx:
                    prev_target_idx = np.asarray(prev_target_idx)[
                        np.random.randint(0, prev_target_n, prev_target_n * int(1 - (i / 10)))]
                    dead_points[prev_target_idx] = 1

            # tool reachable(flydead) check: spatial roi 안에 normal의 projection이 들어가는지?
            projection = normals * np.reshape((depth_bin - points[:, 2]) / (normals_dot + epsilon), (-1, 1))
            points_proj = points - projection
            pcd_proj = copy.deepcopy(pcd)
            np.asarray(pcd_proj.points)[:] = points_proj

            flyable_idx = np.array(roi_spatial.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(points_proj))).reshape(-1, 1)
            flyable_idx = np.intersect1d(flyable_idx,t_idx)
            fly_dead_points = np.ones_like(dead_points)
            fly_dead_points[flyable_idx] = 0
            fly_dead_idx = np.argwhere(fly_dead_points == 1)
            #print(len(points_proj),len(flyable_idx),len(t_idx),len(np.intersect1d(flyable_idx,t_idx)))

            # canny dead points
            canny_dead_points = np.zeros_like(dead_points)
            canny_dead_points[np.argwhere(colors[:, 0] != colors[:, 1]).reshape(-1, )] = 1
            canny_dead_idx = np.argwhere(canny_dead_points == 1)
            not_canny_dead_idx = np.argwhere(canny_dead_points == 0)

            # pickable한 index 따로 확인
            pickable_idx = np.unique(np.intersect1d(np.intersect1d(flyable_idx, angle_pickable_idx), not_canny_dead_idx))

            # bounding points z 바닥 flooring 및 scaling
            roi_bbox = o3d.geometry.OrientedBoundingBox.create_from_points(pcd.points)
            # option_scale_xy = self.instance_config["option_scale_xy"][instance_class]
            # roi_bbox = self.GetRoiBox(pcd, bbox, margin_xy=0.05, margin_floor=0.1, scale_xy=option_scale_xy)
            roi_bbox_pc_idx = np.array(roi_bbox.get_point_indices_within_bounding_box(pcd.points))

            # pickable한 point만 남기고
            roi_bbox_pc_idx = np.array(np.intersect1d(roi_bbox_pc_idx, pickable_idx))

            # random sampling
            if len(roi_bbox_pc_idx) > (num_roi_rand_idx * 2):
                roi_bbox_pc_idx = roi_bbox_pc_idx[np.random.randint(0, len(roi_bbox_pc_idx), num_roi_rand_idx)]
            elif 10 < len(roi_bbox_pc_idx) <= (num_roi_rand_idx * 2):
                roi_bbox_pc_idx = roi_bbox_pc_idx[np.random.randint(0, len(roi_bbox_pc_idx), int(len(roi_bbox_pc_idx)/2))]
            else:
                continue

            # mahalanobis distance 계산
            roi_distance = np.asarray(pcd.compute_mahalanobis_distance())
            roi_distance_crit = np.percentile(roi_distance, 90)

            # dead points 정의
            dead_points_idx = angle_pickable_idx
            # hard dead points 정의
            dead_points_hard_idx = np.unique(np.concatenate((fly_dead_idx, canny_dead_idx)))

            object_type = self.instance_config["object_type"][instance_class]
            is_metallic = self.instance_config["option_metallic"][instance_class]

            # print(roi_bbox_pc_idx)
            # o3d.visualization.draw_geometries([pcd, roi_bbox])

            for idx_in_roi, idx in enumerate(roi_bbox_pc_idx):
                cp_point = points[idx]
                cp_normal = normals[idx]
                cp_normal_dot = normals_dot[idx]

                [n, np_idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[idx], radius)
                np_idx = np.asarray(np_idx).reshape(-1, )
                near_points = copy.deepcopy(points[np_idx])
                near_normals = copy.deepcopy(normals[np_idx])

                # OUT조건
                outcrit = list()
                outcrit.append(np.intersect1d(dead_points_idx, np_idx).shape[0] >= (n * (1 / 2)))
                outcrit.append(np.intersect1d(dead_points_hard_idx, np_idx).shape[0] >= (n * (1 / 4)))
                outcrit.append(n < 30)
                outcrit.append(roi_distance[idx] > roi_distance_crit)
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
                if object_type:
                    near_box_radius_ratio = 1 - np.abs(1 - (near_box_edge[0] * near_box_edge_ratio /
                                                            np.sqrt(np.power(radius,2) - np.power(near_box_height,2) * 2)))
                else:
                    near_box_radius_ratio = 1 - np.abs(1 - (near_box_edge[0] / (radius * 2)))

                # swell surface sample similarity
                # swell = 5/1000
                swell_similarity_score = 0
                if is_metallic:
                    [swell_similarity_score, _, _] = pcd_tree.search_radius_vector_3d(pcd.points[idx], radius * 3)
                else:
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

                mahalanobis_distance = roi_distance[idx_in_roi]

                cp_z = cp_point[2]

                pick_point.append([idx, near_box_edge_ratio, near_box_radius_ratio, near_box_height,
                                  swell_similarity_score, mahalanobis_distance, instance_class, cp_point, cp_normal,
                                  cp_normal_dot, cp_z, near_points])

                search += 1

            if len(pick_point) < 10:
                md = 1
                nbh = 1
            else:
                md = 0.5
                nbh = 0.5

            _, df_sorted = self.GetSortedIdx(data=pick_point, nbe=0.925, nbr=0.925, ss=0.85, nbh=nbh, md=md,
                                             object_type=object_type, is_metallic=is_metallic)
            if df_sorted.shape[0] <= 1:
                _, df_sorted = self.GetSortedIdx(data=pick_point, nbe=0.85, nbr=0.85, ss=0.75, nbh=nbh*1.5, md=md*1.5,
                                                 object_type=object_type, is_metallic=is_metallic)

            if df_sorted.shape[0] == 0:
                pass
            else:
                df_pickable = pd.concat([df_pickable, df_sorted.iloc[:2, :]], axis=0, ignore_index=True, join='outer')

        # if len(pick_point) <= 0:
        #     print("no instance -> ESTIMATE SUCTION TO INIT")
        #     return "INIT", None
        print(f"Point total: {total_search_points} skip: {total_search_points - search}")

        if df_pickable.shape[0] < 1:
            print('*** No pickable point exist -> return to INIT')
            # print("-> ESTIMATE SUCTION TO INIT")
            return "INIT", None

        print("# of pickable points:", df_pickable.shape[0])

        # instance당 하나의 pickable
        print("instance count",df_pickable.value_counts(['instance_class']).shape[0])
        print("len", len(df_pickable))
        if df_pickable.value_counts(['instance_class']).shape[0] > 1:
            df_pickable = df_pickable.drop_duplicates(['instance_class'], keep='first')
            df_pickable = df_pickable.sort_values(by=['point_z'], axis=0, ascending=True).reset_index(drop=True)
        df_pickable = df_pickable.sort_values(by=['point_z'], axis=0, ascending=True).reset_index(drop=True)
        res_list = list()

        for j in range(2):
            if j > df_pickable.shape[0] - 1:
                j = df_pickable.shape[0] - 1
            cand_xyz = np.array([df_pickable['cp_point'][j][0], df_pickable['cp_point'][j][1],
                                 df_pickable['cp_point'][j][2]])
            [_, cand, _] = pcd_total_tree.search_knn_vector_3d(cand_xyz, 1)
            cand = cand[0]

            color = [0, 0, 0]
            color[j] = 1
            np.asarray(pcd_total.colors)[cand] = color

            camerapos = points_total[cand]
            self.prev_target.append(camerapos)
            camerapos = self.coordconvert_world_cam(camerapos[0], camerapos[1], camerapos[2], self.intrinsics)

            norm = df_pickable['cp_normal'][j]
            norm = self.ReliefNormal(norm, angle=10)
            # norm = [(norm[0] * -1), (norm[1]* -1), norm[2]]
            norm = [norm[1], norm[0], norm[2]]
            # norm = [ 0,0,-1]

            #####print("camerapos:", int(camerapos[0]), int(camerapos[1]), int(camerapos[2]))
            candd = self.pc_from_2d2(camerapos[0], camerapos[1], camerapos[2], self.intrinsics)
            candd = candd.tolist()

            #####print("CANDD:", candd)

            candX0, candY0, candZ0 = self.cam2robot(curpos, [int(candd[1][0]), -int(candd[0][0]), int(candd[2][0])])

            polar = calNormToPolar(norm)
            ang_z1, ang_y, ang_z2 = polar

            offset_amt = 10
            offset_pos = [offset_amt * norm[0] * -1, offset_amt * norm[1] * -1, offset_amt * norm[2] * -1, 0, 0, 0]

            #####print("cand:", candX0, candY0, candZ0)
            pose_target_on = self.trans([candX0, candY0, candZ0, ang_z1, ang_y, ang_z2], offset_pos, "DR_BASE")

            #####print("pose_target_on:", pose_target_on)

            # 비행금지구역 체크
            if mode == "RECOG":
                if not self.checkFlybyZone(pose_target_on[0:3]):
                    continue

            res_list.append([camerapos, norm, df_pickable['near_box_radius_ratio'][j],
                             df_pickable['swell_similarity_score'][j], pose_target_on,
                             df_pickable['near_box_height'][j], df_pickable['point_z'][j],
                             df_pickable['near_points'][j], df_pickable['cp_normal'][j],
                             df_pickable['instance_class'][j],cand])

            # 0 xyd, 1 centerNorm, 2 nearbox radius ratio, 3 swell surface similarity score,
            # 4 pose_target_on 5 nearbox height, 6 point z, 7 near_points, 8 cp_normal, 9 class

        #####print("pick points: ", len(res_list))
        target = res_list[0]
        secondTarget = res_list[0] if len(res_list) < 2 else res_list[1]
        #####print("curpos:", curpos)
        # print("TARGET:", target)
        now = datetime.now()

        o3d.io.write_point_cloud(join(os.getcwd(), "log", "pcd", "target"+ now.strftime("%H%M%S") +".pcd"), pcd)

        #####print("Estimate point Time: %.4f" % (time.time() - time_s))

        if len(target) <= 0:
            print("ESTIMATE NO TARGET")
            return "RESET", None

        else:
            # print("TARGET NORMAL :", target[1])
            self.robotStatus["center"] = [target[0][0], target[0][1]]
            self.robotStatus["secondcenter"] = [secondTarget[0][0], secondTarget[0][1]]

            print("target[-1]:", target[-1])
            [aa, bb, _] = pcd_total_tree.search_radius_vector_3d(pcd_total.points[target[-1]], 0.02)
            if aa > 0:
                np.asarray(pcd_total.colors)[bb] = [1,0,0]
            o3d.io.write_point_cloud(join(os.getcwd(), "log", "pcd", "originawithpoint.pcd"), pcd_total)
            self.pclQ.put("GO")

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
        #####print("CURPOS:", curpos)
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
                     ss: float = 0.95, nbh: float = 0.5, md: float = 0.5,
                     object_type: int = 0, is_metallic: int = 0):
        '''GetSortedIndex
        * 지정된 Criteria에 따라 Filtered & Sorted Index Array 반환
         [idx, near_box_edge_ratio, near_box_radius_ratio, near_box_height,
         swell_similarity_score, mahalanobis_distance, i, cp_point, cp_normal,
         cp_normal_dot, cp_z, near_points])

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
                   'swell_similarity_score', 'mahalanobis_distance', 'instance_class', 'cp_point', 'cp_normal',
                   'cp_normal_dot', 'point_z', 'near_points']

        df = pd.DataFrame(data=data, columns=columns)

        if df.shape[0] == 0:
            idx = np.asarray(df['idx'])
            return idx, df
        if nbh >= 1:
            nbh = 1
        if md >= 1:
            md = 1

        is_nberatio = df['near_box_edge_ratio'] >= nbe
        is_nbrratio = df['near_box_radius_ratio'] >= nbr

        if is_metallic and object_type:
            ss_tmp = df['swell_similarity_score'].values
            df['swell_similarity_score'] = (ss_tmp - ss_tmp.min()) / ss_tmp.ptp()
            is_ssscore = df['swell_similarity_score'] >= ss
            df_sorted = df[is_nberatio & is_ssscore]

            # cp_normal_dot으로 cut
            if df_sorted.shape[0] > 3:
                is_md = df_sorted['cp_normal_dot'] < np.percentile(df_sorted['cp_normal_dot'], md * 100)
                df_sorted = df_sorted[is_md]
            # mahalanobis distance로 정렬
                df_sorted = df_sorted.sort_values(by=['mahalanobis_distance'], axis=0, ascending=True).reset_index(drop=True)
            idx_sorted = np.asarray(df_sorted['idx'])

        elif object_type:
            df_sorted = df[is_nberatio & is_nbrratio]

            # nearbox height로 cut 하고 mahalanobis distance로 cut
            if df_sorted.shape[0] > 3:
                is_nbh = df_sorted['near_box_height'] < np.percentile(df_sorted['near_box_height'], nbh * 100)
                df_sorted = df_sorted[is_nbh]
            if df_sorted.shape[0] > 3:
                is_md = df_sorted['mahalanobis_distance'] < np.percentile(df_sorted['mahalanobis_distance'], md * 100)
                df_sorted = df_sorted[is_md]
            # cp_normal_dot으로 정렬
            df_sorted = df_sorted.sort_values(by=['cp_normal_dot'], axis=0, ascending=False).reset_index(drop=True)
            idx_sorted = np.asarray(df_sorted['idx'])

        else:
            is_ssscore = df['swell_similarity_score'] >= ss
            df_sorted = df[is_nberatio & is_nbrratio & is_ssscore]

            # nearbox volume로 cut 하고 mahalanobis distance로 cut
            if df_sorted.shape[0] > 3:
                is_nbh = df_sorted['near_box_height'] < np.percentile(df_sorted['near_box_height'], nbh * 100)
                df_sorted = df_sorted[is_nbh]
            if df_sorted.shape[0] > 3:
                is_md = df_sorted['mahalanobis_distance'] < np.percentile(df_sorted['mahalanobis_distance'], md * 100)
                df_sorted = df_sorted[is_md]
            # cp_normal_dot으로 정렬
            df_sorted = df_sorted.sort_values(by=['cp_normal_dot'], axis=0, ascending=False).reset_index(drop=True)
            idx_sorted = np.asarray(df_sorted['idx'])

        return idx_sorted, df_sorted

    def GetRoiBox(self, pcd, bbox, margin_xy: float = 0, margin_floor: float = 0.2, scale_xy: float = 0):
        """GetRoiBox
        * 입력된 bounding box에 xy margin 및 floor margin 적용해서 return
        """
        assert(0 <= margin_xy < 0.5)
        assert(0 <= margin_floor < 1)
        assert(0 <= scale_xy <= 1)

        pcd_points_idx = np.asarray(bbox.get_point_indices_within_bounding_box(pcd.points))
        if len(pcd_points_idx) < 3:
            return bbox

        points = np.asarray(pcd.points)[pcd_points_idx]
        if scale_xy:
            points = np.asarray(pcd.points)[np.intersect1d(pcd_points_idx, np.argwhere(pcd.colors == [0, 0, 1]))]
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

    def DepthFill(self, array):
        df = pd.DataFrame(np.asarray(array))
        df.replace(0, np.NaN, inplace=True)
        df.fillna(self.region[2], inplace=True)
        df0 = df.interpolate(axis = 0).to_numpy()
        df1 = df.interpolate(axis = 1).to_numpy()
        return (df0 + df1)/2