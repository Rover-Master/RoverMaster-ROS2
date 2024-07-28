from ctypes import alignment
from pyclbr import Class
from ultralytics import YOLO
import argparse, os, torch, time, subprocess, cv2
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Inference Nav
class NavAlignment():
    def __init__(self, model_path):
        self.model_path = model_path
        self.model = YOLO(self.model_path) 

    def process_image(self, image):
        results = self.model(image)
        boxes = results[0].boxes.xywh[0]
        area = boxes[2] * boxes[3]
        if boxes.shape[0] > 0 and int(area) > 2000:
            # self.save_detection_results(boxes, image, results[0].orig_img.shape[1], results[0].orig_img.shape[0])
            return boxes

        else:
            print(f"Area have not med the criteria: {area}")
            return None

    # def prepare_roi(self, image, bbox):
    #     """ Crop and return the ROI from the image based on the bounding box. """
    #     x, y, w, h = bbox
    #     return image[y:y+h, x:x+w]

    # def detect_and_match_features(self, image1, image2):
    #     # Get detection
    #     bbox1 = self.process_image(image1)
    #     bbox2 = self.process_image(image2)
        
    #     # Initialize SIFT detector
    #     sift = cv2.SIFT_create()
    #     # surf = cv2.SURF_create()

    #     # Extract ROIs from the images based on the bounding boxes
    #     roi1 = self.prepare_roi(image1, bbox1)
    #     roi2 = self.prepare_roi(image2, bbox2)

    #     # Detect keypoints and compute descriptors
    #     kp1, des1 = sift.detectAndCompute(roi1, None)
    #     kp2, des2 = sift.detectAndCompute(roi2, None)

    #     # Matcher - FLANN based or BFMatcher if preferred
    #     matcher = cv2.BFMatcher()
    #     matches = matcher.knnMatch(des1, des2, k=2)

    #     # Apply Lowe's ratio test
    #     good_matches = []
    #     for m,n in matches:
    #         print('m.distance:', m.distance, 'n.distance:', n.distance)
    #         if m.distance < 0.75 * n.distance:
    #             good_matches.append(m)

    #     return good_matches, kp1, kp2

    # # Function to triangulate points
    # def triangulate_points(points1, points2, R, t, K):
    #     # Projection matrices
    #     P1 = np.hstack((np.eye(3, 3), np.zeros((3, 1))))
    #     P2 = np.hstack((R, t))
    #     P1 = K @ P1
    #     P2 = K @ P2

    #     # Triangulate points
    #     points_4d_hom = cv2.triangulatePoints(P1, P2, points1, points2)
    #     points_3d = cv2.convertPointsFromHomogeneous(points_4d_hom.T)

    #     return points_3d


    def point_recontruction(self,center1, center2, R, t):
            
            if not isinstance(center1, np.ndarray):
                center1 = center1.numpy()  
            if not isinstance(center2, np.ndarray):
                center2 = center2.numpy() 

            # Intrinsic matrix of teh camera
            K = np.array([[1.72450087e+03, 0.00000000e+00, 5.25392337e+02],
                        [0.00000000e+00, 2.00311395e+03, 1.08207026e+03],
                        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
            
            
            if center1.shape[0] == 2:
                center1 = np.append(center1, 1)  
            if center2.shape[0] == 2:
                center2 = np.append(center2, 1)  
            
            
            # Compute projection matrices
            P1 = K @ np.hstack((np.eye(3), np.zeros((3, 1))))
            P2 = K @ np.hstack((R, np.array([[t[0]], [t[1]], [t[2]]])))

            # Normalize points
            normalized_point1 = np.linalg.inv(K) @ np.array([center1[0], center1[1], 1])
            normalized_point2 = np.linalg.inv(K) @ np.array([center2[0], center2[1], 1])

            # Prepare points for triangulation 
            points1 = normalized_point1[:2].reshape(2, 1)
            points2 = normalized_point2[:2].reshape(2, 1)
            
            # Triangulate points to find 3D point
            point_4d_hom = cv2.triangulatePoints(P1, P2, points1.reshape(2, 1), points2.reshape(2, 1))
            point_3d = cv2.convertPointsFromHomogeneous(point_4d_hom.T) # Front of the camera

            return point_3d


# Inference Detection
class Detection():
    def __init__(self, model_path):
        self.model_path = model_path
        self.model = YOLO(self.model_path)  
    
    def process_image(self, image):
        # image_path = os.path.join(self.image_dir, image_name)
        results = self.model(image)
        boxes = results[0].boxes.xywh
        area = boxes[0][2] * boxes[0][3]
        det_boxes = []
        if boxes.shape[0] > 0 and int(area) > 2000:  # Check if there are any detections
            # self.save_detection_results(boxes, image, results[0].orig_img.shape[1], results[0].orig_img.shape[0])
            det_boxes.append(boxes)
            detections = torch.cat(det_boxes)
            return detections
        else:
            pass

    # Harvesting
    def det_target(box_det, cam_pos):
        x_cam, y_cam = cam_pos.x, cam_pos.y

        if box_det.size() > 0:
            dis_list = []
            for x_det in box_det:
                x_b, y_b, w_b, h_b = x_det.detach().cpu().numpy()
                distance = abs(((x_cam - x_b), (y_cam - y_b)))
                dis_list.append(distance)
                return min(dis_list)
        else:
            return None


class Mapping():
    def __init__(self, model_path):
        self.model_path = model_path
        self.model = YOLO(self.model_path) 

    def mapping(self, image):
        results = self.model(image)
        class_counts = {}
        for box in results[0].boxes:
            class_id = int(
                box.data[0][-1]
            )  # Assuming this is the correct way to get class_id from your results
            if class_id in class_counts:
                class_counts[class_id] += 1
            else:
                class_counts[class_id] = 1

        # summary = ", ".join(f"{count} {self.model.names[class_id]}" for class_id, count in class_counts.items()) # 2 Flowers, 2 Healthys, 1 Stem_Top, 3 Unhealthys
        return class_counts # 0:Flower, 1:Healthy, 2:Stem_Top, 3:Unhealthy #Format: {0: 2, 3: 3, 1: 2, 2: 1}
