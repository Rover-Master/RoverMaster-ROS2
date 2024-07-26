from ctypes import alignment
from pyclbr import Class
from ultralytics import YOLO
import argparse, os, torch, time, subprocess
from PIL import Image
import numpy as np
from control import init, move


# Inference Nav
class NavAlignment:
    def __init__(self, model_path, output_dir):
        self.model_path = model_path
        self.output_dir = output_dir
        self.model = YOLO(self.model_path)  # Load model once in the constructor

    def save_images(self, frame):
        os.makedirs(self.output_dir, exist_ok=True)
        self.process_image(frame)

    def process_image(self, image):
        results = self.model(image)
        boxes = results[0].boxes.xywh
        area = boxes[0][2] * boxes[0][3]
        if boxes.shape[0] > 0 and int(area) > 2000:
            # self.save_detection_results(boxes, image, results[0].orig_img.shape[1], results[0].orig_img.shape[0])
            return boxes

        else:
            print(f"Area have not med the criteria: {area}")
            return None

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
        return class_counts  # 0:Flower, 1:Healthy, 2:Stem_Top, 3:Unhealthy #Format: {0: 2, 3: 3, 1: 2, 2: 1}

    def save_detection_results(
        self, filtered_results, image_name, img_width, img_height
    ):
        save_path = os.path.join(self.output_dir, image_name.replace(".png", ".txt"))
        with open(save_path, "w") as file:
            for box in filtered_results:
                x_center, y_center, width, height = box[0], box[1], box[2], box[3]
                file.write(f"{x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")
        print(f"Saved {image_name} to {save_path}")


# Inference Detection
class Detection:
    def __init__(self, model_path, output_dir):
        self.model_path = model_path
        self.output_dir = output_dir
        self.model = YOLO(self.model_path)  # Load model once in the constructor

    def process_images(self, image):
        os.makedirs(self.output_dir, exist_ok=True)
        self.process_image(image)

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

    def save_detection_results(
        self, filtered_results, image_name, img_width, img_height
    ):
        save_path = os.path.join(self.output_dir, image_name.replace(".png", ".txt"))
        with open(save_path, "w") as file:
            for box in filtered_results:
                x_center, y_center, width, height = box[0], box[1], box[2], box[3]
                file.write(f"{x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")
        print(f"Saved {image_name} to {save_path}")


# TODO Update the models + location
def parse_args():
    parser = argparse.ArgumentParser(
        description="Run YOLO model using the pretrained weights for detection"
    )
    parser.add_argument(
        "--model_path_nav",
        type=str,
        default="/home/zahra/Documents/Robotics/perception/detection/yolov8n/runs/detect/trained_stem/weights/best.pt",
        help="Path to pretrained YOLO model on stem class",
    )
    parser.add_argument(
        "--model_path_det",
        type=str,
        default="/home/zahra/Documents/Robotics/perception/detection/yolov8n/runs/detect/trained_leaf_flower/weights/best.pt",
        help="Path to pretrained YOLO model on flower and unhealthy leaves classes",
    )
    parser.add_argument(
        "--model_path_map",
        type=str,
        default="/home/zahra/Documents/Robotics/perception/detection/yolov8n/runs/detect/trained_leaf_flower/weights/best.pt",
        help="Path to pretrained YOLO model for mapping",
    )
    return parser.parse_args()
