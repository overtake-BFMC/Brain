import os
import sys
import numpy as np
np.set_printoptions(threshold=sys.maxsize)
import onnxruntime as ort
import cv2
from math import exp
import logging
import matplotlib.pyplot as plt

CLASSES = ['car', 'closed-road-stand', 'crosswalk-sign', 'highway-entry-sign', 'highway-exit-sign', 'no-entry-road-sign',
            'one-way-road-sign', 'parking-sign', 'parking-spot', 'pedestrian', 'priority-sign', 'round-about-sign',
            'stop-line', 'stop-sign', 'traffic-light']

COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

meshgrid = []
class_num = len(CLASSES)
headNum = 3
strides = [8, 16, 32]
mapSize = [[80, 80], [40, 40], [20, 20]]
input_imgH = 640
input_imgW = 640

class DetectBox:
    def __init__(self, classId, score, xmin, ymin, xmax, ymax):
        self.classId = classId
        self.score = score
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax

class signDetection:
    def __init__(self, conf_thresh=0.5, iou_thresh=0.45):

        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.model_path = dir_path + "/best.onnx"
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh
        self.detectionModelSession = ort.InferenceSession(self.model_path, providers=['CUDAExecutionProvider']) #TensorrtExecutionProvider

    @staticmethod
    def sigmoid(x):
        return 1 / (1 + exp(-x))

    @staticmethod
    def preprocess_image(img_src, resize_w, resize_h):
        image = cv2.resize(img_src, (resize_w, resize_h), interpolation=cv2.INTER_LINEAR)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image.astype(np.float32)
        image /= 255.0
        image = image.transpose((2, 0, 1))
        image = np.expand_dims(image, axis=0)
        return image
    
    def non_max_suppression(self, boxes, scores, class_ids, iou_threshold=0.5):
        """
        Perform Non-Maximum Suppression (NMS) on the detected bounding boxes.
        
        Parameters:
        - boxes: (N, 4) array of bounding boxes [x, y, w, h]
        - scores: (N,) array of object confidence scores
        - class_ids: (N,) array of class IDs
        - iou_threshold: IoU threshold for suppression
        
        Returns:
        - kept_boxes: Filtered bounding boxes after NMS
        - kept_scores: Corresponding confidence scores
        - kept_class_ids: Corresponding class IDs
        """

        # Convert (x, y, w, h) to (x1, y1, x2, y2) for OpenCV NMS
        x1 = boxes[:, 0] - boxes[:, 2] / 2
        y1 = boxes[:, 1] - boxes[:, 3] / 2
        x2 = boxes[:, 0] + boxes[:, 2] / 2
        y2 = boxes[:, 1] + boxes[:, 3] / 2

        # Prepare for OpenCV NMS
        box_array = np.stack([x1, y1, x2, y2], axis=1)
        
        # Apply OpenCV NMS
        indices = cv2.dnn.NMSBoxes(box_array.tolist(), scores.tolist(), score_threshold=0.5, nms_threshold=iou_threshold)

        if len(indices) == 0:
            return [], [], []

        indices = indices.flatten()

        # Return the selected boxes, scores, and class IDs
        return boxes[indices], scores[indices], class_ids[indices]
    
    def postprocess(self, pred_results, orig_img, confidence_threshold = 0.5):

        detectResult = []
        img_h, img_w, _ = orig_img.shape
        scale_h = img_h / input_imgH
        scale_w = img_w / input_imgW
        #print("ScaleW: ", scale_w)
        #print("ScaleH: ", scale_h)

        predictions = pred_results[0]  # (1, 19, 8400) → Extract first element
        predictions = predictions.squeeze(0)  # (19, 8400) → Remove batch dimension

        boxes = predictions[:4, :].T  # (8400, 4) -> (x, y, w, h)
        class_probs = predictions[4:, :].T  # (8400, 15) -> Class probabilities

        class_ids = np.argmax(class_probs, axis=1)  # Find class with highest probability
        class_scores = np.max(class_probs, axis=1)  # Get the corresponding confidence score

        valid_indices = class_scores > confidence_threshold
        valid_boxes = boxes[valid_indices]  # (N, 4)
        valid_class_scores = class_scores[valid_indices]  # (N,)
        valid_class_ids = np.where(valid_indices, class_ids, -1)[valid_indices]

        nms_boxes, nms_scores, nms_class_ids = self.non_max_suppression(valid_boxes, valid_class_scores, valid_class_ids)

        if isinstance(valid_class_scores, np.ndarray) and valid_class_scores.size != 0:
            scaled_boxes = nms_boxes.copy()
            #print("NMS: ", nms_boxes)
            scaled_boxes[:, 0] *= scale_w  # x
            scaled_boxes[:, 1] *= scale_h  # y
            scaled_boxes[:, 2] *= scale_w  # width
            scaled_boxes[:, 3] *= scale_h  # height
        else:
            scaled_boxes = np.array([])
            nms_class_ids = np.array([])
            nms_scores = np.array([])

        return scaled_boxes, nms_class_ids, nms_scores

    def detect(self, img_path):
        if isinstance(img_path, str):
            orig = cv2.imread(img_path)
        else:
            orig = img_path

        image = self.preprocess_image(orig, input_imgW, input_imgH)

        pred_results = self.detectionModelSession.run(None, {'images': image})
        valid_boxes, valid_class_ids, valid_class_scores = self.postprocess(pred_results, orig)

        #print("Valid: ", valid_indices )
        # Extract NumPy array from the list
        predictions = pred_results[0]  # (1, 19, 8400) → Extract first element
        predictions = predictions.squeeze(0)  # (19, 8400) → Remove batch dimension

        
        boxes = predictions[:4, :].T  # (8400, 4) -> (x, y, w, h)
        class_probs = predictions[4:, :].T  # (8400, 15) -> Class probabilities
        
        for box, class_id, class_score in zip(valid_boxes, valid_class_ids, valid_class_scores):
            x, y, w, h = box
            print(f"x: {x}, y: {y}, w: {w}, h: {h}, object confidence: {class_score:.2f}, class ID: {class_id}")
        #for box in predbox:
        #    boxes.append([int(box.xmin), int(box.ymin), int(box.xmax), int(box.ymax)])
        #    scores.append(box.score)
        #    class_ids.append(box.classId)
        conf_threshold = 0.5
        nms_threshold = 0.4

        class_ids = np.argmax(class_probs, axis=1)  # Find class with highest probability
        class_scores = np.max(class_probs, axis=1)  # Get the corresponding confidence score

        #print("IDS:", class_ids)
        #print("SCORES: ",class_scores)

        #final_scores = confidences * class_scores

        #valid_indices = final_scores > conf_threshold
        #filtered_boxes = boxes[valid_indices]
        #filtered_scores = final_scores[valid_indices]
        #filtered_class_ids = class_ids[valid_indices]

        #indices = cv2.dnn.NMSBoxes(
        #    filtered_boxes.tolist(), filtered_scores.tolist(), conf_threshold, nms_threshold
        #)

        #filtered_boxes = [filtered_boxes[i] for i in indices]
        #filtered_scores = [filtered_scores[i] for i in indices]
        #filtered_class_ids = [filtered_class_ids[i] for i in indices]

        return valid_boxes, valid_class_ids, valid_class_scores
        #return boxes, confidences, class_probs

    def draw_detections(self,image, boxes, class_ids, scores, mask_alpha=0.3):
        """
        Combines drawing masks, boxes, and text annotations on detected objects.
        
        Parameters:
        - image: Input image.
        - boxes: Array of bounding boxes.
        - scores: Confidence scores for each detected object.
        - class_ids: Detected object class IDs.
        - mask_alpha: Transparency of the mask overlay.
        """
        det_img = image.copy()

        img_height, img_width = image.shape[:2]
        font_size = min([img_height, img_width]) * 0.0006
        text_thickness = int(min([img_height, img_width]) * 0.001)

        mask_img = image.copy()

        # Draw bounding boxes, masks, and text annotations
        for class_id, box, score in zip(class_ids, boxes, scores):
            color = COLORS[class_id]
            # Convert (x1, y1, w, h) → (x1, y1, x2, y2)
            cx, cy, w, h = map(int, box)
            x_min = int(cx - w / 2)
            y_min = int(cy - h / 2)
            x_max = int(cx + w / 2)
            y_max = int(cy + h / 2)

            # Draw filled rectangle for mask
            cv2.rectangle(mask_img, (x_min, y_min), (x_max, y_max), color, -1)

            # Draw bounding box
            cv2.rectangle(det_img, (x_min, y_min), (x_max, y_max), color, 2)

            # Prepare text (label and score)
            label = CLASSES[class_id]
            caption = f'{label} {int(score * 100)}%'
            
            # Calculate text size and position
            (tw, th), _ = cv2.getTextSize(text=caption, fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                        fontScale=font_size, thickness=text_thickness)
            th = int(th * 1.2)
            
            # Draw filled rectangle for text background
            cv2.rectangle(det_img, (x_min, y_min - th), (x_min + tw, y_min), color, -1)

            # Draw text over the filled rectangle
            cv2.putText(det_img, caption, (x_min, y_min), cv2.FONT_HERSHEY_SIMPLEX, font_size,
                        (255, 255, 255), text_thickness, cv2.LINE_AA)

        # Blend the mask image with the original image
        det_img = cv2.addWeighted(mask_img, mask_alpha, det_img, 1 - mask_alpha, 0)

        return det_img

if __name__ == "__main__":
    # Replace '../frontend/' with the actual path to your Angular project
    testDetector = signDetection(conf_thresh=0.5, iou_thresh=0.45)

    dir_path = os.path.dirname(os.path.realpath(__file__))
    #image_path = dir_path + "/crosswalk_sign.jpg"
    image_path = dir_path + "/pure-black.jpg"
    image = cv2.imread(image_path)
    filtered_boxes, class_ids, class_scores = testDetector.detect(image)
    detected_image = image
    if class_scores.size != 0:
        detected_image = testDetector.draw_detections(image, filtered_boxes, class_ids, class_scores)
    cv2.imwrite("Detected.jpg", detected_image)
    logging.basicConfig(filename='signDetection.log', level=logging.INFO)
    logger = logging.getLogger()
    #msgStr = "Boxes: " + str(boxes)
    #logger.info(msgStr)
    #msgStr = "Confidences: " + str(scores)
    #logger.info(msgStr)
    #msgStr = "Class IDs:" + str(class_ids)
    #logger.info(msgStr)
    #msgStr = "Class IDs:" + str(results)
    #logger.info(msgStr)
    #for box, class_id, class_score in zip(filtered_boxes, class_ids, class_scores):
    #    x, y, w, h = box
    #    logger.info(f"x: {x}, y: {y}, w: {w}, h: {h}, class_id: {class_id} class Name: {CLASSES[class_id]}, highest prob: {class_score:.2f}")