import numpy as np
import cv2

import onnxruntime

from classes.utils_onnx import xywh2xyxy, nms, draw_detections, get_class_name, get_colors

class Yolo:
    def __init__(self, path_of_model, conf_thres=0.5, iou_thres=0.5):
        # thresholds
        self.conf_threshold = conf_thres
        self.iou_threshold = iou_thres
        # onnx model
        self.session = onnxruntime.InferenceSession(path_of_model)
        self.get_input_details()
        self.get_output_details()

    def get_input_details(self):
        model_inputs = self.session.get_inputs()
        self.input_names = [model_inputs[i].name for i in range(len(model_inputs))]
        self.input_shape = model_inputs[0].shape
        self.input_height = self.input_shape[2]
        self.input_width = self.input_shape[3]
    
    def get_output_details(self):
        model_outputs = self.session.get_outputs()
        self.output_names = [model_outputs[i].name for i in range(len(model_outputs))]

    def preprocess_input(self, frame):
        self.img_height, self.img_width = frame.shape[:2]
        
        input_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        input_frame = cv2.resize(input_frame, (self.input_width, self.input_height))
        # Scale input pixel values to 0 to 1
        input_frame = input_frame / 255.0
        input_frame = input_frame.transpose(2, 0, 1)
        input_tensor = input_frame[np.newaxis, :, :, :].astype(np.float32)
        
        return input_tensor
    
    def rescale_boxes(self, boxes):
        # Rescale boxes to original image dimensions
        input_shape = np.array([self.input_width, self.input_height, self.input_width, self.input_height])
        boxes = np.divide(boxes, input_shape, dtype=np.float32)
        boxes *= np.array([self.img_width, self.img_height, self.img_width, self.img_height])
        return boxes

    def extract_boxes(self, predictions):
        boxes = predictions[:, :4]
        # Scale boxes to original image dimensions
        boxes = self.rescale_boxes(boxes)
        # Convert boxes to xyxy format
        boxes = xywh2xyxy(boxes)
        return boxes
    
    def process_output(self, outputs):
        predictions = np.squeeze(outputs[0]).T
        
		# Filter out object confidence scores below threshold
        scores = np.max(predictions[:, 4:], axis=1)
        predictions = predictions[scores > self.conf_threshold, :]
        scores = scores[scores > self.conf_threshold]
        
        if len(scores) == 0:
            return [], [], []
        
		# Get the class with the highest confidence
        class_ids = np.argmax(predictions[:, 4:], axis=1)
        
		# Get bounding boxes for each object
        boxes = self.extract_boxes(predictions)
        
		# Apply non-maxima suppression to suppress weak, overlapping bounding boxes
        indices = nms(boxes, scores, self.iou_threshold)
        return boxes[indices], scores[indices], class_ids[indices]
    
    def feed_forward(self, frame):
        input_tensor = self.preprocess_input(frame)
        
		# Run inference model
        outputs = self.session.run(self.output_names, {self.input_names[0]: input_tensor})
        
        self.boxes, self.scores, self.class_ids = self.process_output(outputs)
        
        return self.boxes, self.scores, self.class_ids
    
    def draw_detections(self, image, mask_alpha=4):
        return draw_detections(image, self.boxes, self.scores, self.class_ids, mask_alpha)
    
    def xyxyBoxes_to_bottom_centerpoints(self, boxes):
        centerpoints = np.zeros((len(boxes), 1, 2))
        for i in range(len(boxes)):
            centerpoints[i][0][0] = (boxes[i][0] + boxes[i][2]) / 2
            centerpoints[i][0][1] = np.min([boxes[i][1], boxes[i][3]])
        return centerpoints
    
    def get_class_name(self, index):
        return get_class_name(index)
    
    def get_colors(self):
        return get_colors()