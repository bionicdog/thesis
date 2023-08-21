import numpy as np
import cv2

class_names = ['yellow_cone', 'blue_cone', 'orange_cone', 'big_orange_cone']
# Create a list of colors for each class where each color is a tuple of 3 integer values (BGR 255 to 0)
colors = [[255, 0, 0],      # yellow
          [0, 255, 255],    # blue
          [255, 90, 0],     # orange
          [255, 255, 0]]    # red

def xywh2xyxy(input_boxes):
    # print(np.shape(colors))
    # Convert bounding box (x, y, w, h) to bounding box (x1, y1, x2, y2)
    output_boxes = np.copy(input_boxes)
    output_boxes[..., 0] = input_boxes[..., 0] - input_boxes[..., 2] / 2
    output_boxes[..., 1] = input_boxes[..., 1] - input_boxes[..., 3] / 2
    output_boxes[..., 2] = input_boxes[..., 0] + input_boxes[..., 2] / 2
    output_boxes[..., 3] = input_boxes[..., 1] + input_boxes[..., 3] / 2
    return output_boxes

def compute_iou(box, boxes):
    # Compute xmin, ymin, xmax, ymax for both boxes
    xmin = np.maximum(box[0], boxes[:, 0])
    ymin = np.maximum(box[1], boxes[:, 1])
    xmax = np.minimum(box[2], boxes[:, 2])
    ymax = np.minimum(box[3], boxes[:, 3])

    # Compute intersection area
    intersection_area = np.maximum(0, xmax - xmin) * np.maximum(0, ymax - ymin)

    # Compute union area
    box_area = (box[2] - box[0]) * (box[3] - box[1])
    boxes_area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    union_area = box_area + boxes_area - intersection_area

    # Compute IoU
    iou = intersection_area / union_area

    return iou

def nms(boxes, scores, iou_threshold):
    # non-maxima suppression
    # extra information: https://towardsdatascience.com/non-maximum-suppression-nms-93ce178e177c
    # Sort by score
    sorted_indices = np.argsort(scores)[::-1]

    keep_boxes = []
    while sorted_indices.size > 0:
        # Pick the last box
        box_id = sorted_indices[0]
        keep_boxes.append(box_id)

        # Compute IoU of the picked box with the rest
        ious = compute_iou(boxes[box_id, :], boxes[sorted_indices[1:], :])

        # Remove boxes with IoU over the threshold
        keep_indices = np.where(ious < iou_threshold)[0]

        # print(keep_indices.shape, sorted_indices.shape)
        sorted_indices = sorted_indices[keep_indices + 1]
    
    return keep_boxes

def draw_detections(image, boxes, scores, class_ids, mask_alpha=0.3):
    mask_img = image.copy()
    det_img = image.copy()

    img_height, img_width = image.shape[:2]
    size = min([img_height, img_width]) * 0.001
    text_thickness = int(min([img_height, img_width]) * 0.001)

    # Draw bounding boxes and labels of detections
    for box, score, class_id in zip(boxes, scores, class_ids):
        color = colors[class_id]

        x1, y1, x2, y2 = box.astype(int)

        # Draw rectangle
        cv2.rectangle(det_img, (x1, y1), (x2, y2), color, 2)

        label = class_names[class_id]
        caption = f'{label} {int(score * 100)}%'
        (tw, th), _ = cv2.getTextSize(text=caption, fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                      fontScale=size, thickness=text_thickness)
        th = int(th * 1.2)

        cv2.putText(det_img, caption, (x1, y1),
                    cv2.FONT_HERSHEY_SIMPLEX, size, (255, 255, 255), text_thickness, cv2.LINE_AA)

    return cv2.addWeighted(mask_img, mask_alpha, det_img, 1 - mask_alpha, 0)

def get_class_name(index):
    if index >= len(class_names):
        print("index out of range")
        return None
    return class_names[index]

def get_colors():
    return colors