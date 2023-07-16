import cv2
from yolo.point import Point
from ultralytics import YOLO #pip install ultralytics

class Yolo:
	def __init__(self, bboxes=True):
		self.bboxes = bboxes
		#self.nn_path = "YOLOv8n_FSOCO.pt"
		self.model = YOLO("yolo/YOLOv8n_FSOCO.pt")

	def feed_forward(self, frame):

		results = self.model(frame, stream=True)
		yellow_cones = []
		blue_cones = []

		for result in results:
			for box in result.boxes.boxes:
				p1 = Point(box[0].item(), box[1].item())
				p2 = Point(box[2].item(), box[3].item())
				#probability = box[4].item()
				category = int(box[5].item())
				center = Point((p1.x+p2.x)/2, (p1.y+p2.y)/2)
				if category == 0: # yellow
					color = (0, 255, 255)
					yellow_cones.append(center)
				elif category == 1: # blue
					color = (255, 0, 0)
					blue_cones.append(center)
				else:
					color = (255, 255, 255)
				if self.bboxes:
					cv2.rectangle(frame, p1.get_int(), p2.get_int(), color, 3)
		return (yellow_cones, blue_cones)
