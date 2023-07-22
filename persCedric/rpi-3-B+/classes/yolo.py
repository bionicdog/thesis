import cv2

from ultralytics import YOLO #pip install ultralytics-yolo

class Yolo:
	def __init__(self, bboxes=True):
		self.bboxes = bboxes
		self.model = YOLO("data/YOLOv8n_FSOCO.pt")

	def feed_forward(self, frame):

		results = self.model(frame, stream=True)
		yellow_cones = []
		blue_cones = []

		for result in results:
			for box in result.boxes.boxes:
				p1 = (box[0].item(), box[1].item())
				p2 = (box[2].item(), box[3].item())
				#probability = box[4].item()
				category = int(box[5].item())
				# box bottomline center
				center = ((p1.x+p2.x)/2, (p2.y)) # Point((p1.x+p2.x)/2, (p1.y+p2.y)/2)
				if category == 0: # yellow
					color = (0, 255, 255)
					yellow_cones.append(center)
				elif category == 1: # blue
					color = (255, 0, 0)
					blue_cones.append(center)
				else:
					color = (255, 255, 255)
				if self.bboxes:
					p1 = (int(p1[0]), int(p1[1]))
					p2 = (int(p2[0]), int(p2[1]))
					cv2.rectangle(frame, p1, p2, color, 3)
		return (yellow_cones, blue_cones)
