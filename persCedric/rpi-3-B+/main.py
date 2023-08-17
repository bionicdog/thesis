import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

from classes.camera import Camera
from classes.yolo_onnx import Yolo
from classes.homography import Homography

# print timestamps
timestamps = True
# showimages
showprocess = True
floorplan = False
cameraview = True

if floorplan:
    import matplotlib.pyplot as plt

'''functions'''
def showGroundplan(coordinates, class_ids):
    '''
    for i in range(len(class_ids)):
        cv2.circle(combined_img, (int(coordinates[i][0][0]), int(coordinates[i][0][1])), 2, model.get_color(class_ids[i]), 1)
        '''
    plt.figure(0)
    colors = model.get_colors()
    for i in range(len(class_ids)):
        color = colors[class_ids[i]]
        plt.plot(coordinates[i][0][0], coordinates[i][0][1], "^", color=color)
    plt.xlabel("x (mm)")
    plt.ylabel("y (mm)")
    plt.show()
    return

'''initialisation'''
camera = Camera()
print("Camera initialised!")

onnx_path = "data/YOLOv8n_FSOCO.onnx"
model = Yolo(onnx_path)
print("Yolo model initialised!")

homography = Homography()
print("Homography initialised!")

if showprocess & cameraview:
    time_to_run = 2
    fps = 15
    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    file = cv2.VideoWriter('../output.avi', fourcc, fps, (448, 448))


'''running loop'''
if __name__ == '__main__':
    frame = camera.get_frame()
    mask = homography.calculateMask(frame)
    plt.imshow(frame)
    plt.show()
    # image_test = cv2.imread("../20230716-1-small.jpg")
    # mask = homography.calculateMask(image_test)

    if mask is not None:
        print("Homography mask calculated!")
    else:
        print("Homography failed: didn't find chessboard")
        # exit(1)
    
    counter = 0
    while True:
        counter += 1
        # duration processing 1 frame
        t0 = time.perf_counter()

        frame = camera.get_frame()
        t1 = time.perf_counter()
        # cv2.imshow("image", frame)
        # cv2.waitKey(0)

        # running Yolo on the frame
        boxes, scores, class_ids = model.feed_forward(frame)
        t2 = time.perf_counter()

        # extracting cones-position pixel-coordinates
        centerpoints = model.xyxyBoxes_to_centerpoints(boxes)

        # calculating homography
        if mask is not None:
            world_coordinates = homography.perspectiveTransform(centerpoints)
        t3 = time.perf_counter()

        # show timestamps
        if timestamps:
            print(f"Processing image {counter}")
            print(f"Overall time for 1 frame: \t{(t3 - t0)*1000:.2f} ms")
            print(f"Take image time for 1 frame: \t{(t1 - t0)*1000:.2f} ms")
            print(f"YOLO time for 1 frame: \t{(t2 - t1)*1000:.2f} ms")
            print(f"Homography time for 1 frame: \t{(t3 - t2)*1000:.2f} ms")
        
        # show output
        if showprocess:
            if floorplan:
                showGroundplan(world_coordinates, class_ids)
            if cameraview:
                combined_img = model.draw_detections(frame)
                # cv2.imshow("camera view", combined_img)
                '''
                plt.figure(0)
                plt.imshow(combined_img)
                plt.show()
                '''
                print(f"writing image {counter}")
                file.write(combined_img)
                if counter == time_to_run*fps:
                    file.release()
                    break