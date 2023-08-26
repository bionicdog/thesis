import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

# from classes.camera import Camera
from rpi3Bplus.classes.yolo_onnx import Yolo
from rpi3Bplus.classes.homography import Homography
from rpi3Bplus.classes.delaunay import Delaunay

from rpi3Bplus.classes.socket import Server

# print timestamps
timestamps = True
# showimages
recordrun = True
floorplan = False # TODO
cameraview = True

if recordrun and floorplan:
    import matplotlib.pyplot as plt

'''functions'''
def showGroundplan(coordinates, class_ids):
    plt.figure(0)
    plt.plot(0, 0, "*", color="black")
    #colors = model.get_colors()
    colors = ["gold", "blue", "orange", "red"]
    for i in range(len(class_ids)):
        color = colors[class_ids[i]]
        #color = [value/255 for value in color]
        plt.plot(coordinates[i][0][0], coordinates[i][0][1], "^", color=color)
    plt.xlabel("x (mm)")
    plt.ylabel("y (mm)")
    plt.show()
    return

'''initialisation'''
distance_grid = 150

onnx_path = "rpi3Bplus/data/YOLOv8n_FSOCO.onnx"
model = Yolo(onnx_path)
print("Yolo model initialised!")

homography = Homography(distance_grid=distance_grid, square_in_grid=[15, 15])
print("Homography initialised!")

delaunay = Delaunay()
print("Delaunay path-planning initialised")

print("Searching for connection ...")
server_socket = Server()
print("Connection established")

if recordrun & cameraview:
    fps = 10
    time_to_run = 30 # seconds
    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    file = cv2.VideoWriter('trackdrive_videos/cameraview.avi', fourcc, fps, (448, 448))
if recordrun & floorplan:
    file2 = cv2.VideoWriter('trackdrive_videos/map.avi', fourcc, fps, (448, 448))


'''running loop'''
if __name__ == '__main__':
    t_start = time.perf_counter()
    frame = server_socket.recv_frame()
    mask = homography.calculateMask(frame)
    # plt.imshow(frame)
    # plt.show()

    if mask is not None:
        print("Homography mask calculated!")
    else:
        print("Homography failed: didn't find chessboard")
        print("Check if chessboard is {} mm in front of the car".format(distance_grid))
        exit(1)
    
    # to prevent deadlock
    server_socket.send_path([[0,0]])
    counter = 0
    while True:
        counter += 1
        # duration processing 1 frame
        t0 = time.perf_counter()

        frame = server_socket.recv_frame()
        t1 = time.perf_counter()

        # running Yolo on the frame
        boxes, scores, class_ids = model.feed_forward(frame)
        t2 = time.perf_counter()

        # extracting cones-position pixel-coordinates
        centerpoints = model.xyxyBoxes_to_bottom_centerpoints(boxes)

        # calculating homography
        if len(centerpoints) == 0:
            # BRAKE ------------------------------------ BRAKE
            print("No more cones detected")
            exit(0)
        if mask is not None:
            world_coordinates = homography.perspectiveTransform(centerpoints)
        t3 = time.perf_counter()

        # path-planning
        # output only for debugging as it is saved in the class
        if (len(world_coordinates) >= 4):
            yellow_edges, blue_edges, mixed_edges = delaunay.delaunay(world_coordinates, class_ids)
            path = delaunay.getPath()
        else:
            path = [[0, 0]] # brake

        t4 = time.perf_counter()

        # send path control on rpi
        server_socket.send_path(path)

        t5 = time.perf_counter()

        # show timestamps
        if timestamps:
            print(f"Frame\tOverall\tReceive\tYOLO\tHomography\tPath-planning\tSend")
            print(f"{counter}\t{(t5 - t0)*1000:.2f} ms\t{(t1 - t0)*1000:.2f} ms\t{(t2 - t1)*1000:.2f} ms\t{(t3 - t2)*1000:.2f} ms\t{(t4 - t3)*1000:.2f} ms\t{(t5 - t4)*1000:.2f} ms")
            print(f"average fps: {counter / (time.perf_counter() - t_start)}")
            # print(f"Processing image {counter}")
            # print(f"Overall time for 1 frame: \t{(t3 - t0)*1000:.2f} ms")
            # print(f"Take image time for 1 frame: \t{(t1 - t0)*1000:.2f} ms")
            # print(f"YOLO time for 1 frame: \t{(t2 - t1)*1000:.2f} ms")
            # print(f"Homography time for 1 frame: \t{(t3 - t2)*1000:.2f} ms")

        # show output
        if recordrun:
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
                    exit(0)