# Tests
delaunaytest.py, homogrpahy+yolo.py, image_process_test.py, onnxtest.py, yolo_onnxtest.py and yolotest.py are all tests to run on computer.

# Trackdrive
The trackdrive can not run on a raspberry pi 3 B+ alone as YOLO takes a long time to run (+-1sec). For that reason trackdrive uses a connection between computer and raspberry pi.
That way the computer can do most of the calculation including running YOLO.

To run trackdrive, you have to run trackdrive.py (in rpi-3-B+) on the raspberry pi and run trackdrive_comp.py on the computer.
Do not forget to change the address of the raspberry pi in trackdrive_comp.py.