# https://gist.github.com/kittinan/e7ecefddda5616eab2765fdb2affed1b

from rpi3Bplus.classes.socket import Server

import cv2
import matplotlib.pyplot as plt

server_socket = Server()

frame = server_socket.recv()

plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
plt.show()