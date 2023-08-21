# https://gist.github.com/kittinan/e7ecefddda5616eab2765fdb2affed1b

from classes.socket import Client
import cv2

path = "file.png"

client_socket = Client("192.168.1.21")

frame = cv2.imread(path)
client_socket.send(frame)