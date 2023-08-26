import socket
import cv2
import pickle
import struct
import time

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
payload_size = struct.calcsize(">L")

class Client():
    def __init__(self, host, port=8485):
        self.host = host
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.host, self.port))
        self.connection = self.client_socket.makefile('wb')
    
    def send_frame(self, frame):
        result, frame = cv2.imencode('.jpg', frame, encode_param)
        data = pickle.dumps(frame, 0)
        size = len(data)

        # print("{}".format(size))
        self.client_socket.sendall(struct.pack(">L", size) + data)
    
    def send_frame_recv_path(self, frame):
        self.send_frame(frame)

        # receive path
        data = b""
        # print("payload_size: {}".format(payload_size))
        while len(data) < payload_size:
            # print("Recv: {}".format(len(data)))
            data += self.client_socket.recv(4096)
        
        # print("Done Recv: {}".format(len(data)))
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]
        # print(msg_size)
        # print("msg_size: {}".format(msg_size))
        while len(data) < msg_size:
            data += self.client_socket.recv(4096)
        path_data = data[:msg_size]
        data = data[msg_size:]
        

        path = pickle.loads(path_data, fix_imports=True, encoding="bytes")
        return path

class Server():
    def __init__(self, port=8485):
        self.host = ''
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        
        self.conn, self.addr = self.server_socket.accept()
    
    def recv_frame(self):
        data = b""
        # print("payload_size: {}".format(payload_size))
        while len(data) < payload_size:
            # print("Recv: {}".format(len(data)))
            data += self.conn.recv(4096)
        
        # print("Done Recv: {}".format(len(data)))
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]
        # print("msg_size: {}".format(msg_size))
        while len(data) < msg_size:
            data += self.conn.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]

        frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        return frame
    
    def send_path(self, path):
        data = pickle.dumps(path, 0)
        size = len(data)
        self.conn.sendall(struct.pack(">L", size) + data)
        # print(size)