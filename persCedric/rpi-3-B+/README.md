# import requirements:
sudo apt-get update
sudo apt-get upgrade
sudo pip3 install -U pip setuptools wheel
sudo apt install libcamera-apps
sudo pip3 install opencv-python
sudo pip3 install onnxruntime
sudo pip3 install matplotlib

# test camera: (--qt-preview is to test camera using a VNC)
libcamera-hello --qt-preview