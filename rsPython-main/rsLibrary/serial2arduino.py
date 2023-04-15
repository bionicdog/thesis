#!/usr/bin/env python3
import serial
import time
from threading import Thread

ser = None
listen_thread = None

def initSerial(handleMessage=None):
    global ser, listen_thread
    try:        
        ser = serial.Serial('/dev/ttyACM0', 1000000, timeout=1)
    except:
        ser = None
        print('serial connection could not be started')
        return
    ser.flush()
    print('serial connection started')
    listen_thread = Thread(target = listen, args = [handleMessage])
    listen_thread.start()
    
def insertcoord(x,y,z,servo, home = 0, handleMessage=None):
    global ser
    if ser is None:
        print('Initialization of serial connection.')
        initSerial(handleMessage)
        if ser is not None:
            time.sleep(25)
            print('Homing should be finished')
    print('Moving to x = '+str(x)+', y = '+str(y)+', z = '+str(z)+', angle = '+str(servo))
    try:
        ser.write(bytes(""+str(x)+";"+str(y)+";"+str(z)+";"+str(servo)+";"+str(home)+" \n", 'utf-8'))
        ser.flush()
    except:
        print('Sending command over serial failed...')
        
continueListening = True
def listen(handleMessage=None):
    global ser, continueListening
    #prev_msg = ''
    while continueListening:
        line = str( ser.readline() ) #.decode('utf-8').rstrip()
        #msg = str(line) # 
        msg = line[2:len(line)-5]
        # prev_msg != msg:
        if msg is not None and len(msg) > 2:
            #print(msg)
        #prev_msg = msg
            if callable(handleMessage):
                handleMessage(msg)
        time.sleep(0.01)
    print('Finished listening. Closing serial.')
    ser.close()

if __name__ == '__main__':
    #ser = serial.Serial('/dev/ttyACM0', 1000000, timeout=1)
    #ser.flush()
    #time.sleep(12)
    insertcoord(22, 0, 0, 60, 0)
    #ser.write(b"24;4;0;40 \n")
    key = input('press to continue')
    insertcoord(25, 1, 0, 60, 0)
    #ser.write(b"24;4;0;40 \n")
    key = input('press to stop')
  #  time.sleep(100)
    continueListening = False
    listen_thread.join()