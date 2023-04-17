from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import argparse
import time
import imutils
from multiprocessing import Process, Value
from multiprocessing import set_start_method

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="Path to the (optional) video file")
ap.add_argument("-b", "--buffer", default=64, type=int, help="max buffer size")
args = vars(ap.parse_args())

from threading import Thread, Event
from time import sleep

event = Event()
def Robocmd(message):
    #host = "127.0.0.1"
    host = "192.168.19.50"
    port = 8888
    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        soc.connect((host, port))
    except:
        print("Connection error")
        sys.exit()
    if 'click' in message:
        message = message.split(',')
        index = Buttons.index(message[1])
        points = coordinates[index]
        print(points)    
        z=0
        message = message[0]+'||'+str(points[0])+':'+str(points[1])+':'+str(z)
        print(message)
    if 'gripper' in message:
        message = message.split(',')
        message = message[0]+'||'+message[1]
        print(message)
    if 'movej' in message:
        message = message.split('||')
        message = message[0]+'||'+message[1]
        print(message)
    if 'movel' in message:
        message = message.split('||')
        message = message[0]+'||'+message[1]
        print(message)
    if message != 'quit':   
#        message = "\n".join([str(image_path), str(output_location)])
        soc.sendall(str.encode(message))    
        print(soc.recv(1024).decode())
        soc.send(b'--quit--')
#        soc.close()
    elif message == 'quit':
        soc.send(b'--quit--')
        print(soc.recv(1024).decode())

def Ball_tracking(cen_xy):
    #greenLower = (29, 86, 6)
    #greenUpper = (64, 255, 255)
    greenLower = (100, 100, 100)
    greenUpper = (250, 250, 250)
    pts = deque(maxlen=args["buffer"])

    if not args.get("video", False):
        vs = VideoStream(src=1).start()
    else:
        vs = cv2.VideoCapture(args["video"])

    time.sleep(2.0)


    while True:
        frame = vs.read()
        frame = frame[1] if args.get("video", False) else frame
        if frame is None:
            break

        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M['m10']/M['m00']), int(M['m01']/M['m00']))

            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                cen_xy[0] = int(x)
                cen_xy[1] = int(y)

        pts.append(center)

        for i in range(1, len(pts)):
            if pts[i-1] is None or pts[i] is None:
                continue

            thickness = int(np.sqrt(args["buffer"] / float(i+1)) * 2.5)
            cv2.line(frame, pts[i-1], pts[i], (0, 0, 255), thickness)

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        if event.is_set():
            break
        sleep(.5)

    if not args.get("video", False):
        vs.stop()
    else:
        vs.release()

    cv2.destroyAllWindows()



#Robocmd("Linearmov||205,490,200,90,180,-90||5.0"


def Robot_action(x,y):
    #Mapping need to do pixel to mm, here I given direct pixel value x,y
    cmd_string = "Linearmov||"+str(x)+','+str(y)+",200,90,180,-90||0.8"
    print("cmd_string:",cmd_string)
    #Robocmd(cmd_string)




def functioncall1(cen_x):
    while True:
        cen_x[0] += 1
        if event.is_set():
            break
        sleep(.5)
    print('Stop printing')
        
def modify_variable(var):
    while True:
        for i in range(len(var)):
            var[i] += 1
        if event.is_set():
            break
        sleep(.5)
    print('Stop printing')


#my_var = [1, 2, 3]
my_var = [0,0]
t = Thread(target=Ball_tracking, args=(my_var, ))
t.start()
while True:
    try:
        #print(my_var)
        Robot_action(my_var[0],my_var[1])
        sleep(1)
    except KeyboardInterrupt:
        event.set()
        break
t.join()
print(my_var)
