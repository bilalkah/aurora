# Threading for monitor camera
import threading
import cv2
import numpy as np
import imutils
import socket, pickle, struct
import math
from realPos import *

class FrameSegment(object):
    def __init__(self, sock, port = 12345, address = "127.0.0.1"):
        self.s = sock
        self.port = port
        self.addr = address
    
    def udp_frame(self, img):
        MAX_DGRAM = 2**16
        MAX_IMAGE_DGRAM = MAX_DGRAM - 64
        compress_img = cv2.imencode('.jpg', img)[1]
        dat = compress_img.tobytes()
        size = len(dat)
        num_of_segments = math.ceil(size/MAX_IMAGE_DGRAM)
        array_pos_start = 0
        
        while num_of_segments:
            array_pos_end = min(size, array_pos_start + MAX_IMAGE_DGRAM)
            self.s.sendto(
                struct.pack("B",num_of_segments) +
                dat[array_pos_start:array_pos_end],
                (self.addr, self.port)
            )
            array_pos_start = array_pos_end
            num_of_segments -= 1

class ThreadedVideoStream:
    def __init__(
        self, 
        vehicle = None,
        src=0, 
        imshow=False,
        windowsName="drone Cam",
        imwrite=False, 
        fileName="drone.avi",
        livestream=False, 
        s_address= '127.0.0.1',  
        s_port = 12345,
        daemon=True, 
        name="ThreadedVideoStream"
    ):
        
        
        # Server-Client oriented frame transmission
        # Need to change Server-None orientation (UDP)
        self.livestream = livestream
        if self.livestream:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.fs = FrameSegment(self.server_socket, s_port, s_address)
            print("UDP server up")
            
        self.vehicle = vehicle
        self.stream = cv2.VideoCapture(src)
        self.imshow = imshow
        self.imwrite = imwrite
        self.fileName = fileName
        self.windowsName = windowsName
        self.out = None
        
        (self.grabbed, self.frame) = self.stream.read()
        if self.grabbed:
            print("Frame size: ", self.frame.shape)
        self.center = (int(self.stream.get(cv2.CAP_PROP_FRAME_WIDTH)/2), int(self.stream.get(cv2.CAP_PROP_FRAME_HEIGHT)/2))
        
        self.colorDict = {
            "blue" : {
                "lower_color" : [75, 100, 100],
                "upper_color" : [130, 255, 255],
            },
            "red" : {
                "lower_color" : [160, 100, 100],
                "upper_color" : [179, 255, 255],
            },
        }
        self.whichColor = None
        self.maskLower = None
        self.maskUpper = None
        self.color = {
            "red" : (0,0,255),
            "green":(0,255,0),
            "blue": (255,0,0),            
        }
        self.colorLoc = (0,0,0,0,0,0)
          
        # threading
        self.name = name
        self.daemon = daemon
        self.stopped = False
        # start the thread to read frames from the video stream
        threading.Thread(name=self.name, target=self.update,daemon=self.daemon, args=()).start()
        return

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                continue
                
            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()
            if self.grabbed:
                #self.frame = cv2.flip(self.frame, 1)
                if self.whichColor is not None:
                    blurred = cv2.GaussianBlur(self.frame, (31, 31), 0)
                    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
                    mask = cv2.inRange (hsv, self.maskLower, self.maskUpper)
                    colorcnts = cv2.findContours(mask.copy(),
                            cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
                    colorcnts = imutils.grab_contours(colorcnts)
                    if len(colorcnts) > 0:
                        (xg,yg,wg,hg) = cv2.boundingRect(max(colorcnts, key=cv2.contourArea))
                        self.colorLoc = (xg,yg,wg,hg,self.center[0],self.center[1])
                        
                        # bounding box and line
                        self.frame = cv2.rectangle(self.frame,(xg,yg),(xg+wg, yg+hg),self.color[self.whichColor],2)
                        self.frame = cv2.line(self.frame,(self.center[0],self.center[1]),(xg+(wg//2),yg+(hg//2)),self.color[self.whichColor],2)
                        
                        
                        (sizeX,sizeY)= process(self.colorLoc,altitude = 1) #self.vehicle.location.global_relative_frame.alt
                        
                        # X ekseni çizilir
                        cv2.line(self.frame, (self.center[0],self.center[1]), (xg+(wg//2),self.center[1]), self.color["green"])
                        cv2.putText(self.frame, ("%.2f" % (sizeX*100)) + "cm", (xg+(wg//2),self.center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                    self.color["green"], 1, cv2.LINE_AA, False)
                        
                        #Y ekseni çizilir
                        cv2.line(self.frame, (self.center[0],self.center[1]), (self.center[0], yg+(hg//2)), self.color["green"])
                        cv2.putText(self.frame, ("%.2f" % (sizeY*100)) + "cm", (self.center[0], yg+(hg//2)), cv2.FONT_HERSHEY_SIMPLEX, 1, 
                        self.color["green"], 1, cv2.LINE_AA, False)
                        
                if self.imwrite:
                    if self.out is None:
                        self.out = cv2.VideoWriter(self.fileName,cv2.VideoWriter_fourcc(*'DIVX'), 11, self.frame.shape[:2][::-1],1)
                    self.out.write(self.frame)
                
                if self.imshow:
                    cv2.imshow("Drone Cam", self.frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                
                if self.livestream:
                    self.fs.udp_frame(self.frame)

    def read(self):
        return (self.grabbed,self.frame)

    def readLoc(self):
        return self.colorLoc

    def setColor(self,color):
        if self.whichColor == color:
            return
        if color is not None:
            self.maskLower = np.array(self.colorDict[color]["lower_color"], dtype="uint8")
            self.maskUpper = np.array(self.colorDict[color]["upper_color"], dtype="uint8")
        self.whichColor = color
        
    def getColor(self):
        return self.whichColor
    
    def stop(self):
        self.stopped = True
    
    def finish(self):
        print("Camera stopped")
        stop()
        time.sleep(1)
        if self.livestream:
            self.server_socket.close()
            print("Server socket closed")
        if self.imwrite:
            self.out.release()
            print("Video released")
        if self.imshow:
            cv2.destroyWindow("Drone Cam")
            print("Windows closed")
        self.stream.release()
        print("Stream released")
        print("Exiting ThreadedVideoStream")
        

