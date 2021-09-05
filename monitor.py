# Threading for monitor camera
import threading
import cv2
import numpy as np
import imutils
import socket, pickle, struct
from realPos import *

class ThreadedVideoStream:
    def __init__(
        self, 
        src=0, 
        imshow=False,
        windowsName="drone Cam",
        imwrite=False, 
        fileName="drone.avi",
        livestream=False, 
        stream_address= ('192.168.137.233', 9999),  
        daemon=True, 
        name="ThreadedVideoStream"
    ):
        self.livestream = livestream
        
        if self.livestream:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket_address = stream_address
            self.server_socket.bind(self.socket_address)
            try:
                self.server_socket.listen(5)
                print("Listening at: ",self.socket_address)
                self.client_socket, self.addr = self.server_socket.accept()
                print("Client connected")
            except (OSError,IOError):
                print("Server socket error")
                self.server_socket.close()
                self.server_socket = None
                self.livestream = False
        
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
            "blue": (255,0,0),
            "red" : (0,0,255),
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
                        
                        
                        (sizeX,sizeY)= process(self.colorLoc,altitude = 2)
                        
                        # X ekseni çizilir
                        cv2.line(self.frame, (self.center[0],self.center[1]), (xg+(wg//2),self.center[1]), self.color[self.whichColor])
                        cv2.putText(self.frame, ("%.2f" % (sizeX*100)) + "cm", (xg+(wg//2),self.center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                    self.color[self.whichColor], 2, cv2.LINE_AA, False)
                        
                        #Y ekseni çizilir
                        cv2.line(self.frame, (self.center[0],self.center[1]), (self.center[0], yg+(hg//2)), self.color[self.whichColor])
                        cv2.putText(self.frame, ("%.2f" % (sizeY*100)) + "cm", (self.center[0], yg+(hg//2)), cv2.FONT_HERSHEY_SIMPLEX, 1, 
                        self.color[self.whichColor], 2, cv2.LINE_AA, False)
                        
                if self.imwrite:
                    if self.out is None:
                        self.out = cv2.VideoWriter(self.fileName,cv2.VideoWriter_fourcc(*'DIVX'), 11, self.frame.shape[:2][::-1],1)
                    self.out.write(self.frame)
                
                if self.imshow:
                    cv2.imshow("Drone Cam", self.frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                
                if self.livestream:
                    a = pickle.dumps(self.frame)
                    data = struct.pack("Q", len(a)) + a
                    self.client_socket.sendall(data)

    def read(self):
        # return the frame most recently read
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
    
    def stop(self):
        self.stopped = True
    
    def finish(self):
        print("before stop")
        self.stop()
        print("after stop")
        if self.imwrite:
            self.out.release()
        print("after release out")
        if self.imshow:
            cv2.destroyWindow("Drone Cam")
        print("after destroy")
        self.stream.release()
        print("after release stream")
        print("Exiting ThreadedVideoStream")
        

"""
class ThreadedVideoSave:
    def __init__(self, q_in=None, videoOutput= "project.avi", daemon = True,name="ThreadedVideoSave"):
        # initialize queue
        self.q_in = q_in
        self.out = None
        self.videoOutput = videoOutput
        # initialize the thread name
        self.name = name
        # initialize the variable used to indicate if the thread should
        # be stopped
        self.daemon = daemon
        self.stopped = False
        # start the thread to read frames from the video stream
        self.t = threading.Thread(name=self.name, target=self.update,daemon=self.daemon, args=())
        self.t.start()
        return
    
    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # otherwise, read the next frame from the stream
            if self.q_in.empty() is False:
                self.frame = self.q_in.get()
                if self.out is None:
                    self.out = cv2.VideoWriter(self.videoOutput,cv2.VideoWriter_fourcc(*'DIVX'), 10, self.frame.shape[:2][::-1],1)
                self.out.write(self.frame)
        return

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
    
    def finish(self):
        while self.q_in.empty() is False:
            self.out.write(self.q_in.get())
        self.out.release()
        print("Exiting ThreadedVideoSave")
"""