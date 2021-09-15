# Threading for monitor camera
import threading
import cv2
import numpy as np
import imutils
import socket, pickle, struct
import math
from realPos import *
from dronekit import LocationGlobal, LocationGlobalRelative

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
        s_address= '192.168.43.233',  
        s_port = 12345,
        daemon=True, 
        name="ThreadedVideoStream"
    ):
        
        
        # Server-Client oriented frame transmission
        # Need to change Server-None orientation (UDP)
        # Changed !!
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
        self.line = cv2.LINE_AA
        self.fontscale = 0.5
        
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
        self.red = False
        self.blue = False
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
                        
                        
                        (sizeX,sizeY)= process(self.colorLoc, self.vehicle.location.global_relative_frame.alt if self.vehicle is not None else 1)
                        
                        # X ekseni çizilir
                        cv2.line(self.frame, (self.center[0],self.center[1]), (xg+(wg//2),self.center[1]), self.color["green"])
                        cv2.putText(self.frame, ("%.2f" % (sizeX*100)) + "cm", (xg+(wg//2),self.center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                    self.color["green"], 1, cv2.LINE_AA, False)
                        
                        #Y ekseni çizilir
                        cv2.line(self.frame, (self.center[0],self.center[1]), (self.center[0], yg+(hg//2)), self.color["green"])
                        cv2.putText(self.frame, ("%.2f" % (sizeY*100)) + "cm", (self.center[0], yg+(hg//2)), cv2.FONT_HERSHEY_SIMPLEX, 1, 
                        self.color["green"], 1, cv2.LINE_AA, False)
                        
                if self.vehicle is not None:
                    # up - left corner
                    cv2.putText(self.frame, ("Altitude: %.2f" % (self.vehicle.location.global_relative_frame.alt)) + "cm", (10,20), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                    self.color["green"], 1, self.line, False)
                    cv2.putText(self.frame, str(self.vehicle.gps_0), (10,40), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                    self.color["green"], 1, self.line, False)
                    cv2.putText(self.frame, "EKF: "+str(self.vehicle.ekf_ok), (10,60), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                    self.color["green"], 1, self.line, False)
                    cv2.putText(self.frame, "Mode: %s" % self.vehicle.mode.name, (10,80), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                    self.color["green"], 1, self.line, False)
                    cv2.putText(self.frame, "Armed: %s" % self.vehicle.armed, (10,100), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                    self.color["green"], 1, self.line, False)
                    
                    # up - right corner
                    cv2.putText(self.frame, "Roll: %.3f" % self.vehicle.attitude.roll + " Pitch: %.3f" % self.vehicle.attitude.pitch + " Yaw: %.3f" % self.vehicle.attitude.yaw, (self.center[0]*2-325,20), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                    self.color["green"], 1, self.line, False)
                    cv2.putText(self.frame, "Groundspeed: %.3f" % self.vehicle.groundspeed, (self.center[0]*2-180,40), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                    self.color["green"], 1, self.line, False)
                    cv2.putText(self.frame, "Airspeed: %.3f" % self.vehicle.airspeed, (self.center[0]*2-180,60), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                    self.color["green"], 1, self.line, False)
                    
                    # down - left corner
                    if type(self.vehicle.location.global_relative_frame) is LocationGlobal and self.vehicle.location.global_relative_frame is not None:
                        cv2.putText(self.frame, ("Lat: %.3f" %self.vehicle.location.global_frame.lat+" Lon: %.3f" %self.vehicle.location.global_frame.lon+ " Alt: %.3f" %self.vehicle.location.global_frame.alt), (10,self.center[1]*2-10), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                            self.color["green"], 1, self.line, False)
                    elif type(self.vehicle.location.global_relative_frame) is LocationGlobalRelative and self.vehicle.location.global_relative_frame is not None:
                        cv2.putText(self.frame, ("Lat: %.3f" %self.vehicle.location.global_relative_frame.lat+" Lon: %.3f" %self.vehicle.location.global_relative_frame.lon+ " Alt: %.3f" %self.vehicle.location.global_relative_frame.alt), (10,self.center[1]*2-10), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                            self.color["green"], 1, self.line, False)
                    if self.vehicle.location.local_frame.north is not None and self.vehicle.location.local_frame.east is not None and self.vehicle.location.local_frame.down is not None:
                        cv2.putText(self.frame, ("North: %.3f" %self.vehicle.location.local_frame.north+" East: %.3f"%self.vehicle.location.local_frame.east+ " Down: %.3f" %self.vehicle.location.local_frame.down), (10,self.center[1]*2-30), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                            self.color["green"], 1, self.line, False)
                    
                    # down - right corner
                    # print current time down right corner of self.frame
                    cv2.putText(self.frame, ("Blue found: "+ str(self.blue)), (self.center[0]*2-150,self.center[1]*2-50), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                        self.color["green"], 1, self.line, False)
                    cv2.putText(self.frame, ("Red found: " + str(self.red)), (self.center[0]*2-150,self.center[1]*2-30), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                        self.color["green"], 1, self.line, False)
                    cv2.putText(self.frame, time.strftime("%H:%M:%S"), (self.center[0]*2-90,self.center[1]*2-10), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                        self.color["green"], 1, self.line, False)
                    
                    # north arrow
                    radius = 150
                    self.frame = cv2.circle(self.frame, self.center, math.ceil(radius/15), self.color["green"], 1)

                    theta = (- self.vehicle.heading - 90) * 3.14 / 180.0
                    north = (math.ceil(self.center[0] + radius * math.cos(theta)), math.ceil(self.center[1] + radius * math.sin(theta)))
                    cv2.putText(self.frame, "N", (north[0] - 10, north[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                                    self.color["green"], 1, cv2.LINE_AA, False)
                    cv2.arrowedLine(self.frame, self.center, north, self.color["green"])
                    img = cv2.putText(self.frame, "%.2f"%((theta*180/3.14+90)%360), (self.center[0]-10, self.center[1] + 40), cv2.FONT_HERSHEY_SIMPLEX, self.fontscale, 
                        self.color["green"], 1, cv2.LINE_AA, False)
                                            
                if self.imwrite:
                    if self.out is None:
                        self.out = cv2.VideoWriter(self.fileName,cv2.VideoWriter_fourcc(*'DIVX'), 20, self.frame.shape[:2][::-1],1)
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
    
    def setLoc(self):
        self.colorLoc = (0,0,0,0,0,0)

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
    
    def setRed(self):
        self.red = True
    
    def setBlue(self):
        self.blue = True
    
    def finish(self):
        print("Camera stopped")
        self.stop()
        time.sleep(1)
        if self.livestream:
            print("Server socket closed")
            self.server_socket.close()
        if self.imwrite:
            print("Video released")
            self.out.release()
        if self.imshow:
            print("Windows closed")
            #cv2.destroyAllWindows()
        self.stream.release()
        print("Stream released")
        print("Exiting ThreadedVideoStream")
        

