# Threading for monitor camera
import threading
import cv2
import numpy as np
import imutils


class ThreadedVideoStream:
    def __init__(self, src=0, imshow=False, imwrite=False, daemon=True, name="ThreadedVideoStream"):
        # initialize the video camera stream and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(src)
        self.imshow = imshow
        self.imwrite = imwrite
        #self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        #self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        (self.grabbed, self.frame) = self.stream.read()
        print("Frame size: ", self.frame.shape)
        # color detection
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
        
        self.out = None
        self.videoOutput = "videoOutput.avi"
        # initialize the thread name
        self.name = name
        # initialize the variable used to indicate if the thread should
        # be stopped
        self.daemon = daemon
        self.stopped = False
        # start the thread to read frames from the video stream
        threading.Thread(name=self.name, target=self.update,daemon=self.daemon, args=()).start()
        return

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped is False:
                
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
                            self.frame = cv2.rectangle(self.frame,(xg,yg),(xg+wg, yg+hg),self.color[self.whichColor],2)
                            self.frame = cv2.line(self.frame,(self.center[0],self.center[1]),(xg+(wg//2),yg+(hg//2)),self.color[self.whichColor],2)
                    if self.imwrite:
                        if self.out is None:
                            self.out = cv2.VideoWriter(self.videoOutput,cv2.VideoWriter_fourcc(*'DIVX'), 11, self.frame.shape[:2][::-1],1)
                        self.out.write(self.frame)
                    
                    if self.imshow:
                        cv2.imshow("Drone Cam", self.frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

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
        # indicate that the thread should be stopped
        self.stopped = True
    
    def finish(self):
        self.stop()
        if self.imwrite:
            self.stream.release()
        if self.imshow():
            cv2.destroyAllWindows()
        self.out.release()
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