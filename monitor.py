# Threading for monitor camera
import threading
import cv2

class ThreadedVideoStream:
    def __init__(self, src=0, q_out=None, daemon = True,name="ThreadedVideoStream"):
        # initialize the video camera stream and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        (self.grabbed, self.frame) = self.stream.read()
        self.q_out = q_out
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
            if self.stopped:
                return
            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()
            if self.grabbed:
                self.frame = cv2.flip(self.frame, 1)
                self.q_out.put(self.frame)
        return

    def read(self):
        # return the frame most recently read
        return (self.grabbed,self.frame)

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
    
    def __exit__(self):
        self.stop()
        print("Exiting ThreadedVideoStream")
        self.stream.release()
        threading.currentThread().join(1)

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
        threading.Thread(name=self.name, target=self.update,daemon=self.daemon, args=()).start()
        return
    
    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            # otherwise, read the next frame from the stream
            if self.q_in.empty() is False:
                self.frame = self.q_in.get()
                if self.out is None:
                    self.out = cv2.VideoWriter(self.videoOutput,cv2.VideoWriter_fourcc(*'DIVX'), 15, self.frame.shape[:2][::-1],1)
                self.out.write(self.frame)
        return

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
    
    
    def __exit__(self):
        while self.q_in.empty() is False:
            self.out.write(self.q_in.get())
        self.stop()
        print("Exiting ThreadedVideoSave")
        self.out.release()
        threading.currentThread().join(1)