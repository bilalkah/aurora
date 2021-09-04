# Threading for monitor camera
import threading
import cv2

class ThreadedVideoStream:
    def __init__(self, src=0, name="ThreadedVideoStream"):
        # initialize the video camera stream and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        (self.grabbed, self.frame) = self.stream.read()
        # initialize the thread name
        self.name = name
        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False
        # start the thread to read frames from the video stream
        threading.Thread(name=self.name, target=self.update, args=()).start()
        return

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()
            self.frame = cv2.flip(self.frame, 1)
            cv2.imshow("dronecam", self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        return

    def read(self):
        # return the frame most recently read
        return (self.grabbed,self.frame)

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
