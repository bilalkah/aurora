import threading
import cv2
import time
import queue




def Display():
    if myQ.empty() is False:
        frame = myQ.get()
        cv2.imshow('frame', frame)
        cv2.waitKey(1)

def Capture():
    if (thread1.vidcap is None):
        thread1.vidcap_open()
    else:
        ret, frame = vidcap.read()
        if ret:
            return frame
        else:
            return None



class myTread(threading.Thread):
    def __init__(self,threadID, name,target):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.target = target
        self.time = time.time()
        self.vidcap = None
        
    def run(self):
        print("Starting " + self.name)
        self.target()
        if self.life():
            if self.vidcap is not None:
                self.vidcap.release()
            if self.vidcap is None:
                cv2.destroyAllWindows()
            this.join()
        
    def life(self):
        if time.time() - self.time > 30:
            return True
        return False

    def vidcap_open(self):
        if self.vidcap is None:
            self.vidcap = cv2.VideoCapture(0)

if __name__ == '__main__':
    
    myQ = queue.SimpleQueue()
    thread1 = myTread(1, "Thread-1", Capture)
    thread2 = myTread(2, "Thread-2", Display)

    thread1.start()
    thread2.start()

