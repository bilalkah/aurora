from monitor import *
from queue import Queue
import time 

frameQ = Queue(maxsize=0)
myThread = ThreadedVideoStream(q_out=frameQ)
myThread2 = ThreadedVideoSave(q_in=frameQ)

start = time.time()
while True:
    if time.time() - start > 10:
        break
