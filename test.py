from monitor import *
from queue import Queue
import time 
import numpy as np


frameQ = Queue(maxsize=0)
myThread = ThreadedVideoStream(q_out=frameQ)
myThread2 = ThreadedVideoSave(q_in=frameQ)
myThread.setColor("blue")


start = time.time()
while True:
    if time.time() - start > 10:
        myThread.setColor("red")
    if time.time() - start > 20:
        break

while frameQ.empty() in False:
    print("Queue bekleniyor.")