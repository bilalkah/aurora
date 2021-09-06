from monitor import *
import time 
from math import ceil



if __name__ == "__main__":
    myThread = ThreadedVideoStream(livestream=True, stream_address=('192.168.137.61',9999))
    myThread.setColor("blue")

    start = time.time()
    i = 0
    j = 0
    try:
        while True:
            print(myThread.getColor())
            for i in range(10):
                time.sleep(1)
            
            if myThread.getColor() == "blue":
                myThread.setColor("red")
            else:
                myThread.setColor("blue")
                
    except KeyboardInterrupt:
        myThread.finish()