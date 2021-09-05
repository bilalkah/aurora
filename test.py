from monitor import *
import time 




if __name__ == "__main__":
    myThread = ThreadedVideoStream(imwrite=True)
    myThread.setColor("blue")


    start = time.time()
    i = 5
    j = 10
    try:
        while True:
            if time.time() - start > i:
                myThread.setColor("red")
            elif time.time() - start > j:
                myThread.setColor("blue")
                i +=10
                j +=10
    except KeyboardInterrupt:
        myThread.finish()