from monitor import *
import time 




if __name__ == "__main__":
    myThread = ThreadedVideoStream(imwrite=True)
    myThread.setColor("blue")

    start = time.time()
    i = 10

    try:
        while True:
            if  (time.time() - start) > 10:
                myThread.setColor("blur")
                
    except KeyboardInterrupt:
        myThread.finish()