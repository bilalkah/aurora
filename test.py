from monitor import *
import time 




if __name__ == "__main__":
    myThread = ThreadedVideoStream(livestream=True, stream_address=('10.76.134.247',9999))
    myThread.setColor("blue")

    start = time.time()
    i = 10

    try:
        while True:
            if  (time.time() - start) > 10:
                myThread.setColor("red")
                
    except KeyboardInterrupt:
        myThread.finish()