from monitor import *
import time 




if __name__ == "__main__":
    myThread = ThreadedVideoStream(imwrite=True)
    myThread.setColor("blue")


    start = time.time()
    blue = False
    red = True
    try:
        while True:
            if red:
                myThread.setColor("red")
                red =False
                blue = True
            elif blue:
                myThread.setColor("blue")
                red = True
                blue = False
    except KeyboardInterrupt:
        myThread.finish()