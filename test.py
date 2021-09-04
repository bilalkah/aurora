from monitor import *
import time 




if __name__ == "__main__":
    myThread = ThreadedVideoStream()
    myThread.setColor("blue")


    start = time.time()
    blueStop = False
    while True:
        if time.time() - start > 10 and blueStop == False:
            myThread.setColor("red")
            blueStop = True
        if time.time() - start > 20:
            break

    
    myThread.finish()