from monitor import *
import time 
from math import ceil



if __name__ == "__main__":
    myThread = ThreadedVideoStream(imshow=True)
    myThread.setColor("blue")
    try:
        while True:
            continue
                
    except KeyboardInterrupt:
        myThread.finish()
        