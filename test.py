from monitor import *
import time 
from math import ceil



if __name__ == "__main__":
    myThread = ThreadedVideoStream(livestream=True, stream_address=('192.168.137.61',9999))
    myThread.setColor("red")

    try:
        while True:
            continue
                
    except KeyboardInterrupt:
        myThread.finish()
        