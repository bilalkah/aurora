from monitor import *
import time 
from math import ceil



if __name__ == "__main__":
    myThread = ThreadedVideoStream(livestream=True)
    myThread.setColor("blue")
    try:
        while True:
            print("Boş durma aq")
            time.sleep(5)
            continue
                
    except KeyboardInterrupt:
        myThread.finish()
        