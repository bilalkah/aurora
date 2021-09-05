import cv2
import numpy as np
import urllib.request
import time
from math import ceil
from monitor import *



myThread = ThreadedVideoStream()
myThread.setColor("blue")
def streamAndProcess(source, grid=False, size=1):
    stream = urllib.request.urlopen(source)
    myImage = ""
    firstIter = True
    bytes = b''
    display_handle=display(None, display_id=True)

    try:
        start = time.time()
        k = 5
        l = 10
        while True:
            if time.time() - start > k:
                myThread.setColor("red")
            elif time.time() - start > l:
                myThread.setColor("blue")
                k += 10
                l += 10
            bytes += stream.read(1024)
            a = bytes.find(b'\xff\xd8') #frame starting 
            b = bytes.find(b'\xff\xd9') #frame ending
            if a != -1 and b != -1:
                jpg = bytes[a:b+2]
                bytes = bytes[b+2:]
                """
                img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                drawPictureCoor(img)
                drawLineToObj(img,obj)
                size0 = calcReal2(img.shape[1], obj[0], size)
                size1 = calcReal(img.shape[0], obj[1], size)
                drawRealPos(img, obj, [size0,size1])
                if(grid):
                    draw_grid(img)
                """
                success , img = myThread.read()
                if success:
                    encIm = cv2.imencode(".jpeg", img)[1]
                    asd = Image(data=encIm.tobytes())
                    display_handle.update(asd)
            
            print("Geçen süre: %.4f" % (time.time() - start))
    except KeyboardInterrupt:
        pass
    finally:
        display_handle.update(None)
                                
streamAndProcess('http://192.168.137.61:8000/stream.mjpg', size=2)