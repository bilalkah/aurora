import socket
import cv2
import struct
import numpy as np

MAX_DGRAM = 2**16
adr = '192.168.43.233'
def dump_buffer(s):
    while True:
        seg, addr = s.recvfrom(MAX_DGRAM)
        print(seg[0])
        if struct.unpack("B",seg[0:1])[0] == 1:
            print("finish emptying buffer")
            break

def process(img):
    imgnew = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #mat = imgnew[440:480,:,:] 
    mat = imgnew[440:480,:,:] 

    l = np.array([0,0,0])
    u = np.array([360,150,150])
    avg = mat.mean(axis=0).mean(axis=0)
    
    if(avg[0] > l[0] and avg[0] < u[1] and
    avg[1] > l[1] and avg[1] < u[1] and
    avg[2] > l[2] and avg[2] < u[2]):
        cv2.putText(img, "!!! SU BULUNDU !!!", (img.shape[0]//2-200, img.shape[1]//2-80), cv2.FONT_HERSHEY_SIMPLEX, 2, 
                (0,255,0), 3, cv2.LINE_AA, False)
    else:
        cv2.putText(img, "SU ARANIYOR", (img.shape[0]//2-120, img.shape[1]//2-80), cv2.FONT_HERSHEY_SIMPLEX, 2, 
                (0,250,220), 2, cv2.LINE_AA, False)

    mat1 = img[440:470,:,:] 
    avg1 = mat1.mean(axis=0).mean(axis=0)
    img = cv2.rectangle(img, (0,440), (640,480), (0,0,0), -1)
    return img

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((adr,12345))
    dat = b''
    dump_buffer(s)
    while True:
        try:
            seg, addr = s.recvfrom(MAX_DGRAM)
            if struct.unpack("B",seg[0:1])[0] > 1:
                dat += seg[1:]
            else:
                dat += seg[1:]
                img = cv2.imdecode(np.frombuffer(dat, dtype=np.uint8), 1)
                if(img is not None):
                    img = process(img)
                    cv2.imshow("from drone", img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                dat = b''
        except KeyboardInterrupt:
            s.close()
            break
        except BaseException as e:    
            print("hata, "+str(e))
            s.close()
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind((adr,12345))
            dat = b''
            dump_buffer(s)
try:
    main()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
    s.close()
    