import socket
import cv2
import struct
import numpy as np

MAX_DGRAM = 2**16

def dump_buffer(s):
    while True:
        seg, addr = s.recvfrom(MAX_DGRAM)
        print(seg[0])
        if struct.unpack("B",seg[0:1])[0] == 1:
            print("finish emptying buffer")
            break


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1',12345))
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
            s.bind(('127.0.0.1',12345))
            dat = b''
            dump_buffer(s)
try:
    main()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
    s.close()
    