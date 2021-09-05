import cv2
import numpy as np
import time
from math import ceil

"""
@hi: resmin yüksekliği
@ph: cismin resimdeki bulunduğu piksel
@Z: Kameranın yere uzaklığı
"""
def calcRealY(hi, ph, Z):
    # Kamera teknik sabitleri
    hs = 0.00274 # m (2.74mm) 
    f = 0.00290 #m (3.29mm)

    # Cismin resimdeki orta noktaya göre konumu
    pv = hi/2 - ph # px

    # Cismin sensör alanındaki orta noktaya göre konumu
    ys = pv * hs / hi # m

    # Cismin gerçek konumu
    Y = ys * Z / f
    return Y

"""
@hi: resmin yüksekliği
@ph: cismin resimdeki bulunduğu piksel
@Z: Kameranın yere uzaklığı
"""
def calcRealX(hi, ph, Z):
    # Kamera teknik sabitleri
    hs = 0.00367 # m (2.74mm) 
    f = 0.00360 #m (3.29mm)

    # Cismin resimdeki orta noktaya göre konumu
    pv = ph - hi/2  # px

    # Cismin sensör alanındaki orta noktaya göre konumu
    ys = pv * hs / hi # m

    # Cismin gerçek konumu
    Y = ys * Z / f
    return Y

def drawRealPos(img, pos, size):
    w = img.shape[1]
    h = img.shape[0]
    color = (0, 255,0)
    fsize = .7
    thick = 1
    
    # X ekseni çizilir
    cv2.line(img, (ceil(w/2), ceil(h/2)), (pos[0], ceil(h/2)), color)
    cv2.putText(img, ("%.2f" % (size[0]*100)) + "cm", (pos[0], ceil(h/2)), cv2.FONT_HERSHEY_SIMPLEX, fsize, 
                 color, thick, cv2.LINE_AA, False)
    
    # Y ekseni çizilir
    cv2.line(img, (ceil(w/2), ceil(h/2)), (ceil(w/2), pos[1]), color)
    cv2.putText(img, ("%.2f" % (size[1]*100)) + "cm", (ceil(w/2),pos[1]), cv2.FONT_HERSHEY_SIMPLEX, fsize, 
                 color, thick, cv2.LINE_AA, False)


# [xmin ymin xlen ylen hc wc]

def process(loc, altitude=1):
    (xg, yg, wg, hg, cy, cx) = loc
    xg = xg + wg//2
    yg = yg + hg//2
    size0 = abs(calcRealX(cx*2, xg, altitude))
    size1 = abs(calcRealY(cy*2, yg, altitude))
    return (size0, size1)
    #drawRealPos(img, obj, [size0,size1])