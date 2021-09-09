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

def calcSizePixel(width, height, altitude = 10):
    s_width = 0.00376 # m
    s_height = 0.00274 # m
    focal = 0.00360 # m
    
    size = 2.5
    
    totalX = s_width * altitude / focal 
    xsize = width * size / totalX
    
    
    totalY = s_height * altitude/ focal 
    ysize = height * size / totalY
    
    return xsize*ysize
    

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
    (obj_start_x, obj_start_y, 
     obj_width, obj_height,
     center_x, center_y
    ) = loc
    s_width = 0.00376 # m
    s_height = 0.00274 # m
    focal = 0.00360 # m
    
    width = center_x * 2
    height = center_y * 2
    
    img_center_x = obj_start_x + obj_width//2
    img_center_y = obj_start_y + obj_height//2
    
    # X için gerçek uzaklık hesabı
    # Resmin koordinatından orta noktanın koordinatına geçiş
    u = img_center_x - center_x # px = px - px
    # Pixelden sensör üzerindeki gerçek boyut dönüşümü
    x_sensor = s_width * u / width # m = m * px / px
    # Sensör üzerindeki boyuttan üçgen benzerliği ile gerçek boyuta dönüşüm
    x_real = x_sensor * altitude / focal  # m = m * m / m
    
    
    # Y için gerçek uzaklık hesabı
    # Resmin koordinatından orta noktanın koordinatına geçiş
    v = center_y - img_center_y # px = px - px
    # Pixelden sensör üzerindeki gerçek boyut dönüşümü
    y_sensor = s_height * v / height # m = m * px / px
    # Sensör üzerindeki boyuttan üçgen benzerliği ile gerçek boyuta dönüşüm
    y_real = y_sensor * altitude / focal # m = m * m / m
    
    return (x_real, y_real)
