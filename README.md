# Teknofest 2021 Aurora Uav

Su alma ve su bırakma alanlarını tanıma çalışmalarının tamamıdır.  
Kod akışı test çalışmaları için main.py ve yarışma için sualma.py dosyaları ana dosyalardır. Kamera thread (monitor.py) ile açılır ve kod akışı boyunca sürekli okuma yapar. Kamera görüntüsünden gerçek mesafelerin ölçümü bizim yazdığımız realpos.py dosyasında mevcuttur. utils.py dosyasında dronekit yazılımının verdiği yardımcı fonksiyonlar ile bizim yazdığımız kamera görüntüsünden gps koordinatı hesaplayabilmek için yardımcı fonksiyonlar bulunur.