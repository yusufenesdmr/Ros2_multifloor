# Ros2_multifloor

ROS 2 tabanlı çok katlı otel bagaj taşıma robotu.  
Bu proje, bir otel ortamında robotun resepsiyondan odalara bagaj taşımasını, asansör kullanarak kat değiştirmesini ve insan algılama ile güvenli şekilde gezinmesini hedefler.

## Özellikler

- Çok katlı navigasyon
- Asansör tabanlı kat geçişi
- Nav2 ile otonom hareket
- YOLOv8 + LiDAR ile insan algılama
- Dinamik engel yönetimi (`nav2-virtual-layer`)
- SQLite tabanlı katlar arası geçiş / wormhole veritabanı
- Yüksek seviyeli görev yönetimi
- RViz ile görselleştirme
- Simülasyon ve gerçek sistem entegrasyonuna uygun yapı

---

## Proje Yapısı

```bash
Ros2_multifloor/
├── docs/
├── hotel_luggage_robot/
│   ├── action/
│   ├── config/
│   ├── database/
│   ├── launch/
│   ├── maps/
│   ├── msg/
│   ├── rviz/
│   ├── scripts/
│   ├── src/
│   ├── srv/
│   ├── urdf/
│   ├── worlds/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── requirements.txt
│   └── SETUP.md
└── LICENSE
