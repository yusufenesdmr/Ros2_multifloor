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

Gereksinimler
Ubuntu 22.04
ROS 2 Humble
Nav2
nav2-virtual-layer
Python 3.10+
SQLite3

Ek araçlar:

nav2-virtual-layer
SDF2MAP
Kurulum
1. Workspace oluştur
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/yusufenesdmr/Ros2_multifloor.git
git clone https://github.com/sherif1152/nav2-virtual-layer.git
2. Bağımlılıkları kur
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
3. Paketi derle
colcon build --symlink-install
source install/setup.bash
4. Python bağımlılıklarını yükle
pip install -r ~/ros2_ws/src/Ros2_multifloor/hotel_luggage_robot/requirements.txt
Harita Üretimi

Her kat için harita üretmek amacıyla SDF2MAP kullanılabilir.

Örnek adımlar:

Katın .sdf veya .world dosyasını aç
Resolution: 0.05
LIDAR Height: 0.20
occupied_thresh = 0.65
Çıktıyı şu şekilde kaydet:
hotel_luggage_robot/maps/floor_1.pgm
hotel_luggage_robot/maps/floor_1.yaml
Wormhole Veritabanı Oluşturma

Katlar arası geçiş noktaları ve asansör yaklaşım koordinatları SQLite veritabanında tutulur.

cd ~/ros2_ws/src/Ros2_multifloor/hotel_luggage_robot/database
python3 create_demo_db.py \
  --maps-dir ~/ros2_ws/src/Ros2_multifloor/hotel_luggage_robot/maps/ \
  --out hotel_wormholes.db

Not: create_demo_db.py içindeki asansör yaklaşım koordinatlarını kendi haritana göre düzenlemelisin.

Çalıştırma
Tam sistem
ros2 launch hotel_luggage_robot hotel_robot.launch.py \
  initial_map:=floor_1 \
  map_yaml:=/absolute/path/to/floor_1.yaml \
  wormhole_db:=/absolute/path/to/hotel_wormholes.db \
  maps_dir:=/absolute/path/to/maps/
Sadece navigasyon
ros2 launch hotel_luggage_robot navigation.launch.py \
  initial_map:=floor_1 \
  map_yaml:=/absolute/path/to/floor_1.yaml \
  wormhole_db:=/absolute/path/to/hotel_wormholes.db \
  maps_dir:=/absolute/path/to/maps/
Sadece algılama
ros2 launch hotel_luggage_robot detection.launch.py \
  camera_topic:=/camera/image_raw \
  lidar_topic:=/scan
Görev Gönderme

Örnek bagaj taşıma görevi:

ros2 action send_goal /hotel_task_manager/request_delivery \
  hotel_luggage_robot/action/LuggageDelivery \
  "{ task_id: 'TASK001', guest_name: 'Ahmet Yilmaz', guest_room: '305', pickup_floor: 'floor_1', pickup_x: 2.0, pickup_y: 3.0, pickup_yaw: 0.0, dropoff_floor: 'floor_3', dropoff_x: 10.5, dropoff_y: -4.2, dropoff_yaw: 1.57, luggage_count: 2, priority: 1 }"
İzleme

Robot durumunu takip etmek için:

# Aktif kat bilgisi
ros2 topic echo /hotel_robot/current_floor

# Algılanan insanlar
ros2 topic echo /detected_persons

# Görev kuyruğu
ros2 topic echo /hotel_robot/task_queue

# Asansör kapı durumu
ros2 topic echo /elevator_controller/door_state
ROS 2 Arayüzleri
Action
/hotel_task_manager/request_delivery

Yüksek seviyeli bagaj teslim görevi kabul eder.

/hotel_robot/luggage_delivery

Çok katlı navigasyon görevini yürütür.

Service
/elevator_controller/call_elevator

Asansör çağırma servisi.

Topic
/detected_persons
/person_markers
/hotel_robot/current_floor
/hotel_robot/state
/hotel_robot/task_queue
/elevator_controller/door_state
Kullanım Senaryosu
Resepsiyonda görev oluşturulur
Robot bagaj alma noktasına gider
Asansör bölgesine yönelir
Kat değiştirir
Hedef kata geçer
Odaya ulaşır
Görevi tamamlar
Gelecek Geliştirmeler
Gerçek otel PMS entegrasyonu
Web tabanlı görev paneli
Çoklu robot desteği
Gelişmiş görev planlama
Gerçek asansör donanımı entegrasyonu
İnsan yoğunluğuna göre adaptif rota planlama
