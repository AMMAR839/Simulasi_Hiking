# Simulasi Hiking LoRa

Workspace ini berisi simulasi ROS 2 Jazzy + Gazebo Harmonic untuk skenario hiker di area gunung. Simulasi menampilkan hiker yang berjalan di salah satu rute pendakian, membuat data GPS simulasi, menghitung koneksi LoRa ke node relay terdekat, lalu merutekan paket sampai ke base station.

## Gambaran Sistem

Simulasi dibagi menjadi tiga lapisan:

1. Gazebo world untuk tampilan gunung, jalur, pohon, batu, kawah, node LoRa, base station, dan model hiker.
2. Node hiker ROS untuk menghitung posisi hiker di rute, GPS, altitude, yaw, dan pose visual Gazebo.
3. Node jaringan LoRa untuk menghitung link budget, obstacle loss, terrain shadow loss, routing antar-node, dan status paket ke base station.

Data utama ada di `src/hiking_lora_sim/hiking_lora_sim/scenario.py`. File ini menjadi sumber rute, posisi node, obstacle radio, dan rumus tinggi terrain. World Gazebo dibuat ulang dari data itu oleh `src/hiking_lora_sim/tools/generate_mountain_world.py`, sehingga peta visual dan logika ROS tetap sinkron.

## Alur Simulasi

1. User menjalankan launch dengan `trail_name`.
2. Launch memilih world sesuai rute:
   - `ridge_route` memakai `hiking_mountain_ridge_route.sdf`
   - `valley_route` memakai `hiking_mountain_valley_route.sdf`
   - `crater_route` memakai `hiking_mountain_crater_route.sdf`
3. `hiker_agent` membaca titik rute dari `TRAILS`.
4. Setiap timer, `hiker_agent` menghitung jarak tempuh:

```text
distance = elapsed_time * speed_world_units_s
```

5. Fungsi `point_on_trail()` mencari posisi `x`, `y`, dan `yaw` hiker pada segmen rute saat ini.
6. Fungsi `terrain_height_world()` memberi tinggi tanah di posisi itu.
7. `hiker_agent` menerbitkan:
   - `/hiker/pose`: posisi lokal hiker di frame `map`
   - `/hiker/gps`: latitude, longitude, altitude GPS simulasi
   - `/hiker/status`: status teks untuk debug
8. Jika Gazebo aktif, `hiker_agent` juga mengirim pose ke service `/world/hiking_lora_world/set_pose`, sehingga model visual `hiker` berjalan mengikuti rute yang sama dengan data ROS.
9. `lora_network` menerima `/hiker/pose` dan `/hiker/gps`, memilih node relay terdekat yang masih reachable, lalu mencari rute ke `base_station`.
10. `base_station_display` menampilkan paket GPS hiker yang berhasil diterima base station.

## Rute Hiker

Ada tiga rute:

- `ridge_route`: rute punggungan, naik melalui jalur tengah sampai area puncak.
- `valley_route`: rute lembah, lebih mengikuti sisi barat dan area rendah sebelum naik ke utara.
- `crater_route`: rute melewati area kawah lalu lanjut ke summit.

Semua rute mulai dari sekitar base camp `(-104, -94)` dan berakhir di sekitar summit `(91, 81)`. Jalur visual di Gazebo dibuat sebagai deretan box tipis di atas terrain, sedangkan posisi hiker dihitung dari titik-titik rute di `scenario.py`.

## Komponen File

- `src/hiking_lora_sim/hiking_lora_sim/scenario.py`: konfigurasi skenario, daftar rute, node LoRa, obstacle radio, dan fungsi tinggi terrain.
- `src/hiking_lora_sim/tools/generate_mountain_world.py`: generator mesh gunung dan world SDF per-rute.
- `src/hiking_lora_sim/models/hiking_lora_terrain`: model terrain gunung, termasuk mesh `mountain_terrain.dae`.
- `src/hiking_lora_sim/worlds/hiking_mountain_*.sdf`: world Gazebo hasil generator.
- `src/hiking_lora_sim/hiking_lora_sim/hiker_agent.py`: node posisi hiker, GPS, status, dan kontrol pose visual Gazebo.
- `src/hiking_lora_sim/hiking_lora_sim/lora_network.py`: node simulasi jaringan LoRa dan marker visual.
- `src/hiking_lora_sim/hiking_lora_sim/base_station_display.py`: node tampilan log paket yang sampai ke base station.
- `src/hiking_lora_sim/launch/hiking_lora_sim.launch.py`: launch utama untuk Gazebo dan semua node ROS.

## Build

```bash
cd /home/ammar/Documents/Simulasi_Hiking
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Menjalankan Simulasi

Jalankan default `ridge_route`:

```bash
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py
```

Pilih rute:

```bash
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py trail_name:=ridge_route
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py trail_name:=valley_route
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py trail_name:=crater_route
```

Jalankan tanpa Gazebo, hanya ROS dan simulasi jaringan:

```bash
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py use_gazebo:=false
```

Jalankan Gazebo server saja untuk debug headless:

```bash
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py trail_name:=ridge_route gz_args:='-r -s -v 2'
```

## Topic Penting

```bash
ros2 topic echo /hiker/status
ros2 topic echo /hiker/pose
ros2 topic echo /hiker/gps
ros2 topic echo /lora/network_event
ros2 topic echo /base_station/hiker_location
ros2 topic echo /lora/markers
```

Makna topic:

- `/hiker/status`: teks posisi hiker, nama rute, tinggi terrain, altitude, dan GPS.
- `/hiker/pose`: posisi lokal hiker di world Gazebo.
- `/hiker/gps`: data GPS simulasi dengan noise.
- `/lora/network_event`: event JSON berisi status delivered/drop, entry node, rute hop, link budget, obstacle, dan margin.
- `/base_station/hiker_location`: paket lokasi yang berhasil sampai ke base station.
- `/lora/markers`: marker node, hiker, dan link radio untuk visualisasi.

## Logika LoRa

`lora_network` membuat station sementara bernama `hiker` dari `/hiker/pose`. Node ini lalu:

1. Menghitung jarak 3D antara hiker dan setiap node relay.
2. Menghitung free-space path loss berdasarkan frekuensi LoRa.
3. Menambahkan obstacle loss jika garis link melewati forest, cliff, crater, atau terrain obstacle.
4. Menambahkan terrain shadow loss jika tinggi terrain menghalangi garis radio.
5. Menghitung `rx_dbm` dan `margin_db`.
6. Memilih entry node terdekat yang `margin_db >= 0`.
7. Mencari rute relay ke `base_station` memakai graph sederhana berbobot jarak dan margin.
8. Menerbitkan event. Jika rute sampai base station, paket dianggap delivered.

## Parameter Utama

Parameter bisa diubah lewat launch:

- `trail_name`: rute hiker, pilihan `ridge_route`, `valley_route`, `crater_route`.
- `hiker_speed_world_units_s`: kecepatan hiker di koordinat world Gazebo.
- `meters_per_world_unit`: skala konversi world unit ke meter untuk GPS dan radio.
- `gps_noise_std_m`: standar deviasi noise GPS.
- `tx_power_dbm`: daya pancar LoRa.
- `receiver_sensitivity_dbm`: sensitivitas minimum penerima.
- `reference_lat` dan `reference_lon`: titik referensi konversi koordinat lokal ke GPS.

Contoh:

```bash
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py trail_name:=crater_route hiker_speed_world_units_s:=1.2 tx_power_dbm:=20.0 receiver_sensitivity_dbm:=-129.0
```

## Regenerasi Peta

Jika rute, posisi node, obstacle, atau rumus terrain di `scenario.py` diubah, jalankan:

```bash
python3 src/hiking_lora_sim/tools/generate_mountain_world.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Generator akan memperbarui:

- `models/hiking_lora_terrain/meshes/mountain_terrain.dae`
- `worlds/hiking_mountain.sdf`
- `worlds/hiking_mountain_ridge_route.sdf`
- `worlds/hiking_mountain_valley_route.sdf`
- `worlds/hiking_mountain_crater_route.sdf`

## Troubleshooting

Jika `valley_route` atau `crater_route` tidak terbuka, pastikan workspace sudah dibuild ulang dan terminal sudah `source install/setup.bash`.

Jika hiker tidak bergerak di Gazebo, cek log `hiker_agent`. Log normalnya berisi:

```text
Gazebo hiker visual is following <route> through /world/hiking_lora_world/set_pose.
```

Jika yang muncul hanya `Waiting for Gazebo pose service`, Gazebo belum selesai start atau launch dijalankan dengan `use_gazebo:=false`.

Jika tampilan gunung terlihat bolong dari sudut kamera tertentu, regenerasi mesh dengan generator terbaru. Mesh terrain sekarang dibuat dengan normal menghadap atas agar tidak hilang akibat backface culling.
