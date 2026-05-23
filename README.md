# Simulasi Hiking LoRa

Workspace ini berisi simulasi ROS 2 Jazzy + Gazebo Harmonic untuk skenario hiker di area gunung. Simulasi menampilkan hiker yang berjalan di salah satu rute pendakian, membuat data GPS simulasi, menghitung koneksi LoRa ke node relay terdekat, lalu merutekan paket sampai ke base station.

## Gambaran Sistem

Simulasi dibagi menjadi tiga lapisan:

1. Gazebo world untuk tampilan gunung, jalur, pohon, batu, kawah, node LoRa, base station, dan model hiker.
2. Node hiker ROS untuk menghitung posisi hiker di rute, GPS, altitude, yaw, dan pose visual Gazebo.
3. Node jaringan LoRa untuk menghitung link budget, obstacle loss, terrain shadow loss, routing antar-node, dan status paket ke base station.

Data utama default ada di `src/hiking_lora_sim/hiking_lora_sim/scenario.py`. File ini menjadi sumber rute, posisi node, obstacle radio, dan rumus tinggi terrain. World Gazebo dibuat ulang dari data itu oleh `src/hiking_lora_sim/tools/generate_mountain_world.py`, sehingga peta visual dan logika ROS tetap sinkron.

Jika ingin skenario eksternal, file `src/hiking_lora_sim/config/routes.yaml` bisa dipakai lewat parameter `routes_file:=...`. Tanpa parameter itu, simulasi memakai data bawaan dari `scenario.py`.

## Alur Simulasi

1. User menjalankan launch dengan `trail_name`.
2. Launch memilih world sesuai rute:
   - `ridge_route` memakai `hiking_mountain_ridge_route.sdf`
   - `valley_route` memakai `hiking_mountain_valley_route.sdf`
   - `crater_route` memakai `hiking_mountain_crater_route.sdf`
3. `hiker_agent` membaca titik rute dari `TRAILS` di `scenario.py`, atau dari YAML jika `routes_file` diberikan.
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
   - `/hiker/battery`: status baterai dan effective TX power hiker
8. Jika Gazebo aktif, `hiker_agent` juga mengirim pose ke service `/world/hiking_lora_world/set_pose`, sehingga model visual `hiker` berjalan mengikuti rute yang sama dengan data ROS.
9. `lora_network` menerima `/hiker/pose` dan `/hiker/gps`, memilih node relay terdekat yang masih reachable, lalu mencari rute ke `base_station`.
10. `base_station_display` menampilkan paket GPS hiker yang berhasil diterima base station.

## Rute Hiker

Ada tiga rute:

- `ridge_route`: rute punggungan, naik melalui jalur tengah sampai area puncak.
- `valley_route`: rute lembah, lebih mengikuti sisi barat dan area rendah sebelum naik ke utara.
- `crater_route`: rute melewati area kawah lalu lanjut ke summit.

Semua rute mulai dari sekitar base camp `(-104, -94)` dan berakhir di sekitar summit `(91, 81)`. Jalur visual di Gazebo dibuat sebagai deretan box tipis di atas terrain, sedangkan posisi hiker dihitung dari titik-titik rute di `scenario.py`.

## Kode Yang Digunakan

Bagian ini menjelaskan file apa saja yang dipakai oleh simulasi dan perannya.

### File runtime utama

File ini berjalan saat command `ros2 launch hiking_lora_sim hiking_lora_sim.launch.py` dijalankan.

| File | Dipakai untuk | Output / efek |
| --- | --- | --- |
| `src/hiking_lora_sim/launch/hiking_lora_sim.launch.py` | Launch utama. Menentukan world Gazebo, parameter GPS, parameter LoRa, cuaca, baterai, rute, dan node yang dijalankan. | Menjalankan Gazebo, `hiker_agent`, `lora_network`, `base_station_display`, dan dashboard opsional. |
| `src/hiking_lora_sim/hiking_lora_sim/hiker_agent.py` | Simulasi pendaki. Menghitung posisi hiker di jalur, tinggi terrain, GPS simulasi, error GPS, delay hardware, baterai, dan pose visual hiker di Gazebo. | Publish `/hiker/pose`, `/hiker/gps`, `/hiker/status`, `/hiker/battery`; mengirim pose ke `/world/hiking_lora_world/set_pose`. |
| `src/hiking_lora_sim/hiking_lora_sim/lora_network.py` | Simulasi jaringan LoRa. Memilih relay terdekat, menghitung link budget, obstacle loss, terrain shadow, fading, weather loss, PER, duty cycle, collision, routing multi-hop, dan latency. | Publish `/lora/network_event`, `/base_station/hiker_location`, `/lora/markers`. |
| `src/hiking_lora_sim/hiking_lora_sim/base_station_display.py` | Tampilan log base station. Membaca paket yang berhasil sampai dan warning jika paket gagal. | Log terminal berisi GPS hiker, hop, margin, dan route relay. |
| `src/hiking_lora_sim/hiking_lora_sim/dashboard_node.py` | Dashboard terminal opsional. Aktif jika `use_dashboard:=true`. | Menampilkan statistik paket, baterai, cuaca, SF, ToA, route aktif, dan link margin. |

### File data dan konfigurasi

File ini tidak semuanya berjalan sebagai node, tetapi datanya dipakai oleh node atau launch.

| File / folder | Dipakai untuk | Catatan |
| --- | --- | --- |
| `src/hiking_lora_sim/hiking_lora_sim/scenario.py` | Sumber data default: `TRAILS`, `LORA_NODES`, `BASE_STATION`, `RADIO_OBSTACLES`, fungsi terrain, konversi GPS, dan loader YAML. | Ini sumber utama jika `routes_file` kosong. |
| `src/hiking_lora_sim/config/routes.yaml` | Konfigurasi eksternal rute, node LoRa, base station, dan obstacle radio. | Dipakai hanya jika launch diberi `routes_file:=/path/ke/routes.yaml`. |
| `src/hiking_lora_sim/config/radio_notes.yaml` | Catatan parameter radio. | Tidak dijalankan sebagai node; hanya dokumentasi konfigurasi radio. |
| `src/hiking_lora_sim/worlds/hiking_mountain_*.sdf` | World Gazebo per-rute. | Dipilih oleh launch dari `trail_name`. |
| `src/hiking_lora_sim/models/hiking_lora_terrain/` | Model terrain gunung untuk Gazebo. | Berisi `model.sdf`, `model.config`, dan mesh `mountain_terrain.dae`. |

### File generator dan packaging

| File | Dipakai untuk | Kapan dipakai |
| --- | --- | --- |
| `src/hiking_lora_sim/tools/generate_mountain_world.py` | Membuat mesh gunung, visual jalur, base camp, tower LoRa, pohon, batu, kawah, hiker awal, dan world SDF per-rute. | Dipakai manual saat ingin regenerasi peta, bukan setiap launch. |
| `src/hiking_lora_sim/setup.py` | Mendaftarkan package Python, console script, launch, world, config, dan model agar ikut ter-install oleh `colcon build`. | Dipakai saat build. |
| `src/hiking_lora_sim/package.xml` | Manifest ROS 2: dependency `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `ros_gz_sim`, dan launch. | Dipakai oleh ROS 2/colcon. |
| `src/hiking_lora_sim/setup.cfg` | Konfigurasi install script package Python. | Dipakai oleh build system. |

### Alur kode saat simulasi berjalan

```text
hiking_lora_sim.launch.py
  -> Gazebo Harmonic membuka hiking_mountain_<trail_name>.sdf
  -> hiker_agent.py menghitung posisi, GPS, baterai, dan pose Gazebo
  -> lora_network.py membaca pose/GPS lalu menghitung link LoRa dan routing
  -> base_station_display.py membaca paket yang sampai ke base station
  -> dashboard_node.py menampilkan ringkasan terminal jika use_dashboard:=true
```

Jadi kode inti yang benar-benar menggerakkan simulasi adalah `hiker_agent.py` dan `lora_network.py`. File lain mendukung world, konfigurasi, tampilan, build, atau regenerasi asset.

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

Jalankan dengan dashboard terminal:

```bash
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py use_dashboard:=true
```

Jalankan dengan konfigurasi YAML eksternal:

```bash
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py routes_file:=/home/ammar/Documents/Simulasi_Hiking/src/hiking_lora_sim/config/routes.yaml
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
ros2 topic echo /hiker/battery
ros2 topic echo /lora/network_event
ros2 topic echo /base_station/hiker_location
ros2 topic echo /lora/markers
```

Makna topic:

- `/hiker/status`: teks posisi hiker, nama rute, tinggi terrain, altitude, dan GPS.
- `/hiker/pose`: posisi lokal hiker di world Gazebo.
- `/hiker/gps`: data GPS simulasi dengan noise, TTFF cold start, DOP, dan multipath.
- `/hiker/battery`: status baterai perangkat hiker, TX count, estimasi sisa, voltage, low-power mode, dan effective TX power.
- `/lora/network_event`: event JSON berisi status delivered/drop, entry node, rute hop, link budget, obstacle, dan margin.
- `/base_station/hiker_location`: paket lokasi yang berhasil sampai ke base station.
- `/lora/markers`: marker node, hiker, dan link radio untuk visualisasi.

## Logika LoRa

`lora_network` membuat station sementara bernama `hiker` dari `/hiker/pose`. Node ini lalu:

1. Menghitung jarak 3D antara hiker dan setiap node relay.
2. Menghitung free-space path loss berdasarkan frekuensi LoRa.
3. Menambahkan obstacle loss jika garis link melewati forest, cliff, crater, atau terrain obstacle.
4. Menambahkan terrain shadow loss dan knife-edge diffraction jika terrain menghalangi garis radio.
5. Menambahkan terrain scatter loss, weather loss, humidity loss, dan efek temperature noise.
6. Menambahkan fading: Rician untuk link LOS dan Rayleigh untuk link NLOS.
7. Menghitung `rx_dbm`, `snr_db`, dan `margin_db`.
8. Memilih entry node terdekat yang `margin_db >= 0`.
9. Mencari rute relay ke `base_station` memakai graph sederhana berbobot jarak dan margin.
10. Mengecek duty cycle 1%, Packet Error Rate, collision, cuaca ekstrem, CRC, hop-count limit, dan latency.
11. Menerbitkan event. Jika rute sampai base station dan tidak drop, paket dianggap delivered.

Drop reason yang mungkin muncul di `/lora/network_event`:

- `no_route`: tidak ada relay yang reachable atau tidak ada rute ke base station.
- `duty_cycle`: batas duty cycle LoRa tercapai.
- `per_model`: paket drop karena probabilitas error dari margin link.
- `collision`: simulasi tabrakan channel.
- `weather`: drop tambahan karena hujan lebat atau badai.
- `protocol`: paket dibuang oleh aturan protokol, misalnya CRC invalid atau hop count berlebih.

## Logika GPS, Cuaca, Dan Baterai

`hiker_agent` tidak hanya mengeluarkan koordinat ideal. Node ini juga membuat perilaku yang lebih dekat ke perangkat lapangan:

1. Saat awal launch, GPS berada pada fase cold start selama `ttff_delay_s`; `/hiker/gps` publish status `NO_FIX`.
2. Setelah TTFF selesai, hiker publish GPS fix dengan noise.
3. Noise GPS dipengaruhi `gps_noise_std_m`, DOP area hutan/lembah, dan multipath dekat batu/terrain.
4. Cuaca mengubah kecepatan jalan hiker melalui `weather`; badai membuat hiker jauh lebih lambat daripada kondisi cerah.
5. Battery model menghitung pemakaian berdasarkan TX current, idle current, Time on Air, dan kapasitas.
6. Saat baterai rendah, node masuk low-power mode dan effective TX power dikurangi.
7. Data baterai dikirim ke `lora_network`, sehingga link budget bisa ikut turun saat baterai melemah.

## Parameter Utama

Parameter bisa diubah lewat launch:

- `use_gazebo`: `true` untuk membuka Gazebo, `false` untuk menjalankan ROS-only.
- `gz_args`: argumen untuk Gazebo, misalnya `-r -v 3`.
- `trail_name`: rute hiker, pilihan `ridge_route`, `valley_route`, `crater_route`.
- `hiker_speed_world_units_s`: kecepatan hiker di koordinat world Gazebo.
- `meters_per_world_unit`: skala konversi world unit ke meter untuk GPS dan radio.
- `gps_noise_std_m`: standar deviasi noise GPS.
- `ttff_delay_s`: waktu tunggu cold start GPS sebelum fix tersedia.
- `tx_power_dbm`: daya pancar LoRa.
- `spreading_factor`: LoRa SF 7 sampai 12. Semakin besar SF, sensitivitas lebih baik tetapi data rate lebih lambat dan Time on Air lebih lama.
- `fading_model`: `rayleigh`, `rician`, atau `none`.
- `rician_k_db`: parameter K-factor untuk fading Rician.
- `weather`: `clear`, `fog`, `light_rain`, `heavy_rain`, atau `thunderstorm`.
- `temperature_c`: suhu lingkungan untuk efek noise perangkat.
- `humidity_pct`: kelembapan untuk tambahan loss atmosfer.
- `battery_capacity_mah`: kapasitas baterai perangkat hiker.
- `tx_current_ma`: arus saat transmit.
- `idle_current_ma`: arus idle.
- `routes_file`: path YAML eksternal untuk override route, node LoRa, base station, dan obstacle.
- `use_dashboard`: `true` untuk menyalakan dashboard terminal.
- `dashboard_refresh_hz`: refresh rate dashboard.
- `reference_lat` dan `reference_lon`: titik referensi konversi koordinat lokal ke GPS.

Contoh:

```bash
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py trail_name:=crater_route hiker_speed_world_units_s:=1.2 tx_power_dbm:=20.0 spreading_factor:=10 weather:=light_rain
```

Catatan: sensitivitas receiver tidak diatur lewat parameter launch terpisah. Sensitivitas dihitung otomatis dari `spreading_factor` di `lora_network.py`.

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

Catatan penting:

- Perubahan `scenario.py` memerlukan regenerasi world agar tampilan Gazebo ikut berubah.
- Perubahan `routes.yaml` dipakai runtime jika `routes_file` diberikan, tetapi tidak otomatis mengubah visual world SDF.
- `trail_name` di launch saat ini dibatasi ke `ridge_route`, `valley_route`, dan `crater_route`. Jika ingin nama rute baru dari YAML, choices di launch file juga perlu diperbarui.

## Troubleshooting

Jika `valley_route` atau `crater_route` tidak terbuka, pastikan workspace sudah dibuild ulang dan terminal sudah `source install/setup.bash`.

Jika hiker tidak bergerak di Gazebo, cek log `hiker_agent`. Log normalnya berisi:

```text
Gazebo hiker visual is following <route> through /world/hiking_lora_world/set_pose.
```

Jika yang muncul hanya `Waiting for Gazebo pose service`, Gazebo belum selesai start atau launch dijalankan dengan `use_gazebo:=false`.

Jika tampilan gunung terlihat bolong dari sudut kamera tertentu, regenerasi mesh dengan generator terbaru. Mesh terrain sekarang dibuat dengan normal menghadap atas agar tidak hilang akibat backface culling.
