# Simulasi Hiking LoRa

Workspace ini berisi simulasi ROS 2 Jazzy + Gazebo Harmonic untuk skenario hiker di area gunung. Hiker mengikuti jalur pendakian, mengambil posisi GPS simulasi, mengirim lokasi lewat LoRa ke node terdekat, lalu paket dirutekan antar-node sampai base station di base camp.

## Komponen

- `hiking_mountain_*.sdf`: world Gazebo per-rute berisi area gunung luas, terrain mesh 3D, beberapa jalur pendakian, pepohonan, bebatuan, kawah, node LoRa, base station, dan hiker yang bergerak mengikuti track.
- `models/hiking_lora_terrain`: mesh medan gunung yang dipakai oleh Gazebo sebagai visual dan collision terrain.
- `tools/generate_mountain_world.py`: generator world/terrain dari data skenario agar peta dan model ROS tetap sinkron.
- `hiker_agent`: node ROS yang mensimulasikan GPS hiker dan menerbitkan `/hiker/gps` serta `/hiker/pose`.
- `lora_network`: node ROS yang menghitung link budget LoRa, gangguan obstacle/terrain, node terdekat, routing antar-node, dan status delivery.
- `base_station_display`: node ROS yang menerima dan menampilkan lokasi hiker yang berhasil sampai ke base station.

## Build

```bash
cd /home/ammar/Documents/Simulasi_Hiking
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Menjalankan Simulasi

Dengan Gazebo:

```bash
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py
```

Memilih track hiker:

```bash
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py trail_name:=ridge_route
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py trail_name:=valley_route
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py trail_name:=crater_route
```

Tanpa Gazebo, hanya simulasi ROS/network:

```bash
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py use_gazebo:=false
```

Topic penting:

```bash
ros2 topic echo /hiker/gps
ros2 topic echo /lora/network_event
ros2 topic echo /base_station/hiker_location
ros2 topic echo /lora/markers
```

## Parameter Utama

Parameter dapat diubah lewat launch atau file launch:

- `meters_per_world_unit`: skala jarak dunia Gazebo ke meter.
- `tx_power_dbm`: daya pancar LoRa.
- `receiver_sensitivity_dbm`: sensitivitas minimum penerima.
- `frequency_mhz`: frekuensi LoRa untuk perhitungan free-space path loss.
- `gps_noise_std_m`: noise GPS hiker.
- `trail_name`: rute hiker, tersedia `ridge_route`, `valley_route`, dan `crater_route`.

Contoh:

```bash
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py tx_power_dbm:=20.0 receiver_sensitivity_dbm:=-129.0
```

## Regenerasi Peta

Jika koordinat track, node LoRa, atau formula medan diubah di `src/hiking_lora_sim/hiking_lora_sim/scenario.py`, regenerasi world dan mesh:

```bash
python3 src/hiking_lora_sim/tools/generate_mountain_world.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```
