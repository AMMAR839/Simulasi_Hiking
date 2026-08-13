# Simulasi Hiking LoRa

Workspace ini berisi simulasi ROS 2 Jazzy + Gazebo Harmonic untuk skenario hiker di area gunung. Simulasi menampilkan hiker yang berjalan di salah satu rute pendakian, membuat data GPS simulasi, menghitung koneksi LoRa ke node relay terdekat, lalu merutekan paket sampai ke base station.

![Simulasi Hiker](video_assets/simulasi_hiker.gif)

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

---

**Catatan:**
Untuk panduan instalasi, build, parameter, logika sistem, serta cara menjalankan simulasi selengkapnya, silakan merujuk ke file [TEKNIS.md](TEKNIS.md).
