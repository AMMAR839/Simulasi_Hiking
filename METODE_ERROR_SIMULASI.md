# Metode Error Simulasi Hiking LoRa

Dokumen ini menjelaskan metode error yang dipakai pada simulasi Hiking LoRa. Istilah "error" di sini berarti gangguan yang membuat posisi GPS meleset, link LoRa melemah, paket terlambat, atau paket gagal sampai ke base station.

Sumber implementasi utama:

- `src/hiking_lora_sim/hiking_lora_sim/lora_network.py`
- `src/hiking_lora_sim/hiking_lora_sim/hiker_agent.py`
- `src/hiking_lora_sim/hiking_lora_sim/scenario.py`
- `src/hiking_lora_sim/config/routes.yaml`

## Ringkasan Model

Simulasi memakai dua kelompok error:

1. Error radio LoRa: path loss, obstacle, terrain, cuaca, fading, duty cycle, collision, dan packet error rate.
2. Error pendaki/GPS: GPS cold start, noise GPS, DOP, multipath, delay hardware, baterai, dan perubahan kecepatan karena cuaca.

Pohon, batu, kawah, dan punggungan gunung tidak dihitung dari setiap mesh visual satu per satu. Untuk radio, obstacle dihitung dari area lingkaran di `RADIO_OBSTACLES` atau `radio_obstacles` pada YAML. Jadi 240 pohon visual di Gazebo adalah visual hutan, sedangkan redaman radio memakai zona hutan abstrak seperti `lower_dense_forest` dan `valley_forest`.

## Rumus Lengkap Tiap Metode

Bagian ini merangkum rumus yang dipakai oleh simulasi. Beberapa rumus adalah model standar, sementara beberapa lainnya adalah penyederhanaan heuristik agar mudah dipakai pada simulasi ROS 2/Gazebo.

### Notasi

| Simbol | Arti |
|---|---|
| `S` | Skala meter per world unit, default `35.0 m/unit`. |
| `d_m` | Jarak 3D antara pemancar dan penerima dalam meter. |
| `d_km` | Jarak 3D dalam kilometer. |
| `f_MHz` | Frekuensi LoRa dalam MHz, default `915 MHz`. |
| `Pt` | TX power dalam dBm. |
| `Gt`, `Gr` | Gain antena TX dan RX dalam dB. Di kode keduanya memakai `antenna_gain_db`. |
| `rx_dbm` | Daya terima hasil link budget. |
| `margin_db` | Selisih antara `rx_dbm` dan sensitivitas receiver. |
| `N(μ, σ)` | Sampel Gaussian dengan mean `μ` dan standard deviation `σ`. |

### 1. Jarak 3D Link

Untuk dua station `A(x1, y1, z1)` dan `B(x2, y2, z2)`:

```text
dx_m = (x1 - x2) * S
dy_m = (y1 - y2) * S
dz_m = z1 - z2
d_m  = max(sqrt(dx_m^2 + dy_m^2 + dz_m^2), 1.0)
d_km = d_m / 1000
```

Rumus ini dipakai sebelum menghitung FSPL, obstacle, cuaca, fading, dan semua loss berbasis jarak.

### 2. Free-Space Path Loss

```text
FSPL_dB = 32.44 + 20 log10(d_km) + 20 log10(f_MHz)
```

Ini mengikuti bentuk praktis free-space basic transmission loss pada Recommendation ITU-R P.525.

### 3. Deteksi Sinyal Menabrak Obstacle

Obstacle radio berbentuk lingkaran `C(cx, cy, r)`. Link radio dari `A(ax, ay)` ke `B(bx, by)` dianggap melewati obstacle jika jarak titik terdekat pada segmen garis ke pusat lingkaran lebih kecil dari radius.

```text
AB = (bx - ax, by - ay)
AC = (cx - ax, cy - ay)
t  = clamp((AC . AB) / |AB|^2, 0, 1)
P  = A + t * AB
d_min = sqrt((px - cx)^2 + (py - cy)^2)

intersect = d_min < r
```

Jika memotong obstacle, kedalaman lintasan dihitung sebagai chord:

```text
chord_world = 2 * sqrt(r^2 - d_min^2)
chord_m     = chord_world * S
ref_chord_m = 2 * r * S
depth_factor = chord_m / ref_chord_m
```

### 4. Obstacle Loss untuk Pohon, Batu, Kawah, Terrain

Untuk obstacle bukan pohon:

```text
L_obstacle = loss_db * depth_factor
```

Untuk obstacle `trees`:

```text
epsilon_wind ~ Uniform(-0.05, 0.05)
L_trees = loss_db * depth_factor * wet_factor(weather) * (1 + epsilon_wind)
```

Faktor `wet_factor`:

| Cuaca | wet_factor |
|---|---:|
| `clear` | 1.00 |
| `fog` | 1.15 |
| `light_rain` | 1.40 |
| `heavy_rain` | 1.70 |
| `thunderstorm` | 2.00 |

Model ini adalah penyederhanaan dari ide attenuation in vegetation: makin dalam lintasan radio menembus vegetasi, makin besar redamannya. Faktor basah dipakai karena daun/kanopi basah biasanya memperparah redaman.

### 5. Terrain Shadow Loss

Link radio disampling pada `18` titik di antara TX dan RX. Pada setiap titik `i`, posisi dan tinggi garis LOS dihitung:

```text
ratio = i / sample_count
x_i = x_tx + (x_rx - x_tx) * ratio
y_i = y_tx + (y_rx - y_tx) * ratio
line_alt_i = z_tx + (z_rx - z_tx) * ratio
terrain_alt_i = terrain_altitude_m(x_i, y_i)
```

Koreksi kelengkungan bumi:

```text
d1 = ratio * horizontal_distance_m
d2 = (1 - ratio) * horizontal_distance_m
curvature_m = (d1 * d2) / (2 * R_earth)
terrain_alt_eff = terrain_alt_i + curvature_m
```

Clearance Fresnel heuristik:

```text
fresnel_clearance_m = 8.0 + 0.012 * min(i, sample_count - i) * S
```

Sampel dianggap terhalang jika:

```text
terrain_alt_eff + fresnel_clearance_m > line_alt_i
```

Loss akhirnya:

```text
L_shadow = min(22.0, 3.2 * obstructed_samples)
```

### 6. Knife-Edge Diffraction

Jika ada titik terrain yang melewati garis LOS, kode mengambil clearance terburuk `h`:

```text
h = terrain_alt_eff - line_alt
```

Panjang gelombang:

```text
lambda = c / (f_MHz * 10^6)
```

Parameter Fresnel-Kirchhoff:

```text
nu = h * sqrt(2 * (d1 + d2) / (lambda * d1 * d2))
```

Loss knife-edge:

```text
L_ke = 6.02 + 9.11 * nu + 1.27 * nu^2
```

Kode memakai difraksi hanya jika `nu > -0.7`, lalu menghindari double-counting dengan shadow loss:

```text
L_diffraction = max(0, L_ke - L_shadow)
```

### 7. Terrain Scatter Loss

```text
L_terrain = d_km * terrain_loss_db_per_km
```

Default:

```text
terrain_loss_db_per_km = 2.5
```

Ini adalah loss tambahan berbasis jarak untuk menggambarkan sebaran/pantulan terrain yang tidak masuk langsung ke model shadow.

### 8. Weather Loss: Awan, Kabut, Hujan, Badai

Kode memakai profil cuaca:

```text
L_weather = attn_db_km(weather) * d_km + noise_db(weather)
```

| weather | attn_db_km | noise_db | extra_drop_prob |
|---|---:|---:|---:|
| `clear` | 0.00 | 0.0 | 0.00 |
| `fog` | 0.00 | 0.5 | 0.00 |
| `light_rain` | 0.01 | 1.0 | 0.00 |
| `heavy_rain` | 0.05 | 3.0 | 0.05 |
| `thunderstorm` | 0.10 | 8.0 | 0.15 |

Tambahan drop cuaca:

```text
weather_drop = random() < extra_drop_prob(weather)
```

Model ini sengaja lebih sederhana dari ITU-R P.838. P.838 memakai:

```text
gamma_R = k * R^alpha
```

dengan `R` adalah rain rate dalam mm/h. Simulasi ini tidak punya input rain rate, jadi dipakai profil `weather` yang langsung memberi redaman per kilometer dan noise.

### 9. Humidity Loss

```text
L_humidity = d_km * max(0, (humidity_pct - 50) / 50) * 0.003
```

Efek ini kecil, tetapi tetap dimasukkan agar kelembapan ekstrem bisa mempengaruhi link budget.

### 10. Temperature Noise

```text
if temperature_c < 0:
    temp_noise_db = min(1.5, abs(temperature_c) * 0.05)
elif temperature_c > 40:
    temp_noise_db = min(1.0, (temperature_c - 40) * 0.03)
else:
    temp_noise_db = 0
```

Ini adalah model sederhana untuk menggambarkan perubahan noise/efisiensi perangkat pada temperatur ekstrem.

### 11. Link Budget Total

```text
rx_dbm =
    Pt
    + Gt + Gr
    - FSPL_dB
    - L_obstacle
    - L_shadow
    - L_diffraction
    - L_terrain
    - L_weather
    - temp_noise_db
    - L_humidity
    - L_fading
```

Di kode:

```text
Gt = Gr = antenna_gain_db
```

### 12. Thermal Noise, SNR, dan Margin

Thermal noise floor:

```text
N_dBm = -174 + 10 log10(BW_Hz) + NF
```

Default kode:

```text
BW = 125000 Hz
NF = 6 dB
N_dBm ≈ -117 dBm
```

SNR dan margin:

```text
SNR_dB = rx_dbm - N_dBm
margin_db = rx_dbm - receiver_sensitivity_dbm(SF)
```

### 13. Fading Rayleigh

Dipakai saat link NLOS, yaitu ketika ada obstacle loss atau terrain shadow.

```text
sigma = 1 / sqrt(2)
I ~ N(0, sigma)
Q ~ N(0, sigma)
A = sqrt(I^2 + Q^2)
L_fading = -20 log10(max(A, 1e-10))
```

Rayleigh merepresentasikan multipath tanpa komponen LOS dominan.

### 14. Fading Rician

Dipakai saat link LOS bersih.

```text
K = 10^(rician_k_db / 10)
mu = sqrt(K / (K + 1))
sigma = 1 / sqrt(2 * (K + 1))
I ~ N(mu, sigma)
Q ~ N(0, sigma)
A = sqrt(I^2 + Q^2)
L_fading = -20 log10(max(A, 1e-10))
```

Rician merepresentasikan multipath dengan satu komponen LOS dominan.

### 15. Sensitivitas Receiver Berdasarkan Spreading Factor

```text
receiver_sensitivity_dbm = table_sensitivity[SF]
```

| SF | Sensitivitas dBm |
|---:|---:|
| 7 | -123.0 |
| 8 | -126.0 |
| 9 | -129.0 |
| 10 | -132.0 |
| 11 | -134.5 |
| 12 | -136.0 |

Semakin besar SF, sensitivitas makin baik, tetapi Time on Air makin panjang.

### 16. LoRa Time on Air

Simulasi memakai BW 125 kHz, coding rate 4/5, explicit header, payload default 20 byte.

```text
T_sym = 2^SF / BW
T_preamble = (8 + 4.25) * T_sym
N_payload = max(8, ceil((8 * payload_bytes - 4 * SF + 28 + 16) / (4 * SF)) * 5)
T_payload = N_payload * T_sym
T_on_air = T_preamble + T_payload
```

Kode menyimpan Time on Air dalam milidetik.

### 17. Packet Error Rate

```text
PER = 1 / (1 + exp(0.8 * (margin_db - 2.0)))
```

Makna praktis:

```text
margin rendah  -> PER tinggi
margin tinggi  -> PER rendah
```

Drop paket:

```text
per_drop = random() < PER
```

### 18. Duty Cycle

Kode memakai batas 1 persen per 1 jam:

```text
duty_used = sum(T_on_air_s dalam 3600 detik terakhir) / 3600
duty_allowed = duty_used <= 0.01
```

Jika tidak allowed:

```text
drop_reason = "duty_cycle"
```

### 19. Channel Collision

```text
p_collision = base_collision_prob * n_nodes * T_on_air_s
```

Default:

```text
base_collision_prob = 0.005
n_nodes = jumlah relay + hiker
```

Drop collision:

```text
collision_drop = random() < p_collision
```

### 20. CRC, Hop Count, TTL, dan Protokol Paket

CRC error probability:

```text
p_crc_error = max(0.005, PER * 0.1)
crc_valid = random() > p_crc_error
```

Hop count:

```text
hop_count = len(route) - 1
hop_count_exceeded = hop_count > 6
```

Header overhead:

```text
header_bytes = 13 + hop_count * 2
```

Paket dibuang jika:

```text
packet_discarded =
    hop_count_exceeded
    or ttl_expired
    or (delivered and not crc_valid)
```

### 21. GPS TTFF Cold Start

```text
if elapsed_time < ttff_delay_s:
    GPS status = STATUS_NO_FIX
else:
    GPS status = STATUS_FIX
```

Default:

```text
ttff_delay_s = 8.0
```

### 22. GPS DOP Area Hutan dan Terrain

Jika pendaki berada di dalam obstacle:

```text
depth_frac = 1 - distance_to_center / obstacle_radius
```

DOP:

```text
DOP = min(4.5, 1.0 + sum(depth_frac * k_kind))
```

Koefisien:

| kind | k_kind |
|---|---:|
| `trees` | 1.8 |
| `terrain` | 1.2 |
| `crater` | 0.8 |
| `rocks` | 0.5 |

### 23. GPS Multipath

Multipath dihitung untuk `rocks` dan `terrain` di sekitar pendaki:

```text
if distance < 1.5 * obstacle_radius:
    proximity = max(0, 1 - distance / (1.5 * obstacle_radius))
    multipath_m += proximity * gps_noise_std_m * 2.5
```

### 24. GPS Noise Total

```text
effective_noise_m = gps_noise_std_m * DOP + multipath_m

noise_east  ~ N(0, effective_noise_m)
noise_north ~ N(0, effective_noise_m)
noise_alt   ~ N(0, effective_noise_m * 0.5)
```

Konversi ke koordinat lokal:

```text
noisy_x = x + noise_east / S
noisy_y = y + noise_north / S
```

### 25. Kecepatan Pendaki karena Cuaca

```text
effective_speed = base_speed * weather_speed_factor
```

| Cuaca | speed_factor |
|---|---:|
| `clear` | 1.00 |
| `fog` | 0.85 |
| `light_rain` | 0.80 |
| `heavy_rain` | 0.65 |
| `thunderstorm` | 0.50 |

Saat low-power mode:

```text
effective_speed = effective_speed * 0.85
```

### 26. Hardware Delay dan Clock Drift

Hardware delay:

```text
sensor_read_delay_ms  ~ Uniform(10, 50)
processing_delay_ms   ~ Uniform(30, 150)
packet_prep_delay_ms  ~ Uniform(5, 30)
hw_delay_ms = sensor_read_delay_ms + processing_delay_ms + packet_prep_delay_ms
```

Clock drift:

```text
clock_drift_rate ~ N(0, 8e-5)
clock_drift_s += clock_drift_rate * dt
```

Watchdog restart:

```text
node_restarted = random() < 0.0001
```

### 27. Baterai dan Penurunan TX Power

Konsumsi per tick:

```text
charge_tick_mah = (tx_current_ma * toa_s + idle_current_ma * idle_s) / 3600
charge_used_mah += charge_tick_mah
remaining_mah = battery_capacity_mah - charge_used_mah
SoC = remaining_mah / battery_capacity_mah
```

Tegangan model linear:

```text
voltage_v = 4.20 - (1 - SoC) * (4.20 - 3.00)
```

Penurunan TX power saat baterai di bawah 30 persen:

```text
if SoC < 0.30:
    tx_reduction_db = (1 - SoC / 0.30) * 6.0
    effective_tx_power_dbm = 17.0 - tx_reduction_db
else:
    effective_tx_power_dbm = 17.0
```

Low-power mode:

```text
low_power_mode = battery_percentage < 20
```

## Error Radio LoRa

### 1. Free-Space Path Loss

Setiap link LoRa dihitung dari jarak 3D antara dua station:

```text
FSPL = 32.44 + 20 log10(distance_km) + 20 log10(frequency_mhz)
```

Nilai ini menjadi redaman dasar. Default frekuensi adalah 915 MHz.

### 2. Obstacle Loss

Obstacle radio didefinisikan sebagai lingkaran:

```text
name, x, y, radius, loss_db, kind
```

Jenis obstacle yang dipakai:

| kind | Makna | Efek |
|---|---|---|
| `trees` | Area hutan/pepohonan | Redaman mengikuti kedalaman lintasan di area hutan, lalu dikalikan faktor daun basah saat kabut/hujan. |
| `rocks` | Area bebatuan/tebing | Redaman mengikuti kedalaman lintasan, tanpa faktor daun basah. |
| `crater` | Area kawah/rim kawah | Redaman mengikuti kedalaman lintasan, tanpa faktor daun basah. |
| `terrain` | Punggungan/bayangan gunung | Redaman mengikuti kedalaman lintasan, tanpa faktor daun basah. |

Metodenya:

1. Simulasi mengecek apakah garis radio dari node A ke node B memotong lingkaran obstacle.
2. Jika memotong, dihitung panjang chord atau kedalaman lintasan di dalam obstacle.
3. Redaman dihitung proporsional terhadap kedalaman tersebut.
4. Untuk `trees`, redaman dikalikan faktor daun basah dan diberi variasi angin kecil sekitar +/-5 persen.

Faktor daun basah:

| Cuaca | Faktor foliage |
|---|---:|
| `clear` | 1.00 |
| `fog` | 1.15 |
| `light_rain` | 1.40 |
| `heavy_rain` | 1.70 |
| `thunderstorm` | 2.00 |

Catatan penting: pohon visual di Gazebo tidak otomatis dihitung sebagai obstacle individual. Yang mempengaruhi radio adalah area obstacle bertipe `trees`.

### 3. Terrain Shadow Loss

Terrain shadow loss menghitung apakah permukaan gunung menghalangi garis lurus radio. Metodenya:

1. Link radio disampling pada 18 titik.
2. Pada setiap titik, sistem membandingkan tinggi garis LOS dengan tinggi terrain.
3. Ada tambahan clearance Fresnel agar link yang terlalu dekat ke permukaan tetap dianggap terganggu.
4. Setiap sampel yang terhalang menambah shadow loss.

Rumus heuristik:

```text
shadow_loss_db = min(22.0, 3.2 * jumlah_sampel_terhalang)
```

### 4. Knife-Edge Diffraction

Jika terrain benar-benar melewati garis LOS, simulasi menghitung difraksi dengan pendekatan Fresnel-Kirchhoff knife-edge.

Metodenya:

1. Cari titik terrain dengan clearance terburuk terhadap garis LOS.
2. Hitung parameter `nu` berdasarkan jarak ke pemancar, jarak ke penerima, panjang gelombang, dan tinggi halangan.
3. Hitung diffraction loss.
4. Untuk menghindari double-counting, hasil akhir memakai tambahan difraksi di atas shadow loss.

### 5. Terrain Scatter Loss

Selain terrain yang memblokir LOS, ada loss tambahan berbasis jarak:

```text
terrain_loss_db = distance_km * terrain_loss_db_per_km
```

Default `terrain_loss_db_per_km` adalah 2.5 dB/km.

### 6. Cuaca: Awan, Kabut, Hujan, Badai

Simulasi tidak membuat partikel awan/hujan sebagai model fisik radio. Cuaca dimodelkan sebagai nilai redaman dan noise tambahan pada link LoRa.

Profil cuaca:

| weather | attn_db_km | noise_db | extra_drop_prob |
|---|---:|---:|---:|
| `clear` | 0.00 | 0.0 | 0.00 |
| `fog` | 0.00 | 0.5 | 0.00 |
| `light_rain` | 0.01 | 1.0 | 0.00 |
| `heavy_rain` | 0.05 | 3.0 | 0.05 |
| `thunderstorm` | 0.10 | 8.0 | 0.15 |

Rumus weather loss:

```text
weather_loss_db = attn_db_km * distance_km + noise_db
```

Untuk `heavy_rain` dan `thunderstorm`, ada peluang paket drop tambahan melalui `extra_drop_prob`.

Dalam konteks pertanyaan "awan", model yang paling dekat adalah `fog`, yaitu kabut/awan rendah yang menambah noise 0.5 dB dan membuat foliage sedikit lebih basah. Tidak ada model awan 3D yang secara langsung memblokir LoRa.

### 7. Kelembapan dan Temperatur

Kelembapan menambah loss kecil:

```text
humidity_loss_db = distance_km * max(0, (humidity_pct - 50) / 50) * 0.003
```

Temperatur mempengaruhi noise:

| Kondisi | Efek |
|---|---|
| Di bawah 0 C | Noise naik sampai maksimum 1.5 dB. |
| Di atas 40 C | Noise naik sampai maksimum 1.0 dB. |

### 8. Fading: Rician dan Rayleigh

Simulasi memilih jenis fading berdasarkan kondisi link:

| Kondisi link | Fading |
|---|---|
| LOS, tanpa obstacle dan tanpa terrain shadow | Rician |
| NLOS, ada obstacle atau terrain shadow | Rayleigh |
| `fading_model:=none` | Fading dimatikan |

Rician dipakai untuk link yang masih punya komponen LOS kuat. Rayleigh dipakai untuk link yang lebih banyak pantulan dan tidak punya LOS bersih.

### 9. Receiver Sensitivity dan Spreading Factor

Spreading Factor mengubah sensitivitas receiver dan Time on Air.

| SF | Sensitivitas dBm | Data rate bps |
|---:|---:|---:|
| 7 | -123.0 | 5468.75 |
| 8 | -126.0 | 3125.0 |
| 9 | -129.0 | 1757.8 |
| 10 | -132.0 | 878.9 |
| 11 | -134.5 | 476.6 |
| 12 | -136.0 | 250.0 |

Margin link dihitung:

```text
margin_db = rx_dbm - receiver_sensitivity_dbm
```

Jika margin kecil, peluang error paket naik.

### 10. Packet Error Rate

PER dihitung dari margin memakai fungsi logistik:

```text
PER = 1 / (1 + exp(0.8 * (margin_db - 2.0)))
```

Maknanya:

- Margin sekitar 0 dB: peluang error tinggi.
- Margin sekitar 6 dB: peluang error jauh lebih kecil.
- Margin sekitar 10 dB: peluang error rendah.

Jika random number lebih kecil dari PER, paket dianggap drop dengan alasan `per_model`.

### 11. Duty Cycle

Simulasi membatasi penggunaan kanal LoRa:

```text
duty_cycle_limit = 1 persen per 3600 detik
```

Time on Air setiap paket dicatat. Jika penggunaan kanal melebihi batas, paket berikutnya bisa drop dengan alasan `duty_cycle`.

### 12. Channel Collision

Collision dimodelkan sebagai probabilitas:

```text
collision_prob = base_collision_prob * jumlah_node * time_on_air_s
```

Default `base_collision_prob` adalah 0.005. Semakin banyak node dan semakin lama Time on Air, peluang collision semakin tinggi.

### 13. Packet Protocol

Simulasi juga memberi error protokol:

- Sequence number.
- CRC valid atau gagal.
- Deteksi duplicate packet.
- Hop count limit.
- TTL konseptual.
- Header overhead.

Paket dibuang jika CRC gagal, hop count melebihi batas, atau TTL expired.

## Error GPS dan Pendaki

### 1. TTFF Cold Start

Pada awal simulasi, GPS belum langsung fix. Default:

```text
ttff_delay_s = 8.0
```

Selama periode ini, `/hiker/gps` dipublish dengan status `STATUS_NO_FIX`, latitude 0, longitude 0, dan altitude 0.

### 2. GPS Gaussian Noise

Setelah GPS fix, posisi diberi noise Gaussian:

```text
noise_east  ~ N(0, effective_noise_m)
noise_north ~ N(0, effective_noise_m)
```

Default `gps_noise_std_m` adalah 2.0 m.

### 3. DOP karena Hutan dan Terrain

Faktor DOP naik jika pendaki berada di area obstacle. Semakin dekat ke pusat obstacle, efeknya semakin besar.

| Obstacle | Tambahan DOP maksimum |
|---|---:|
| `trees` | 1.8 |
| `terrain` | 1.2 |
| `crater` | 0.8 |
| `rocks` | 0.5 |

DOP dibatasi maksimum 4.5.

Noise efektif:

```text
effective_noise_m = gps_noise_std_m * dop + multipath_m
```

### 4. Multipath GPS

Multipath ditambahkan saat pendaki dekat `rocks` atau `terrain`. Modelnya:

1. Cek jarak pendaki ke obstacle.
2. Jika masih dalam 1.5 kali radius obstacle, multipath ditambah.
3. Multipath makin besar saat pendaki makin dekat ke obstacle.

Ini menggambarkan pantulan sinyal GPS dari tebing, batu, atau lereng.

### 5. Cuaca Mengubah Kecepatan Pendaki

Cuaca juga mempengaruhi gerak pendaki:

| Cuaca | Faktor kecepatan |
|---|---:|
| `clear` | 1.00 |
| `fog` | 0.85 |
| `light_rain` | 0.80 |
| `heavy_rain` | 0.65 |
| `thunderstorm` | 0.50 |

Saat low-power mode aktif, kecepatan dikurangi lagi menjadi 85 persen dari kecepatan efektif.

### 6. Hardware Delay dan Clock Drift

Simulasi menambahkan delay hardware:

- Sensor read delay: 10 sampai 50 ms.
- Processing delay: 30 sampai 150 ms.
- Packet preparation delay: 5 sampai 30 ms.
- Clock drift: random kecil sekitar crystal drift.

Ada juga watchdog restart dengan probabilitas sangat kecil per tick.

### 7. Baterai dan TX Power

Baterai dihitung dari konsumsi TX dan idle:

```text
charge_tick = (tx_current_ma * toa_s + idle_current_ma * idle_s) / 3600
```

Saat baterai di bawah 20 persen, low-power mode aktif. Saat state of charge di bawah 30 persen, effective TX power dikurangi sampai maksimum 6 dB. Saat baterai habis, node berhenti aktif.

## Output yang Bisa Dicek

Data error radio keluar di topic:

```text
/lora/network_event
```

Field penting:

- `delivered`
- `drop_reason`
- `links`
- `obstacle_loss_db`
- `terrain_shadow_loss_db`
- `diffraction_loss_db`
- `terrain_loss_db`
- `weather_loss_db`
- `humidity_loss_db`
- `fading_loss_db`
- `crossed_obstacles`
- `snr_db`
- `margin_db`

Data GPS dan pendaki keluar di topic:

```text
/hiker/gps
/hiker/status
/hiker/battery
```

Field penting pada status:

- `dop`
- `gps_err`
- `drift`
- `hw_delay`
- `weather`
- `speed_factor`

## Parameter yang Bisa Diubah Saat Launch

Contoh:

```bash
ros2 launch hiking_lora_sim hiking_lora_sim.launch.py weather:=heavy_rain spreading_factor:=10 humidity_pct:=85 temperature_c:=32
```

Parameter yang relevan:

| Parameter | Fungsi |
|---|---|
| `weather` | Memilih profil cuaca: `clear`, `fog`, `light_rain`, `heavy_rain`, `thunderstorm`. |
| `spreading_factor` | Mengubah sensitivitas, data rate, Time on Air, duty cycle, dan peluang collision. |
| `fading_model` | `rayleigh`, `rician`, atau `none`. |
| `rician_k_db` | K-factor untuk Rician fading. |
| `tx_power_dbm` | Daya pancar LoRa. |
| `humidity_pct` | Kelembapan untuk loss atmosfer kecil. |
| `temperature_c` | Temperatur untuk noise tambahan. |
| `gps_noise_std_m` | Noise dasar GPS. |
| `ttff_delay_s` | Delay GPS cold start. |
| `battery_capacity_mah` | Kapasitas baterai hiker. |
| `routes_file` | File YAML eksternal untuk mengganti rute, node, base station, dan obstacle. |

## Batasan Model

Model ini cukup untuk simulasi teknis dan laporan konsep, tetapi ada batasan:

- Awan/hujan tidak dimodelkan sebagai partikel fisik 3D yang memotong sinyal.
- Pohon visual tidak dihitung satu per satu sebagai obstacle radio.
- Obstacle radio memakai area lingkaran abstrak, bukan raycast mesh Gazebo.
- Loss vegetasi, batu, dan terrain adalah model heuristik, bukan hasil pengukuran lapangan.
- Terrain shadow dan diffraction memakai sampling dan pendekatan knife-edge, bukan full wave propagation.

Dengan batasan tersebut, simulasi ini tetap bisa dipertanggungjawabkan sebagai simulasi berbasis link budget dan model probabilistik, selama dijelaskan bahwa obstacle dan cuaca adalah representasi matematis, bukan pengukuran fisik langsung.

## Peta Metode ke Referensi/Jurnal

Tabel ini menunjukkan rujukan untuk tiap kelompok metode. Jika kolom rujukan berisi "heuristik internal", artinya rumus itu dibuat sebagai pendekatan simulasi sederhana di kode, bukan klaim model standar penuh dari jurnal tertentu.

| Metode | Dipakai untuk | Rujukan |
|---|---|---|
| Free-space path loss | Redaman dasar jarak radio | [R1] |
| Obstacle `trees` / vegetasi | Redaman hutan dan daun basah | [R2] |
| Obstacle `rocks`, `crater`, `terrain` | Redaman zona batu/kawah/ridge berbasis kedalaman lintasan | Heuristik internal, terinspirasi link-budget obstruction loss |
| Terrain shadow | Sampel terrain yang menutup LOS/Fresnel clearance | [R3], heuristik internal |
| Knife-edge diffraction | Redaman difraksi punggungan gunung | [R3] |
| Rain/weather attenuation | Hujan/kabut/badai sebagai redaman dan noise | [R4], heuristik internal |
| Humidity loss | Loss kecil akibat kelembapan | Heuristik internal |
| Temperature noise | Noise perangkat pada temperatur ekstrem | Heuristik internal |
| Rayleigh fading | Link NLOS/multipath tanpa LOS dominan | [R5] |
| Rician fading | Link LOS dengan komponen dominan | [R5] |
| LoRa sensitivity dan Time on Air | Efek SF, BW, CR, payload terhadap durasi dan sensitivitas | [R6], [R7] |
| Duty cycle dan collision | Pembatasan kanal dan peluang tabrakan paket | [R7], [R8] |
| Packet Error Rate | Drop probabilistik dari margin link | Heuristik internal berbasis bentuk logistik, dipakai agar margin rendah menaikkan peluang drop |
| CRC, hop count, TTL | Error protokol paket | Heuristik internal |
| GPS DOP | Noise GPS membesar di hutan/terrain | [R9], heuristik internal |
| GPS multipath | Pantulan sinyal GPS dekat batu/terrain | [R10] |
| Hardware delay, clock drift, watchdog | Delay sensor/proses dan gangguan node | Heuristik internal |
| Baterai dan TX power drop | Konsumsi node dan pelemahan TX saat SoC rendah | Heuristik internal |

## Daftar Referensi

[R1] ITU-R, "Recommendation ITU-R P.525-5: Calculation of free-space attenuation," 2024.  
https://www.itu.int/dms_pubrec/itu-r/rec/p/R-REC-P.525-5-202411-I!!PDF-E.pdf

[R2] ITU-R, "Recommendation ITU-R P.833-8: Attenuation in vegetation," 2013.  
https://www.itu.int/dms_pubrec/itu-r/rec/p/R-REC-P.833-8-201309-S!!PDF-E.pdf

[R3] ITU-R, "Recommendation ITU-R P.526: Propagation by diffraction."  
https://www.itu.int/rec/r-rec-p.526/en

[R4] ITU-R, "Recommendation ITU-R P.838-3: Specific attenuation model for rain for use in prediction methods," 2005.  
https://www.itu.int/dms_pubrec/itu-r/rec/p/R-REC-P.838-3-200503-I!!PDF-E.pdf

[R5] S. O. Rice, "Mathematical Analysis of Random Noise," Bell System Technical Journal, vol. 24, no. 1, pp. 46-156, 1945.  
https://doi.org/10.1002/j.1538-7305.1945.tb00453.x

[R6] Semtech, "SX1272/3/6/7/8 LoRa Modem Designer's Guide," Application Note AN1200.13.  
https://www.semtech.com/uploads/documents/LoraDesignGuide_STD.pdf

[R7] N. Chinchilla-Romero, J. Navarro-Ortiz, P. Munoz, and P. Ameigeiras, "Collision Avoidance Resource Allocation for LoRaWAN," Sensors, vol. 21, no. 4, 1218, 2021.  
https://doi.org/10.3390/s21041218

[R8] F. Adelantado, X. Vilajosana, P. Tuset-Peiro, B. Martinez, J. Melia-Segui, and T. Watteyne, "Understanding the Limits of LoRaWAN," IEEE Communications Magazine, vol. 55, no. 9, pp. 34-40, 2017.  
https://doi.org/10.1109/MCOM.2017.1600613

[R9] L. Giraud, A. Ducom, S. Roche, A. F. Munoz, and L. Ries, "Deriving a Dilution of Precision Indicator for GNSS Factor Graph Optimization Solutions," Engineering Proceedings, vol. 88, no. 1, 41, 2024.  
https://www.mdpi.com/2673-4591/88/1/41

[R10] J. Huang, R. Sun, R. Yang, X. Zhan, and W. Chen, "Navigation Domain Multipath Characterization Using GNSS Direct Position Estimation in Urban Canyon Environment," Proceedings of ION GNSS+ 2022.  
https://www.ion.org/publications/pdf.cfm?articleID=18438
