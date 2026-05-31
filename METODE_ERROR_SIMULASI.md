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

### Prinsip Penggunaan Referensi

Tidak semua error pada simulasi harus memiliki satu jurnal khusus yang memberikan angka persis sama dengan nilai pada kode. Dalam simulasi ini, referensi digunakan dengan tiga tingkatan:

1. **Model standar atau referensi kuat**, yaitu rumus yang memang umum dipakai pada komunikasi radio atau GPS, seperti free-space path loss, knife-edge diffraction, Rayleigh/Rician fading, LoRa Time on Air, dan duty cycle.
2. **Model empiris atau semi-empiris**, yaitu metode yang konsepnya didukung literatur, tetapi nilai parameternya tetap perlu disesuaikan dengan skenario, seperti redaman vegetasi, hujan, GPS DOP, dan multipath.
3. **Heuristik internal simulasi**, yaitu model sederhana yang dibuat agar fenomena sistem dapat direpresentasikan secara komputasional, misalnya obstacle loss untuk batu/kawah sebagai zona abstrak, packet error rate berbasis fungsi logistik, delay hardware, clock drift, watchdog restart, dan penurunan TX power akibat baterai.

Dengan pembagian ini, simulasi tidak mengklaim bahwa semua angka adalah hasil pengukuran lapangan atau angka baku dari jurnal. Yang dipertanggungjawabkan adalah alur pemodelannya: setiap gangguan memiliki dasar fisik atau alasan teknis, sedangkan nilai parameter yang bersifat heuristik diposisikan sebagai asumsi simulasi yang dapat dikalibrasi pada pengujian lapangan.

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

$$
\begin{aligned}
d_{x,m} &= (x_1 - x_2)S \\
d_{y,m} &= (y_1 - y_2)S \\
d_{z,m} &= z_1 - z_2 \\
d_m &= \max\left(\sqrt{d_{x,m}^2 + d_{y,m}^2 + d_{z,m}^2},\ 1.0\right) \\
d_{km} &= \frac{d_m}{1000}
\end{aligned}
$$

Rumus ini dipakai sebelum menghitung FSPL, obstacle, cuaca, fading, dan semua loss berbasis jarak.

### 2. Free-Space Path Loss

$$
L_{\mathrm{FSPL}} =
32.44 + 20\log_{10}(d_{km}) + 20\log_{10}(f_{MHz})
$$

Ini mengikuti bentuk praktis free-space basic transmission loss pada Recommendation ITU-R P.525.

### 3. Deteksi Sinyal Menabrak Obstacle

Obstacle radio berbentuk lingkaran `C(cx, cy, r)`. Link radio dari `A(ax, ay)` ke `B(bx, by)` dianggap melewati obstacle jika jarak titik terdekat pada segmen garis ke pusat lingkaran lebih kecil dari radius.

$$
\begin{aligned}
\vec{AB} &= (b_x-a_x,\ b_y-a_y) \\
\vec{AC} &= (c_x-a_x,\ c_y-a_y) \\
t &= \mathrm{clamp}\left(\frac{\vec{AC}\cdot\vec{AB}}{\|\vec{AB}\|^2},\ 0,\ 1\right) \\
P &= A + t\vec{AB} \\
d_{\min} &= \sqrt{(p_x-c_x)^2 + (p_y-c_y)^2}
\end{aligned}
$$

$$
\mathrm{intersect} =
\begin{cases}
1, & d_{\min} < r \\
0, & d_{\min} \ge r
\end{cases}
$$

Jika memotong obstacle, kedalaman lintasan dihitung sebagai chord:

$$
\begin{aligned}
c_{\mathrm{world}} &= 2\sqrt{r^2-d_{\min}^2} \\
c_m &= c_{\mathrm{world}}S \\
c_{\mathrm{ref},m} &= 2rS \\
F_{\mathrm{depth}} &= \frac{c_m}{c_{\mathrm{ref},m}}
\end{aligned}
$$

### 4. Obstacle Loss untuk Pohon, Batu, Kawah, Terrain

Untuk obstacle bukan pohon:

$$
L_{\mathrm{obstacle}} = L_{\mathrm{base}}F_{\mathrm{depth}}
$$

Untuk obstacle `trees`:

$$
\epsilon_{\mathrm{wind}} \sim U(-0.05,\ 0.05)
$$

$$
L_{\mathrm{trees}} =
L_{\mathrm{base}}F_{\mathrm{depth}}F_{\mathrm{wet}}(w)(1+\epsilon_{\mathrm{wind}})
$$

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

$$
\begin{aligned}
\rho_i &= \frac{i}{N_s} \\
x_i &= x_{tx} + (x_{rx}-x_{tx})\rho_i \\
y_i &= y_{tx} + (y_{rx}-y_{tx})\rho_i \\
z_{\mathrm{LOS},i} &= z_{tx} + (z_{rx}-z_{tx})\rho_i \\
z_{\mathrm{terrain},i} &= h_{\mathrm{terrain}}(x_i,y_i)
\end{aligned}
$$

Koreksi kelengkungan bumi:

$$
\begin{aligned}
d_1 &= \rho_i d_{\mathrm{horiz}} \\
d_2 &= (1-\rho_i)d_{\mathrm{horiz}} \\
C_{\mathrm{earth}} &= \frac{d_1d_2}{2R_{\mathrm{earth}}} \\
z_{\mathrm{terrain,eff},i} &= z_{\mathrm{terrain},i} + C_{\mathrm{earth}}
\end{aligned}
$$

Clearance Fresnel heuristik:

$$
C_{\mathrm{Fresnel},i} = 8.0 + 0.012 \cdot \min(i,\ N_s-i)S
$$

Sampel dianggap terhalang jika:

$$
z_{\mathrm{terrain,eff},i} + C_{\mathrm{Fresnel},i} > z_{\mathrm{LOS},i}
$$

Loss akhirnya:

$$
L_{\mathrm{shadow}} = \min(22.0,\ 3.2N_{\mathrm{blocked}})
$$

### 6. Knife-Edge Diffraction

Jika ada titik terrain yang melewati garis LOS, kode mengambil clearance terburuk `h`:

$$
h = z_{\mathrm{terrain,eff}} - z_{\mathrm{LOS}}
$$

Panjang gelombang:

$$
\lambda = \frac{c}{f_{MHz}\cdot10^6}
$$

Parameter Fresnel-Kirchhoff:

$$
\nu = h\sqrt{\frac{2(d_1+d_2)}{\lambda d_1d_2}}
$$

Loss knife-edge dihitung dengan dua bentuk agar sesuai dengan implementasi saat ini. Untuk `nu` kecil sampai sedang, kode memakai bentuk polinomial:

$$
L_{\mathrm{ke}} =
\begin{cases}
6.02 + 9.11\nu + 1.27\nu^2, & -0.7 < \nu \le 2.4 \\
13.46 + 20\log_{10}(\nu), & \nu > 2.4 \\
0, & \nu \le -0.7
\end{cases}
$$

Pembatasan ini penting karena bentuk polinomial dapat menghasilkan nilai ribuan dB jika digunakan pada `nu` yang sangat besar. Untuk kondisi terrain yang sangat menghalangi lintasan radio, kode memakai bentuk asimtotik/logaritmik agar nilai diffraction loss tetap dapat dipakai sebagai indikator NLOS berat tanpa menjadi angka redaman yang tidak realistis. Kode kemudian menghindari double-counting dengan shadow loss:

$$
L_{\mathrm{diffraction}} = \max(0,\ L_{\mathrm{ke}} - L_{\mathrm{shadow}})
$$

### 7. Terrain Scatter Loss

$$
L_{\mathrm{terrain}} = d_{km}K_{\mathrm{terrain}}
$$

Default:

$$
K_{\mathrm{terrain}} = 2.5\ \mathrm{dB/km}
$$

Ini adalah loss tambahan berbasis jarak untuk menggambarkan sebaran/pantulan terrain yang tidak masuk langsung ke model shadow.

### 8. Weather Loss: Awan, Kabut, Hujan, Badai

Kode memakai profil cuaca:

$$
L_{\mathrm{weather}} = a_w d_{km} + n_w
$$

| weather | attn_db_km | noise_db | extra_drop_prob |
|---|---:|---:|---:|
| `clear` | 0.00 | 0.0 | 0.00 |
| `fog` | 0.00 | 0.5 | 0.00 |
| `light_rain` | 0.01 | 1.0 | 0.00 |
| `heavy_rain` | 0.05 | 3.0 | 0.05 |
| `thunderstorm` | 0.10 | 8.0 | 0.15 |

Tambahan drop cuaca:

$$
P_{\mathrm{weather\ drop}} = p_w
$$

Model ini sengaja lebih sederhana dari ITU-R P.838. P.838 memakai:

$$
\gamma_R = kR^{\alpha}
$$

dengan `R` adalah rain rate dalam mm/h. Simulasi ini tidak punya input rain rate, jadi dipakai profil `weather` yang langsung memberi redaman per kilometer dan noise.

### 9. Humidity Loss

$$
L_{\mathrm{humidity}} =
d_{km}\max\left(0,\frac{H-50}{50}\right)0.003
$$

Efek ini kecil, tetapi tetap dimasukkan agar kelembapan ekstrem bisa mempengaruhi link budget.

### 10. Temperature Noise

$$
N_T =
\begin{cases}
\min(1.5,\ 0.05|T|), & T < 0^\circ C \\
\min(1.0,\ 0.03(T-40)), & T > 40^\circ C \\
0, & 0^\circ C \le T \le 40^\circ C
\end{cases}
$$

Ini adalah model sederhana untuk menggambarkan perubahan noise/efisiensi perangkat pada temperatur ekstrem.

### 11. Link Budget Total

$$
\begin{aligned}
P_{rx} &= P_t + G_t + G_r \\
&\quad {}- L_{\mathrm{FSPL}} - L_{\mathrm{obstacle}} - L_{\mathrm{shadow}} - L_{\mathrm{diffraction}} \\
&\quad {}- L_{\mathrm{terrain}} - L_{\mathrm{weather}} - N_T - L_{\mathrm{humidity}} - L_{\mathrm{fading}}
\end{aligned}
$$

Di kode:

$$
G_t = G_r = G_{\mathrm{antenna}}
$$

### 12. Thermal Noise, SNR, dan Margin

Thermal noise floor:

$$
N_{\mathrm{dBm}} = -174 + 10\log_{10}(BW_{\mathrm{Hz}}) + NF
$$

Default kode:

$$
BW = 125000\ \mathrm{Hz},\quad NF = 6\ \mathrm{dB},\quad
N_{\mathrm{dBm}} \approx -117\ \mathrm{dBm}
$$

SNR dan margin:

$$
\begin{aligned}
SNR_{\mathrm{dB}} &= P_{rx} - N_{\mathrm{dBm}} \\
M_{\mathrm{link}} &= P_{rx} - S_{\mathrm{rx}}(SF)
\end{aligned}
$$

### 13. Fading Rayleigh

Dipakai saat link NLOS, yaitu ketika ada obstacle loss atau terrain shadow.

$$
\begin{aligned}
\sigma &= \frac{1}{\sqrt{2}} \\
I &\sim \mathcal{N}(0,\sigma) \\
Q &\sim \mathcal{N}(0,\sigma) \\
A &= \sqrt{I^2+Q^2} \\
L_{\mathrm{fading}} &= -20\log_{10}(\max(A,\ 10^{-10}))
\end{aligned}
$$

Rayleigh merepresentasikan multipath tanpa komponen LOS dominan.

### 14. Fading Rician

Dipakai saat link LOS bersih.

$$
\begin{aligned}
K &= 10^{K_{\mathrm{dB}}/10} \\
\mu &= \sqrt{\frac{K}{K+1}} \\
\sigma &= \frac{1}{\sqrt{2(K+1)}} \\
I &\sim \mathcal{N}(\mu,\sigma) \\
Q &\sim \mathcal{N}(0,\sigma) \\
A &= \sqrt{I^2+Q^2} \\
L_{\mathrm{fading}} &= -20\log_{10}(\max(A,\ 10^{-10}))
\end{aligned}
$$

Rician merepresentasikan multipath dengan satu komponen LOS dominan.

### 15. Sensitivitas Receiver Berdasarkan Spreading Factor

$$
S_{\mathrm{rx}} = S_{\mathrm{table}}(SF)
$$

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

$$
\begin{aligned}
T_{\mathrm{sym}} &= \frac{2^{SF}}{BW} \\
T_{\mathrm{preamble}} &= (8+4.25)T_{\mathrm{sym}} \\
N_{\mathrm{payload}} &=
\max\left(8,\ \left\lceil
\frac{8PL - 4SF + 28 + 16}{4SF}
\right\rceil 5\right) \\
T_{\mathrm{payload}} &= N_{\mathrm{payload}}T_{\mathrm{sym}} \\
T_{\mathrm{onair}} &= T_{\mathrm{preamble}} + T_{\mathrm{payload}}
\end{aligned}
$$

Kode menyimpan Time on Air dalam milidetik.

### 17. Packet Error Rate

$$
PER = \frac{1}{1+\exp(0.8(M_{\mathrm{link}}-2.0))}
$$

Makna praktis:

$$
M_{\mathrm{link}}\downarrow \Rightarrow PER\uparrow,\qquad
M_{\mathrm{link}}\uparrow \Rightarrow PER\downarrow
$$

Drop paket:

$$
P_{\mathrm{PER\ drop}} = PER \cdot 0.7^{H_{\mathrm{link}}-1}
$$

`H_link` adalah jumlah link pada route aktif. Faktor `0.7^(H_link-1)` digunakan pada implementasi saat ini untuk mengurangi agresivitas PER pada route multi-hop. Asumsinya, relay node memiliki posisi antena dan kestabilan perangkat yang lebih baik daripada portable node, sehingga paket multi-hop tidak langsung dihukum terlalu berat hanya karena jumlah hop bertambah. Faktor ini adalah heuristik simulasi dan dapat dikalibrasi ulang saat pengujian lapangan.

### 18. Duty Cycle

Kode memakai dasar batas duty cycle 1 persen per 1 jam. Interval pengiriman LoRa juga disesuaikan dari Time on Air agar node tidak mengirim terlalu rapat:

$$
T_{\mathrm{interval}} =
\frac{T_{\mathrm{onair}}}{D_{\mathrm{limit}}}
$$

dengan:

$$
D_{\mathrm{limit}} = 0.01
$$

Untuk pengecekan rolling window, implementasi saat ini memberi relaksasi internal untuk skenario multi-hop:

$$
D_{\mathrm{limit,mh}} = 1.5D_{\mathrm{limit}} = 0.015
$$

Relaksasi ini adalah asumsi simulasi agar relay multi-hop yang berbagi beban transmisi tidak terlalu cepat masuk kondisi `duty_cycle`. Jika digunakan untuk pembahasan regulasi, nilai acuan yang tetap perlu disebut adalah 1 persen.

$$
D_{\mathrm{used}} =
\frac{\sum_{3600s} T_{\mathrm{onair},s}}{3600}
$$

$$
\mathrm{duty\ allowed} =
\begin{cases}
1, & D_{\mathrm{used}} \le D_{\mathrm{limit,mh}} \\
0, & D_{\mathrm{used}} > D_{\mathrm{limit,mh}}
\end{cases}
$$

Jika tidak allowed:

$$
D_{\mathrm{used}} > D_{\mathrm{limit,mh}} \Rightarrow R_{\mathrm{drop}}=\mathrm{dutycycle}
$$

### 19. Channel Collision

$$
p_{\mathrm{collision}} =
p_{\mathrm{base,eff}}N_{\mathrm{nodes}}T_{\mathrm{onair},s}
$$

Default:

$$
p_{\mathrm{base}}=0.005,\qquad
p_{\mathrm{base,eff}}=0.6p_{\mathrm{base}},\qquad
N_{\mathrm{nodes}}=N_{\mathrm{relay}}+N_{\mathrm{hiker\ aktif}}
$$

Drop collision:

$$
P_{\mathrm{collision\ drop}} = p_{\mathrm{collision}}
$$

### 20. CRC, Hop Count, TTL, dan Protokol Paket

CRC error probability:

$$
p_{\mathrm{CRC}} = \max(0.005,\ 0.1PER)
$$

$$
P(\mathrm{CRC\ valid}) = 1 - p_{\mathrm{CRC}}
$$

Hop count:

$$
H = \lvert \mathrm{route}\rvert - 1
$$

$$
\mathrm{hop\ exceeded} =
\begin{cases}
1, & H > 6 \\
0, & H \le 6
\end{cases}
$$

Header overhead:

$$
B_{\mathrm{header}} = 13 + 2H
$$

Paket dibuang jika:

$$
\mathrm{discarded} =
\mathrm{hop\ exceeded}
\lor \mathrm{TTL\ expired}
\lor (\mathrm{delivered}\land \neg\mathrm{CRC\ valid})
$$

### 21. GPS TTFF Cold Start

$$
\mathrm{GPS\ status} =
\begin{cases}
\mathrm{NO\_FIX}, & t < T_{\mathrm{TTFF}} \\
\mathrm{FIX}, & t \ge T_{\mathrm{TTFF}}
\end{cases}
$$

Default:

$$
T_{\mathrm{TTFF}} = 8.0\ \mathrm{s}
$$

### 22. GPS DOP Area Hutan dan Terrain

Jika pendaki berada di dalam obstacle:

$$
F_{\mathrm{depth,GPS}} =
1 - \frac{d_{\mathrm{center}}}{r_{\mathrm{obstacle}}}
$$

DOP:

$$
DOP =
\min\left(
4.5,\ 
1.0 + \sum_{j=1}^{N_{\mathrm{obs}}}
F_{\mathrm{depth},j}K_{\mathrm{kind},j}
\right)
$$

Koefisien:

| kind | k_kind |
|---|---:|
| `trees` | 1.8 |
| `terrain` | 1.2 |
| `crater` | 0.8 |
| `rocks` | 0.5 |

### 23. GPS Multipath

Multipath dihitung untuk `rocks` dan `terrain` di sekitar pendaki:

$$
P_j =
\max\left(0,\ 1-\frac{d_j}{1.5r_j}\right)
$$

$$
M_{\mathrm{multipath}} =
\sum_{j\in\{\mathrm{rocks,terrain}\}}
P_j \cdot \sigma_{\mathrm{GPS}} \cdot 2.5
$$

### 24. GPS Noise Total

$$
\sigma_{\mathrm{eff}} = \sigma_{\mathrm{GPS}}DOP + M_{\mathrm{multipath}}
$$

$$
\begin{aligned}
e_{\mathrm{east}} &\sim \mathcal{N}(0,\sigma_{\mathrm{eff}}) \\
e_{\mathrm{north}} &\sim \mathcal{N}(0,\sigma_{\mathrm{eff}}) \\
e_{\mathrm{alt}} &\sim \mathcal{N}(0,0.5\sigma_{\mathrm{eff}})
\end{aligned}
$$

Konversi ke koordinat lokal:

$$
\begin{aligned}
x_{\mathrm{GPS}} &= x + \frac{e_{\mathrm{east}}}{S} \\
y_{\mathrm{GPS}} &= y + \frac{e_{\mathrm{north}}}{S}
\end{aligned}
$$

### 25. Kecepatan Pendaki karena Cuaca

$$
v_{\mathrm{eff}} = v_{\mathrm{base}}F_{\mathrm{weather}}
$$

| Cuaca | speed_factor |
|---|---:|
| `clear` | 1.00 |
| `fog` | 0.85 |
| `light_rain` | 0.80 |
| `heavy_rain` | 0.65 |
| `thunderstorm` | 0.50 |

Saat low-power mode:

$$
v_{\mathrm{eff,lowpower}} = 0.85v_{\mathrm{eff}}
$$

### 26. Hardware Delay dan Clock Drift

Hardware delay:

$$
\begin{aligned}
\tau_{\mathrm{sensor}} &\sim U(10,50)\ \mathrm{ms} \\
\tau_{\mathrm{process}} &\sim U(30,150)\ \mathrm{ms} \\
\tau_{\mathrm{prep}} &\sim U(5,30)\ \mathrm{ms} \\
\tau_{\mathrm{HW}} &= \tau_{\mathrm{sensor}}+\tau_{\mathrm{process}}+\tau_{\mathrm{prep}}
\end{aligned}
$$

Clock drift:

$$
\dot{\tau}_{\mathrm{drift}} \sim \mathcal{N}(0,\ 8\times10^{-5})
$$

$$
\tau_{\mathrm{drift}}(t+\Delta t) =
\tau_{\mathrm{drift}}(t) + \dot{\tau}_{\mathrm{drift}}\Delta t
$$

Watchdog restart:

$$
P_{\mathrm{watchdog\ restart}} = 0.0001
$$

### 27. Baterai dan Penurunan TX Power

Konsumsi per tick:

$$
\Delta Q_{\mathrm{mAh}} =
\frac{I_{\mathrm{tx}}T_{\mathrm{onair},s}+I_{\mathrm{idle}}T_{\mathrm{idle},s}}{3600}
$$

$$
\begin{aligned}
Q_{\mathrm{used}}(t+\Delta t) &= Q_{\mathrm{used}}(t)+\Delta Q_{\mathrm{mAh}} \\
Q_{\mathrm{remain}} &= Q_{\mathrm{capacity}}-Q_{\mathrm{used}} \\
SoC &= \frac{Q_{\mathrm{remain}}}{Q_{\mathrm{capacity}}}
\end{aligned}
$$

Tegangan model linear:

$$
V = 4.20 - (1-SoC)(4.20-3.00)
$$

Penurunan TX power saat baterai di bawah 30 persen:

$$
L_{\mathrm{battery}} =
\begin{cases}
\left(1-\frac{SoC}{0.30}\right)6.0, & SoC < 0.30 \\
0, & SoC \ge 0.30
\end{cases}
$$

$$
P_{t,\mathrm{eff}} = 17.0 - L_{\mathrm{battery}}
$$

Low-power mode:

$$
LPM =
\begin{cases}
1, & B_{\mathrm{pct}} < 20 \\
0, & B_{\mathrm{pct}} \ge 20
\end{cases}
$$

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

Rumus yang digunakan pada implementasi saat ini dibuat bertingkat:

```text
jika -0.7 < nu <= 2.4:
  diffraction_raw = 6.02 + 9.11 * nu + 1.27 * nu^2

jika nu > 2.4:
  diffraction_raw = 13.46 + 20 log10(nu)

jika nu <= -0.7:
  diffraction_raw = 0

diffraction_loss_db = max(0, diffraction_raw - shadow_loss_db)
```

Bagian polinomial dipakai untuk kondisi halangan kecil sampai sedang. Untuk `nu` yang besar, simulasi memakai bentuk logaritmik agar nilai diffraction loss tidak meledak menjadi sangat besar. Ini penting karena pada terrain pegunungan, beda tinggi antara terrain dan garis LOS bisa besar; jika seluruh kondisi dipaksa memakai polinomial, hasilnya dapat menjadi ribuan dB dan tidak lagi berguna sebagai model simulasi.

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

### 9. Routing Multi-Hop Deterministik

Routing pada versi simulasi sekarang dibuat lebih stabil dengan memisahkan dua jenis perhitungan link budget:

1. Link budget deterministik, yaitu perhitungan tanpa fading acak.
2. Link budget aktual, yaitu perhitungan dengan fading acak untuk evaluasi paket.

Pemilihan relay masuk (`entry_node`) dilakukan dari posisi hiker ke semua relay. Relay hanya dipilih jika margin deterministik hiker-ke-relay masih layak dan relay tersebut memiliki route menuju base station. Route antar relay dihitung menggunakan graph Dijkstra yang berisi base station dan seluruh relay node. Edge antar node hanya dibuat jika margin deterministik antar relay bernilai minimal 0 dB.

Bobot graph yang digunakan:

```text
cost = 100 + 0.1 * distance_m
```

Nilai 100 menjadi penalti per hop agar route tidak memakai terlalu banyak relay jika tidak perlu. Komponen jarak dipakai untuk memilih link yang lebih pendek ketika jumlah hop sama. Dengan cara ini, route tidak mudah berubah hanya karena fading sesaat. Fading tetap dihitung setelah route dipilih, lalu dipakai untuk RSSI aktual, margin aktual, SNR, dan peluang paket gagal.

Pada kondisi tertentu, simulasi memberi fallback untuk link hiker-ke-entry relay dengan margin antara -3 dB sampai 0 dB. Fallback ini hanya berlaku pada link pertama dari hiker ke relay, bukan pada seluruh link relay-to-relay. Tujuannya agar hiker yang berada di batas cakupan masih dapat diuji sebagai kondisi marginal, tetapi backbone antar relay tetap dipilih dari link yang lebih stabil.

### 10. Receiver Sensitivity dan Spreading Factor

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

### 11. Packet Error Rate

PER dihitung dari margin memakai fungsi logistik:

```text
PER = 1 / (1 + exp(0.8 * (margin_db - 2.0)))
```

Maknanya:

- Margin sekitar 0 dB: peluang error tinggi.
- Margin sekitar 6 dB: peluang error jauh lebih kecil.
- Margin sekitar 10 dB: peluang error rendah.

Pada route multi-hop, implementasi saat ini mengurangi agresivitas PER dengan faktor:

```text
PER_drop = PER * 0.7^(hop_count - 1)
```

Faktor ini adalah heuristik simulasi. Alasannya, relay node diasumsikan memiliki posisi antena dan pemasangan yang lebih stabil dibanding portable node, sehingga route multi-hop tidak langsung dihukum terlalu berat hanya karena jumlah hop bertambah. Untuk laporan, bagian yang kuat secara teori adalah hubungan margin terhadap peluang error; faktor `0.7^(hop_count - 1)` perlu disebut sebagai asumsi simulasi yang dapat dikalibrasi pada pengujian lapangan.

Jika random number lebih kecil dari PER, paket dianggap drop dengan alasan `per_model`.

### 12. Duty Cycle

Simulasi membatasi penggunaan kanal LoRa:

```text
duty_cycle_limit = 1 persen per 3600 detik
```

Interval pengiriman paket LoRa juga dihitung dari Time on Air:

```text
tx_interval_s = time_on_air_s / duty_cycle_limit
```

Dengan batas 1 persen, paket SF9 dengan Time on Air sekitar 153 ms menghasilkan interval kirim sekitar 15,3 detik. Time on Air setiap paket tetap dicatat dalam rolling window satu jam. Pada implementasi multi-hop saat ini, budget internal untuk relay dibuat sedikit lebih longgar:

```text
multi_hop_limit = 1.5 * duty_cycle_limit
```

Artinya batas internal simulasi menjadi 1,5 persen untuk menghindari relay terlalu cepat selalu gagal `duty_cycle` saat menerima beban dari banyak hiker. Ini bukan klaim regulasi baru. Untuk pembahasan regulasi, angka yang aman tetap 1 persen; angka 1,5 persen adalah asumsi simulasi agar skenario multi-hop dapat diamati.

### 13. Channel Collision

Collision dimodelkan sebagai probabilitas:

```text
collision_prob = base_collision_prob_eff * jumlah_node * time_on_air_s
base_collision_prob_eff = 0.6 * base_collision_prob
```

Default `base_collision_prob` adalah 0.005, sehingga probabilitas efektif yang dipakai adalah 0.003. `jumlah_node` dihitung dari jumlah relay aktif ditambah jumlah hiker aktif. Semakin banyak hiker yang mengirim paket dan semakin lama Time on Air, peluang collision semakin tinggi.

### 14. Packet Protocol

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

Poin penting untuk laporan: metode error tidak harus seluruhnya berasal dari satu jurnal yang memberikan angka identik dengan kode. Untuk Capstone 1, yang perlu ditunjukkan adalah bahwa gangguan utama memiliki dasar teknis, rumus yang standar digunakan ketika tersedia, dan parameter yang belum memiliki data lapangan dinyatakan sebagai asumsi simulasi. Pada Capstone 2, parameter heuristik seperti loss batu/kawah, wet factor, PER logistik, delay hardware, dan penurunan TX power dapat dikalibrasi menggunakan data pengujian perangkat fisik.

## Peta Metode ke Referensi/Jurnal

Tabel ini menunjukkan posisi setiap metode terhadap referensi. Kolom "Status" membedakan apakah metode tersebut memakai rumus standar, model empiris, atau heuristik internal. Jika statusnya heuristik internal, artinya model tersebut tidak diklaim sebagai rumus baku dari jurnal tertentu, tetapi digunakan sebagai pendekatan komputasional agar simulasi dapat menghasilkan perilaku yang bisa dianalisis.

| Metode | Dipakai untuk | Status | Rujukan / dasar |
|---|---|---|---|
| Free-space path loss | Redaman dasar jarak radio | Standar | [R1] |
| Obstacle `trees` / vegetasi | Redaman hutan dan daun basah | Empiris / semi-empiris | [R2]; nilai `loss_db` dan `wet_factor` adalah parameter simulasi. |
| Obstacle `rocks`, `crater`, `terrain` | Redaman zona batu/kawah/ridge berbasis kedalaman lintasan | Heuristik internal | Terinspirasi konsep obstruction/clutter/excess loss; nilai dB bukan angka baku material batu/kawah. |
| Terrain shadow | Sampel terrain yang menutup LOS/Fresnel clearance | Heuristik berbasis konsep propagasi | [R3] untuk konsep LOS/diffraction; rumus `min(22, 3.2N)` adalah pendekatan internal. |
| Knife-edge diffraction | Redaman difraksi punggungan gunung | Standar / semi-empiris | [R3]; implementasi memakai bentuk bertingkat agar `nu` besar tidak menghasilkan loss tidak realistis. |
| Rain/weather attenuation | Hujan/kabut/badai sebagai redaman dan noise | Semi-empiris / heuristik | [R4] untuk konsep rain attenuation; profil `weather` adalah parameter simulasi karena tidak memakai rain rate aktual. |
| Humidity loss | Loss kecil akibat kelembapan | Heuristik internal | Parameter kecil untuk sensitivitas lingkungan; perlu kalibrasi jika dipakai sebagai prediksi fisik. |
| Temperature noise | Noise perangkat pada temperatur ekstrem | Heuristik internal | Representasi sederhana efek temperatur pada perangkat, bukan model komponen elektronik rinci. |
| Rayleigh fading | Link NLOS/multipath tanpa LOS dominan | Standar statistik kanal | [R5] |
| Rician fading | Link LOS dengan komponen dominan | Standar statistik kanal | [R5] |
| LoRa sensitivity dan Time on Air | Efek SF, BW, CR, payload terhadap durasi dan sensitivitas | Vendor / teknis LoRa | [R6] |
| Duty cycle | Batas penggunaan kanal radio | Aturan operasi / model kanal | [R8]; nilai 1 persen menjadi acuan, sedangkan 1,5 persen pada multi-hop adalah relaksasi internal simulasi. |
| Collision | Peluang tabrakan paket saat banyak node aktif | Model probabilistik sederhana | [R7], [R8] untuk konteks collision LoRaWAN; rumus probabilitas efektif di kode adalah heuristik internal. |
| Packet Error Rate | Drop probabilistik dari margin link | Heuristik internal | Fungsi logistik dipakai agar margin rendah menaikkan peluang drop; faktor multi-hop `0.7^(H-1)` adalah asumsi simulasi. |
| CRC, hop count, TTL | Error protokol paket | Heuristik internal | Digunakan untuk merepresentasikan aturan paket dan batas protokol. |
| GPS DOP | Noise GPS membesar di hutan/terrain | Konsep GNSS + heuristik | [R9]; pemetaan obstacle ke DOP adalah parameter simulasi. |
| GPS multipath | Pantulan sinyal GPS dekat batu/terrain | Konsep GNSS + heuristik | [R10]; besaran multipath di kode adalah pendekatan skenario. |
| Hardware delay, clock drift, watchdog | Delay sensor/proses dan gangguan node | Heuristik internal | Merepresentasikan ketidakidealan perangkat embedded secara sederhana. |
| Baterai dan TX power drop | Konsumsi node dan pelemahan TX saat SoC rendah | Heuristik internal | Dipakai untuk melihat dampak energi terhadap link budget; perlu data perangkat nyata untuk kalibrasi akhir. |

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
