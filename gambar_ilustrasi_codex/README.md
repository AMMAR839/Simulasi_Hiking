# Gambar Ilustrasi Codex untuk Bab 10

Folder ini berisi visualisasi ulang Bab 10 yang dibuat sebagai ilustrasi laporan, bukan screenshot simulator. Gambar dibuat dengan Matplotlib melalui `generate_codex_figures.py` agar dapat diregenerasi dengan gaya visual yang konsisten.

| File | Kegunaan di Bab 10 |
|---|---|
| `codex_fig10_01_architecture.png` | Arsitektur alur data ROS 2 dan Gazebo. |
| `codex_fig10_02_topology.png` | Topologi konseptual LoRa multi-hop dari summit ke basecamp. |
| `codex_fig10_03_environment_map.png` | Peta terrain, rute, obstacle, relay node, dan base station. |
| `codex_fig10_04_los_diffraction.png` | Ilustrasi LOS, terrain shadow, dan diffraction loss. |
| `codex_fig10_05_network_3d.png` | Visualisasi isometrik terrain, rute, dan node LoRa. |
| `codex_fig10_06_gps_pipeline.png` | Alur pembentukan GPS sintetis. |
| `codex_fig10_07_battery_tx.png` | Hubungan SoC baterai dan effective TX power. |
| `codex_fig10_08_link_budget.png` | Waterfall komponen link budget. |
| `codex_fig10_09_routing_multihop.png` | Graf routing multi-hop dan contoh route terpilih. |
| `codex_fig10_10_sf_tradeoff.png` | Trade-off spreading factor, sensitivitas, dan time on air. |
| `codex_fig10_11_per_pdr.png` | Model PER dan PDR end-to-end. |
| `codex_fig10_12_drop_reason_tree.png` | Mekanisme klasifikasi drop reason. |
| `codex_fig10_13_metrics_summary.png` | Ringkasan kelompok metrik evaluasi. |
| `codex_fig10_14_scenario_matrix.png` | Matriks skenario rute dan cuaca. |
| `codex_fig10_15_weather_impact.png` | Dampak cuaca terhadap parameter simulasi. |
| `codex_fig10_16_poc_pdr.png` | Ringkasan PDR model untuk PoC. |
| `codex_fig10_17_drop_reason_heatmap.png` | Heatmap penyebab kegagalan paket. |
| `codex_fig10_18_metrics_radar.png` | Radar ringkasan kelayakan PoC. |

Regenerasi semua gambar:

```bash
python3 gambar_ilustrasi_codex/generate_codex_figures.py
```
