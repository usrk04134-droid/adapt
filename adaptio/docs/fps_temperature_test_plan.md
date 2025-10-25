# FPS and Temperature Test Plan

Goal: quantify improvement from dynamic horizontal FOV (≈2000x500) vs default (3500x2500 and 3500x500).

Environment:
- Basler a2A4096-9gmPRO
- Same scene, stable lighting, same exposure/gain
- Network tuning per docs/requirements.md

Procedure:
1. Baseline full FOV (3500x2500):
   - Start scanner (NORMAL sensitivity)
   - Record FPS over 30s (image_received_count delta / time)
   - Record camera temps via existing metrics: `basler_camera_temperature`, `basler_camera_max_temperature`
   - Record CPU temp: `cat /sys/class/thermal/thermal_zone*/temp` (external)
2. Vertical crop only (3500x500):
   - Force scanner to hold horizontal full width; allow current vertical logic to minimize height
   - Record FPS and temps as above for 30s
3. Vertical + horizontal crop (~2000x500):
   - Allow new horizontal ROI logic
   - Record FPS and temps for 30s

Metrics:
- FPS: average and p10/p90
- Camera temperature (C): average and max
- CPU temperature (C): average and max

Acceptance:
- FPS improvement target: ≥2.0x vs full FOV and ≥1.3x vs vertical-only
- Temps must remain within manufacturer limits; no thermal throttling
