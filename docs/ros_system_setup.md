# Robocat ROS - Setup i comandes

Aquest document recull les comandes principals per configurar i arrancar el sistema ROS actual.

## 1) Build del workspace
```
cd ~/robocat_ws
colcon build --symlink-install
source install/setup.bash
```

## 2) Llançar el sistema complet
```
ros2 launch robocat_bringup robocat.launch.py
```

## 3) Pairing i telemetria (ja integrat)
- El pairing es fa sol en arrencar amb `pairing_node`.
- La telemetria es publica via `web_telemetry_node`.

## 4) WebRTC (robot)
### 4.1 Node de signaling
Ja inclos a `robocat.launch.py`:
- `webrtc_signaling_node` amb config a `robocat_bringup/config/webrtc.yaml`

### 4.2 Video estable amb rpicam + v4l2loopback
**Carrega el loopback:**
```
sudo modprobe -r v4l2loopback
sudo modprobe v4l2loopback video_nr=2 card_label=robocat_cam exclusive_caps=1 max_buffers=2
```

**Pipeline rpicam -> /dev/video2 (YUYV):**
```
rpicam-vid -t 0 --codec mjpeg --width 1640 --height 1232 --framerate 15 -o - \
| ffmpeg -loglevel error -re -i - \
  -f v4l2 -pix_fmt yuyv422 -s 640x480 -r 15 \
  -vcodec rawvideo /dev/video2
```

**Config ROS per llegir /dev/video2:**
```
webrtc_signaling_node:
  ros__parameters:
    camera_device: "/dev/video2"
    camera_pixel_format: "yuyv422"
    camera_width: 640
    camera_height: 480
    camera_fps: 15
```

### 4.3 Quality presets (opcional)
Millor qualitat (si la xarxa aguanta):
```
rpicam-vid -t 0 --codec mjpeg --width 1920 --height 1080 --framerate 20 -o - \
| ffmpeg -loglevel error -re -i - \
  -f v4l2 -pix_fmt yuyv422 -s 1280x720 -r 20 \
  -vcodec rawvideo /dev/video2
```

## 5) Servei systemd (camera pipeline)
### 5.1 Script
`/usr/local/bin/robocat_cam.sh`
```
#!/usr/bin/env bash
set -e

modprobe v4l2loopback video_nr=2 card_label=robocat_cam exclusive_caps=1 max_buffers=2

rpicam-vid -t 0 --codec mjpeg --width 1640 --height 1232 --framerate 15 -o - \
| ffmpeg -loglevel error -re -i - \
  -f v4l2 -pix_fmt yuyv422 -s 640x480 -r 15 \
  -vcodec rawvideo /dev/video2
```

### 5.2 Servei
`/etc/systemd/system/robocat-cam.service`
```
[Unit]
Description=Robocat camera pipeline (rpicam -> v4l2loopback)
After=network.target

[Service]
ExecStart=/usr/local/bin/robocat_cam.sh
Restart=always
RestartSec=2
User=root
Group=video

[Install]
WantedBy=multi-user.target
```

### 5.3 Activar servei
```
sudo chmod +x /usr/local/bin/robocat_cam.sh
sudo systemctl daemon-reload
sudo systemctl enable --now robocat-cam.service
sudo systemctl status robocat-cam.service
```

## 6) Audio (speaker + mic)
### 6.1 Dependencies
```
sudo apt install -y alsa-utils
sudo apt install -y python3-vosk python3-sounddevice
```

### 6.2 Model Vosk (Català)
```
mkdir -p ~/.vosk
cd ~/.vosk
wget -O vosk-ca.zip https://alphacephei.com/vosk/models/vosk-model-small-ca-0.22.zip
unzip vosk-ca.zip
mv vosk-model-small-ca-0.22 model
```

### 6.3 Activar micro (offline)
`robocat_bringup/config/audio.yaml`:
```
mic_node:
  ros__parameters:
    enabled: true
    stt_backend: "vosk"
    vosk_model_path: "/home/robocat-v2/.vosk/model"
```

### 6.4 Provar speaker
```
ros2 topic pub /audio/play_file std_msgs/String "data: 'cute_1_clean.wav'"
ros2 topic pub /audio/say std_msgs/String "data: 'Hola, soc el Robocat'"
ros2 topic pub /audio/emotion std_msgs/String "data: 'happy'"
```

### 6.5 Diagnòstic ALSA (si no sona)
```
aplay -l
speaker-test -t wav -c 2
aplay /home/robocat-v2/robocat_ws/src/Robocat-ROS/robocat_hw/assets/sounds_clean/cute_1_clean.wav
```
Per I2S (MAX98357) el dispositiu surt a `aplay -l` com:
```
card 2: sndrpihifiberry [snd_rpi_hifiberry_dac], device 0: ...
```
Per això l’ALSA device correcte és:
```
audio_device: "plughw:2,0"
```

## 7) Logs utils
```
ros2 topic echo /robot_pairing/state
ros2 topic echo /audio/heard
ros2 topic echo /audio/wake
```
