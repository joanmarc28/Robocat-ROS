# Robocat ROS - Setup i documentacio completa

Aquest document recull tot el necessari per executar el robot amb ROS2, connectar-lo al backend i controlar-lo des de la web.

## 0) Estructura de sistema (resum)
- Robot (ROS2): envia telemetria per HTTP i rep comandes via WS.
- Backend (FastAPI): rep telemetria, guarda a BD, envia comandes via WS.
- Web: UI per canviar idioma i control de moviments.
- BD (PostgreSQL): guarda telemetria i marca ONLINE/OFFLINE.

## 1) Build del workspace ROS
```
cd ~/robocat_ws
colcon build --symlink-install
source install/setup.bash
```

## 1.1 Dependències Python actuals (venv robocat-webrtc)
Activar sempre el mateix entorn per evitar barrejar paquets de sistema i venv:
```
source /home/robocat-v2/.venvs/robocat-webrtc/bin/activate
```

Paquets base ROS build/tooling dins venv:
```
pip install --upgrade pip
pip install colcon-common-extensions catkin_pkg empy lark PyYAML
```

Paquets runtime que s'estan usant al robot (audio, vision, webrtc, hw):
```
pip install \
  requests aiortc av \
  vosk sounddevice \
  ultralytics torch==2.4.1 torchvision==0.19.1 \
  opencv-python numpy scikit-learn Pillow tensorflow keras \
  smbus2 lgpio luma.oled luma.core RPi.GPIO
```

Comprovacio rapida:
```
python -c "import ultralytics, torch, cv2, sklearn, vosk, sounddevice; print('deps ok')"
```

## 2) Llançar el sistema complet
```
ros2 launch robocat_bringup robocat.launch.py
```

## 3) Pairing i telemetria
### 3.1 Pairing (automatic)
El pairing es fa sol en arrencar amb `pairing_node`.
Config:
`robocat_bringup/config/pairing.yaml`

### 3.2 Telemetria (HTTP)
El robot publica telemetria via HTTP:
`robocat_bringup/config/telemetry.yaml`
```
telemetry_url: "https://europerobotics.jmprojects.cat/api/telemetry"
```
Node:
`robocat_hw/web_telemetry_node.py`

## 4) WebRTC (robot)
### 4.1 Node de signaling
Inclòs a `robocat.launch.py`:
- `webrtc_signaling_node` (config `robocat_bringup/config/webrtc.yaml`)

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
sudo apt install -y alsa-utils python3-venv portaudio19-dev
python3 -m venv ~/.venvs/robocat-webrtc
source ~/.venvs/robocat-webrtc/bin/activate
pip install vosk sounddevice requests aiortc av
```

### 6.2 Models Vosk (Catala + Espanol + English)
```
mkdir -p ~/.vosk
cd ~/.vosk
wget -O vosk-ca.zip https://alphacephei.com/vosk/models/vosk-model-small-ca-0.4.zip
wget -O vosk-es.zip https://alphacephei.com/vosk/models/vosk-model-small-es-0.42.zip
wget -O vosk-en.zip https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-ca.zip && mv vosk-model-small-ca-0.4 ca
unzip vosk-es.zip && mv vosk-model-small-es-0.42 es
unzip vosk-en.zip && mv vosk-model-small-en-us-0.15 en
```

### 6.3 Activar micro (offline)
`robocat_bringup/config/audio.yaml`:
```
mic_node:
  ros__parameters:
    enabled: true
    stt_backend: "vosk"
    language: "ca"
    vosk_model_path: "/home/robocat-v2/.vosk/ca"
    vosk_model_map_json: >-
      {"ca":"/home/robocat-v2/.vosk/ca","es":"/home/robocat-v2/.vosk/es","en":"/home/robocat-v2/.vosk/en"}
```

### 6.4 Canviar idioma en calent
```
ros2 service call /audio/set_language robocat_msgs/srv/SetLanguage "{language: 'es'}"
ros2 service call /audio/set_language robocat_msgs/srv/SetLanguage "{language: 'en'}"
```

### 6.5 Provar speaker
```
ros2 topic pub /audio/play_file std_msgs/String "data: 'cute_1_clean.wav'"
ros2 topic pub /audio/say std_msgs/String "data: 'Hola, soc el Robocat'"
ros2 topic pub /audio/emotion std_msgs/String "data: 'happy'"
```

### 6.6 Diagnostic ALSA (si no sona)
```
aplay -l
speaker-test -t wav -c 2
aplay /home/robocat-v2/robocat_ws/src/Robocat-ROS/robocat_hw/assets/sounds_clean/cute_1_clean.wav
```
Per I2S (MAX98357) el dispositiu surt a `aplay -l` com:
```
card 2: sndrpihifiberry [snd_rpi_hifiberry_dac], device 0: ...
```
L'ALSA device correcte:
```
audio_device: "plughw:2,0"
```

## 7) Vision AI (matricules + contenidors + emocions)
### 7.1 Llibreries Python necessaries (exactes)
Paquets base:
- ultralytics
- opencv-python (o `python3-opencv` via apt)
- numpy
- scikit-learn
- Pillow

Per OCR de matricules (CNN, opcional):
- tensorflow
- keras

Per YOLO (si no ve inclos amb ultralytics al teu entorn):
- torch
- torchvision

Versions que s'han validat al robot:
- torch==2.4.1
- torchvision==0.19.1
- ultralytics==8.3.40

Per emocions locals:
- Mode actual recomanat: OpenCV cascade (no necessita keras)
- Mode model real (opcional): tensorflow/keras o tflite-runtime

### 7.2 On son els models
S'han copiat a:
```
robocat_vision/assets/
  models/
  yolo/
  label_encoder/
```
Els scripts de `robocat_old/vision/*` ja apunten aqui.

### 7.3 Notes d'instal-lacio (Ubuntu)
Si uses apt per OpenCV:
```
sudo apt install -y python3-opencv
```
Si uses pip (recomanat dins venv):
```
pip install ultralytics==8.3.40 torch==2.4.1 torchvision==0.19.1 \
  opencv-python numpy scikit-learn Pillow tensorflow keras
```
Si falta `torch`/`torchvision`, instala'ls amb la versio compatible per aarch64.

### 7.4 Deteccio d'emocio humana (local)
`vision_node` publica `human_emotion` a `/vision/events` i caixes a `/vision/detections`.

Config actual (estable i lleugera):
```
vision_node:
  ros__parameters:
    detect_human_emotion: true
    emotion_backend: "cascade"
    emotion_interval_sec: 0.5
    emotion_cooldown_sec: 1.5
    emotion_min_confidence: 0.55
```

Backend `cascade`:
- Detecta cara (`haarcascade_frontalface_default.xml`)
- Detecta somriure (`haarcascade_smile.xml`)
- Publica `emotion=happy` si hi ha somriure, altrament `emotion=default`

Si vols millor precisio, canvia a model entrenat d'emocions (FER) i afegeix backend model (tensorflow o tflite).

## 7) I2C + servos (PCA9685)
### 7.1 Activar I2C (Ubuntu Server)
```
echo "dtparam=i2c_arm=on" | sudo tee -a /boot/firmware/config.txt
sudo modprobe i2c-dev
echo "i2c-dev" | sudo tee -a /etc/modules
sudo reboot
```

### 7.2 Detectar dispositius
```
ls /dev/i2c-*
sudo i2cdetect -y 1
```
Esperat: dispositiu a `0x40` (PCA9685).

### 7.3 Llibreries Python (system)
```
sudo apt install -y python3-rpi.gpio
sudo python3 -m pip install --break-system-packages \
  adafruit-blinka adafruit-circuitpython-pca9685 adafruit-circuitpython-motor
```

## 8) Comandes de moviment (web -> backend -> robot)
### 8.1 Backend
Endpoint nou:
```
POST /api/robots/:id/movement
Body: { "action": "endavant" | "enrere" | "rotar" | ... }
```

### 8.2 Robot
El node `ws_command_node` escolta `/ws/telemetria` i publica a:
```
/robocat/cmd
```
Configuracio:
`robocat_bringup/config/commands.yaml`

### 8.3 ROS cmd_node
`robocat_control/cmd_node.py` executa les comandes:
- endavant, enrere, rotar, maneta
- ajupir, normal, hind_sit, recte, strech, up, calibrar
- demo

## 9) WebSocket de comandes
### 9.1 Robot
`robocat_hw/ws_command_node.py`:
- WS a `/ws/telemetria`
- token via `robot_token` (query param)

### 9.2 Backend
WebSocket:
```
wss://europerobotics.jmprojects.cat/ws/telemetria
```

### 9.3 Nginx (proxy WS)
```
location /ws/telemetria {
    proxy_pass http://127.0.0.1:8013/ws/telemetria;
    proxy_http_version 1.1;
    proxy_set_header Upgrade $http_upgrade;
    proxy_set_header Connection "upgrade";
    proxy_set_header Host $host;
    proxy_read_timeout 60s;
    proxy_send_timeout 60s;
}
```

### 9.4 Backend websockets
Assegura suport WS a Uvicorn:
```
pip install websockets
```
O `uvicorn[standard]`.

## 10) BD: ONLINE/OFFLINE automàtic
### 10.1 Trigger ONLINE
El trigger posa ONLINE quan arriba telemetria.

### 10.2 OFFLINE amb pg_cron
Job `robot_mark_offline` cada 2 minuts:
```
SELECT fn_robot_mark_offline(5);
```
Config a `JMProjects.cat/backends/backend_europerobotics/bd/schema.sql`.

## 11) Logs utils
```
ros2 topic echo /robot_pairing/state
ros2 topic echo /audio/heard
ros2 topic echo /audio/wake
ros2 topic echo /robocat/cmd
```

## 12) Errors comuns
### 12.1 "ModuleNotFoundError: board"
Instal·lar `adafruit-blinka` al Python del sistema:
```
sudo python3 -m pip install --break-system-packages adafruit-blinka
```

### 12.2 WS timeout
- Comprova Nginx proxy
- Comprova que Uvicorn té websockets
- Prova: `curl -i https://europerobotics.jmprojects.cat/ws/telemetria`

### 12.3 Problemes d'entorn Python barrejat
Símptomes típics:
- `No module named ...` tot i haver fet `pip install`
- shebang dels nodes apunta a `/usr/bin/python3`

Comprovacions:
```
source /home/robocat-v2/.venvs/robocat-webrtc/bin/activate
which python
which colcon
head -n 1 install/robocat_vision/lib/robocat_vision/vision_node
```

Rebuild net amb venv:
```
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

