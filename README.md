# ros2wifibot — Stack ROS2 Humble pour Wifibot Lab (RPi4)

Portage ROS2 Humble du robot **Wifibot Lab** (originellement RPi2 + ROS1), piloté par une
**Raspberry Pi 4** sous **Ubuntu 22.04 Server (64 bits)**, sans Docker (installation native).

Utilisé en BTS CIEL pour les TP robotique/ROS2.

## Matériel

| Élément | Détail |
|---|---|
| Calculateur | Raspberry Pi 4 (4 Go) — Ubuntu 22.04 Server 64 bits |
| Carte motrice | dsPIC33EP du Wifibot (protocole série binaire propriétaire, 19200 bauds) |
| Lidar | YDLidar X2 (USB, adaptateur CP2102, `10c4:ea60`) |
| Caméra | Caméra USB générique (UVC) |
| Manette | PS4 DualShock4, Bluetooth (retour haptique/rumble) |
| IMU | Yahboom ICM-42670-P + QMC5883P ("YbImu"), USB, adaptateur CH340 (`1a86:7523`) |
| Capteurs IR | 2x Sharp GP2Y0A02YK, câblés sur les ADC du dsPIC33EP (adc1/adc2 = gauche, adc3/adc4 = droite) |
| Alimentation Pi | UBEC 5V/6A sur port USB-C (câble avec résistance CC 5.1kΩ requise), alimentation 5V du Wifibot conservée pour la carte moteur |

## Vue d'ensemble des paquets ROS2

```
~/ros2_ws/src/
├── ros2wifibot/            # Paquet principal : driver robot, capteurs IR, bouton d'extinction, launch
├── ydlidar_ros2_driver/    # Driver du lidar YDLidar X2 (cloné depuis GitHub, compilé depuis les sources)
└── ybimu_ros2_driver/      # Driver custom de l'IMU Yahboom (POSIX/termios, protocole propriétaire)
```

### `ros2wifibot/`
```
ros2wifibot/
├── CMakeLists.txt
├── package.xml
├── include/ros2wifibot/
│   ├── wifibot.h
│   └── libwifibot.h        # driver bas niveau dsPIC33EP (LGPL, J.C. Mammana)
├── src/
│   ├── wifibot.cpp          # node principal (odométrie, cmd_vel, /status, TF odom->base_link)
│   └── libwifibot.cpp
├── scripts/
│   ├── ir_distance_node.py  # capteurs IR -> sensor_msgs/Range + rumble manette
│   └── shutdown_button_node.py  # extinction propre sur appui long manette
├── msg/
│   └── Status.msg           # état brut renvoyé par le dsPIC (batterie, ADC, vitesses, odométrie)
├── launch/
│   └── wifibot.launch.py
└── config/
    ├── wifibot.yaml
    ├── ydlidar_x2.yaml
    ├── teleop_joy.yaml
    └── ekf.yaml
```

### `ybimu_ros2_driver/`
```
ybimu_ros2_driver/
├── CMakeLists.txt
├── package.xml
├── include/ybimu_ros2_driver/ybimu.h
├── src/
│   ├── ybimu.cpp             # driver POSIX/termios, décode le protocole trame 0x7E 0x23 ...
│   └── ybimu_node.cpp        # node ROS2 : /imu/data_raw, /imu/mag, correction de montage
├── launch/ybimu.launch.py
└── config/ybimu.yaml
```

## 1. Préparation du système (avant de copier les paquets)

### 1.1 OS
Installer **Ubuntu Server 22.04 64 bits** sur la RPi4 (via Raspberry Pi Imager). Vérifier que
l'utilisateur créé est bien dans le groupe `sudo` (bug connu de préconfiguration RPi Imager :
vérifier/corriger `/etc/group` si besoin en montant la carte SD sur un autre poste).

### 1.2 ROS2 Humble
```bash
# Suivre l'installation officielle ROS2 Humble (deb packages) pour Ubuntu 22.04
sudo apt update && sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-v4l2-camera \
  ros-humble-joy-linux \
  ros-humble-teleop-twist-joy \
  ros-humble-robot-localization \
  python3-colcon-common-extensions \
  python3-serial \
  dos2unix
```

> `ydlidar_ros2_driver` n'existe pas en paquet apt : il se compile depuis les sources (voir §3).

### 1.3 Activer l'UART GPIO et désactiver la console série (nécessaire pour parler au dsPIC)

Le port série utilisé (`/dev/ttyS0`, mini-UART exposé sur les broches GPIO14/15) doit être
activé et libéré de la console Linux.

Dans `/boot/firmware/config.txt`, ajouter :
```
enable_uart=1
```

Dans `/boot/firmware/cmdline.txt`, retirer `console=serial0,115200` (ou équivalent).

```bash
sudo systemctl mask serial-getty@ttyS0.service
sudo reboot
```

> On reste ici sur le mini-UART (`ttyS0`), pas sur le PL011 complet (`ttyAMA0`) : pas besoin de
> `dtoverlay=disable-bt` (qui swap l'UART interne du Bluetooth vers les GPIO) — cette manip est
> nécessaire sur RPi5 mais pas dans cette configuration RPi4.

### 1.4 Règles udev — noms de périphériques stables

**Important** : ne jamais référencer `/dev/ttyUSB0`/`/dev/ttyUSB1` directement dans les configs
— l'ordre d'attribution dépend de l'ordre de branchement/détection et **change** au moindre
débranchement. Utiliser systématiquement les symlinks udev stables ci-dessous.

Fichier `/etc/udev/rules.d/99-ros2-devices.rules` :
```
# YDLidar X2 (adaptateur CP2102)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ydlidar", MODE="0666"

# IMU Yahboom YbImu (adaptateur CH340)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="myimu", MODE="0666"

# Permissions générales (fallback)
SUBSYSTEM=="tty", MODE="0666"
SUBSYSTEM=="video4linux", MODE="0666"
```

Appliquer :
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Vérifier :
```bash
ls -l /dev/ydlidar /dev/myimu
```

### 1.5 Sudoers pour l'extinction par bouton manette

Fichier `/etc/sudoers.d/wifibot-shutdown` :
```
ubuntu ALL=(ALL) NOPASSWD: /sbin/shutdown
```

## 2. Récupération des paquets (git clone)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/<ton-compte>/<ton-depot>.git
```

Ça crée `~/ros2_ws/src/<ton-depot>/ros2wifibot` et `.../ybimu_ros2_driver` — `colcon build`
les trouve automatiquement (recherche récursive des `package.xml`), peu importe ce niveau de
dossier supplémentaire.

> Les fichiers Python (`ir_distance_node.py`, `shutdown_button_node.py`) doivent être en fin de
> ligne Unix (LF). Un `git clone` direct sur la RPi ne pose pas ce problème ; si tu es passé par
> une édition/copie Windows entre-temps, corrige avant build :
> ```bash
> dos2unix ~/ros2_ws/src/<ton-depot>/ros2wifibot/scripts/*.py
> chmod +x ~/ros2_ws/src/<ton-depot>/ros2wifibot/scripts/*.py
> ```

## 3. Cloner et compiler `ydlidar_ros2_driver`

```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git
# Le SDK YDLidar-SDK doit être compilé et installé au préalable (cmake && make && sudo make install)
```

## 4. Build de l'espace de travail

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 5. Configuration — points d'attention

| Fichier | Paramètre clé | Valeur |
|---|---|---|
| `ros2wifibot/config/wifibot.yaml` | `port` | `/dev/ttyS0` |
| `ros2wifibot/config/wifibot.yaml` | `publish_odom_tf` | `true` (surchargé à `false` par le launch si `use_ekf:=true`) |
| `ros2wifibot/config/ydlidar_x2.yaml` | `port` | `/dev/ydlidar` (**pas** `/dev/ttyUSB0`) |
| `ros2wifibot/config/ydlidar_x2.yaml` | `fixed_resolution` | `true` (warnings "Real points > fixed points" bénins) |
| `ybimu_ros2_driver/config/ybimu.yaml` | `port` | `/dev/myimu` (**pas** `/dev/ttyUSB1`) |
| `ybimu_ros2_driver/config/ybimu.yaml` | `mount_flip_axis` | `"y"` (dépend du montage physique — à recalibrer si le module est remonté différemment) |
| `ros2wifibot/config/teleop_joy.yaml` | `enable_button` | `4` (L1, deadman switch) |

**ROS_DOMAIN_ID** : ne rien forcer (domaine 0 par défaut), pour rester séparé d'un éventuel
autre robot (ex. TurtleBot en domaine 30) sur le même réseau. Le service systemd ci-dessous ne
doit donc **pas** définir `ROS_DOMAIN_ID`.

## 6. Test manuel

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ros2wifibot wifibot.launch.py
```

Arguments de launch disponibles :
| Argument | Défaut | Effet |
|---|---|---|
| `use_lidar` | `true` | Active le YDLidar X2 |
| `use_camera` | `true` | Active la caméra USB |
| `use_joy` | `false` | Active la manette PS4 + teleop |
| `use_imu` | `true` | Active l'IMU Yahboom |
| `use_ekf` | `true` | Active la fusion `robot_localization` (odom + IMU) |

## 7. Service systemd (démarrage automatique)

Fichier `/etc/systemd/system/wifibot.service` :
```ini
[Unit]
Description=Wifibot ROS2 bringup
After=network.target

[Service]
Type=simple
User=ubuntu
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && ros2 launch ros2wifibot wifibot.launch.py use_joy:=true'
Restart=on-failure
RestartSec=5
StartLimitIntervalSec=60
StartLimitBurst=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable wifibot.service
sudo systemctl start wifibot.service
```

Vérification :
```bash
systemctl status wifibot.service
journalctl -u wifibot.service -f
```

## 8. Dépannage rapide

| Symptôme | Cause probable | Solution |
|---|---|---|
| `Error during serial.timeout` sur le wifibot | Mauvais port, ou console série active sur l'UART | Vérifier `/boot/firmware/cmdline.txt` + `port: /dev/ttyS0` |
| Lidar se connecte mais aucune donnée / moteur ne tourne | Port USB mal identifié après rebranchement (`ttyUSB0`/`ttyUSB1` inversés) | Utiliser `/dev/ydlidar` et `/dev/myimu`, jamais `ttyUSBx` en dur |
| `Real points N > fixed points 270` | Variance naturelle de vitesse moteur du lidar | Bénin, ignorer ou `fixed_resolution: false` |
| Manette : pas de rumble | `joy_linux_node` mal configuré | Vérifier le paramètre `dev_ff` (device evdev séparé du `dev` principal, ex. `/dev/input/event3`) |
| Deux warnings "multiple authorities" sur `odom->base_link` | `wifibot_node` **et** `ekf_node` publient tous les deux la TF | `publish_odom_tf: false` piloté automatiquement par `use_ekf` dans le launch |
| `ros2 topic list` vide malgré service actif | `ROS_DOMAIN_ID` différent entre le service et le shell interactif | Ne jamais forcer `ROS_DOMAIN_ID` dans le service |
| Script Python : `/usr/bin/env: 'python3\r'` | Fin de ligne Windows (CRLF) | `dos2unix` sur le fichier puis rebuild |

## 9. Origine et attribution

`ros2wifibot` est un portage ROS2 Humble du paquet ROS1 **roswifibot** :
- Robot : [Wifibot Lab](https://www.wifibot.com/)
- Paquet ROS1 d'origine : [arnaud-ramey/roswifibot](https://github.com/arnaud-ramey/roswifibot/tree/master)
- `libwifibot.h`/`libwifibot.cpp` (driver bas niveau du protocole dsPIC33EP) : repris du paquet
  d'origine, LGPL, Jean-Charles Mammana.

Le portage vers ROS2 (rclcpp, tf2, paramètres, message `Status.msg`) ainsi que les paquets
`ybimu_ros2_driver` et les nodes `ir_distance_node.py`/`shutdown_button_node.py` sont spécifiques
à cette adaptation pour un usage pédagogique BTS CIEL.

## 10. Licence

Ce dépôt est sous licence **MIT** (voir [`LICENSE`](LICENSE)), à l'exception de
`include/ros2wifibot/libwifibot.h` et `src/libwifibot.cpp` qui restent sous **LGPL**, héritée
du paquet ROS1 d'origine (Jean-Charles Mammana).
