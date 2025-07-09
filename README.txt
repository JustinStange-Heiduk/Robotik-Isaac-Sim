# README – Autonomer Roboter in häuslicher Umgebung (Simulation mit Isaac Sim & ROS2)

Dieses Verzeichnis enthält alle zentralen Dateien des Projekts „KI-gesteuerter Roboter zur autonomen Navigation und Objekterkennung in einer simulierten Wohnumgebung“. Es wurde mit Isaac Sim, ROS2 Humble und Python realisiert.

## Ordner- und Dateistruktur

| Element                | Beschreibung                                                                 |
|------------------------|------------------------------------------------------------------------------|
| `Download Objects/`    | Enthält heruntergeladene 3D-Assets (z. B. Möbel) für Isaac Sim               |
| `map/`                 | Speichert die erstellte Karte (`wohnung.pgm`, `wohnung.yaml`) für Nav2       |
| `my_nav2_launch/`      | Enthält Launch-Dateien für Nav2                                              |
| `my_slam_launch/`      | Enthält Launch-Dateien für die SLAM-Initialisierung mit slam_toolbox         |
| `BSDSAP_Projekt_...`   | Projektbericht (DOCX und PDF)                                                |
| `my_nav.rviz`          | Enthält rviz Konfiguration für Lidar, Map, Nav2 und TF                       |
| `Nachweis_Praxisphase_`| Praxisnachweis						                |
| `scene_camera.usd`     | Isaac Sim Szene mit statischer Kamera für Objekterkennung                    |
| `scene_nav.usd`        | Isaac Sim Szene für die Navigation mit Sensorik und beweglichem Roboter      |
| `Wohnung_isaac_sim.png`| Isaac Sim erstellte occupancy Map der Wohnung				|
| `yolo_node/`           | Enthält das Python-Skript für die YOLOv8m-Objekterkennung                    |
| `yolov8m.pt`           | Vortrainiertes YOLOv8m-Modell (Ultralytics)                                  |

## Initiale Setup-Schritte

Vor dem Start müssen einige grundlegende Installationen und Umgebungsvariablen eingerichtet werden:

```bash
# Wichtige Systempfade setzen
chmod 0700 /run/user/1000/

# ROS2 und Workspace initialisieren
source /opt/ros/humble/setup.bash
source ~/isaac_example/IsaacSim-ros_workspaces/humble_ws/install/setup.bash

Zusätzliche Installationen
Folgende ROS2-Pakete wurden zusätzlich installiert (via rosdep oder manuell):

sudo apt update
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-cv-bridge python3-opencv

Außerdem wird ultralytics (YOLOv8) über pip benötigt:

pip install ultralytics

Wichtige Befehle zur Ausführung

1. RViz2 starten (mit festem Layout)

export RVIZ_CONFIG_FILE=/Dein Pfad/my_rviz2/my_nav.rviz
ros2 run rviz2 rviz2 -d /Dein Pfad/my_nav.rviz --ros-args --param use_sim_time:=true

2. SLAM-Kartenerstellung starten

ros2 launch my_slam_launch my_slam_toolbox_launch.py use_sim_time:=true

3. Navigation Karte starten

ros2 launch my_nav2_launch my_nav2_launch.py use_sim_time:=true

4. Karte speichern

ros2 run nav2_map_server map_saver_cli -f ~/Dein Pfad/wohnung

5. Lokalisierung (ohne erneutes Mapping)

ros2 launch nav2_bringup localization_launch.py \
  map:=/Dein Pfad/wohnung.yaml \
  use_sim_time:=true

6.YOLO-Objekterkennung starten

cd ~/Dein Pfad/yolo
python3 yolo_node.py

7. RViz2 starten (mit festem Layout)

export RVIZ_CONFIG_FILE=/Dein Pfad/my_rviz2/my_nav.rviz
ros2 run rviz2 rviz2 -d /Dein Pfad/my_nav.rviz --ros-args --param use_sim_time:=true

