# autonomous_driving

Dieses Paket startet die Gazebo-Simulation für den **Aeris**-Roboter und bereitet die Sensor-Daten so auf, dass LIO-SAM (SLAM) direkt damit arbeiten kann. Es bildet die Grundlage für die spätere Nav2-Trajektorienplanung.

---

## Simulation starten

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch autonomous_driving sim_Aeris.launch.py
```

Es öffnen sich automatisch **Gazebo** (Sonoma-Außenwelt) und **RViz2**.

---

## Fahrzeug steuern

Das Gazebo-Fenster enthält rechts ein eingebettetes **Teleop-Panel**. Dort lassen sich Vorwärtsgeschwindigkeit und Drehrate per Schieberegler einstellen und das Fahrzeug per Klick fahren.

Empfohlene Startwerte im Panel:

| Parameter | Wert      |
|-----------|-----------|
| Forward   | 10.0 m/s  |
| Yaw       | 5.0 rad/s |

Das Panel sendet die Befehle direkt auf `/cmd_vel`, von wo sie über den ROS–Gazebo-Bridge das AckermannSteering-Plugin im Fahrzeugmodell erreichen.

---

## Wie die Komponenten zusammenhängen

Gazebo und LIO-SAM sind an einer Stelle nicht direkt kompatibel. Eine Adapter-Node schließt diese Lücke:

### `ouster_timestamp_relay` — Per-Punkt-Zeitstempel für LIO-SAM

**Problem:** Das echte Ouster-LiDAR liefert neben dem Header-Timestamp der gesamten Punktwolke zusätzlich ein `t`-Feld pro Punkt — einen relativen Zeitstempel in Nanosekunden relativ zum Scan-Start. Da sich der Sensor dreht, werden verschiedene Punkte zu leicht unterschiedlichen Zeitpunkten erfasst. LIO-SAM nutzt diese Per-Punkt-Offsets um Bewegungsverzerrungen im Scan zu korrigieren (Deskewing). Gazebos `gpu_lidar`-Plugin liefert dieses Feld nicht — ohne `t` verwirft LIO-SAM jeden einzelnen Scan.

**Lösung:** Dieser Node berechnet `t` synthetisch aus dem Azimutwinkel jedes Punktes (Offset in ns relativ zum Scan-Start) und hängt das Feld an die Punktwolke an.

**Warum keine weiteren Adapter-Nodes:** Zwei ursprünglich vorhandene Nodes (`gazebo_odom_tf`, `imu_world_orientation_relay`) wurden nach Analyse entfernt, da sie auf falschen Annahmen beruhten und selbst Probleme verursachten:

- `gazebo_odom_tf` publizierte denselben `odom → chassis` Transform den LIO-SAM selbst publiziert → TF-Konflikt
- `imu_world_orientation_relay` war ein Workaround für `<localization>WORLD</localization>` im SDF — dieser Wert ist in gz-sensors8 ungültig; der korrekte Wert `ENU` liefert die IMU-Orientierung direkt im Weltframe

Details: [ADR-002](../../docs/adr/ADR_002_gazebo_liosam_hilfsnode_analyse.md)

---

## Konfigurationsdateien

| Datei | Zweck |
|-------|-------|
| `config/params_sim.yaml` | LIO-SAM-Parameter (Topics, Frames, Sensor-Konfiguration) |
| `config/robot_sim.urdf.xacro` | URDF-Robotermodell für die Simulation |
| `config/rviz2_sim_Aeris.rviz` | RViz2-Layout |
| `config/gazebo_gui.config` | Gazebo-GUI (Panel-Layout, Teleop-Plugin, Kamerapose) |
| `worlds/Sonoma_world.sdf` | Gazebo-Simulationswelt; IMU mit `<localization>ENU</localization>` |

---

## Topics (Übersicht)

| Topic | Typ | Richtung |
|-------|-----|----------|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo → LIO-SAM |
| `/ouster/points` | `sensor_msgs/PointCloud2` | Gazebo → ROS |
| `/ouster/points_timestamped` | `sensor_msgs/PointCloud2` | Adapter → LIO-SAM |
| `/model/prius_hybrid/odometry` | `nav_msgs/Odometry` | Gazebo → ROS |

---

## Installation

### Voraussetzungen

- **Ubuntu 24.04** Noble
- **ROS2 Jazzy** (nicht Humble — Bridge-API und Paketnamen unterscheiden sich)
- **Gazebo Harmonic 8.14** (zwingend — `<localization>ENU</localization>` ist gz-sensors8-spezifisch)

### Abhängigkeiten installieren

```bash
# Gazebo Harmonic
sudo apt install gz-harmonic
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge

# ROS2-Pakete
sudo apt install \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros \
  ros-jazzy-rviz2 \
  ros-jazzy-nav-msgs \
  ros-jazzy-sensor-msgs

# Python
sudo apt install python3-numpy
```

### Package einbinden

```bash
# In den src/-Ordner des Workspaces legen
cd ~/ros2_ws/src
git clone <bundle-oder-repo-url> autonomous_driving

# LIO-SAM ebenfalls in src/ einrichten (separat)

# Workspace bauen
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

> **Hinweis:** LIO-SAM muss separat installiert und gebaut werden. Das Package erwartet LIO-SAM im selben Workspace unter `src/LIO-SAM/`. Die LIO-SAM-Parameter für die Simulation liegen in `config/params_sim.yaml` dieses Packages.

---

## Nav2-Trajektorienplanung (in Entwicklung)

Langfristiges Ziel ist eine vollständige autonome Navigation mit Nav2. Noch ausstehende Komponenten:

| Komponente | Rolle in Nav2 | Status |
|---|---|---|
| LIO-SAM Odometrie | Lokalisierung (`map → odom` TF) | zu integrieren |
| LIO-SAM Map | Statische Karte für globale Planung | zu integrieren |
| Costmap-Package | Hinderniserkennung (Costmap2D) | noch nicht erstellt |
| Nav2 Planner Server | Globale Pfadplanung | zu konfigurieren |
| Nav2 Controller Server | Lokale Trajektorie (MPPI) | zu konfigurieren |

Hintergründe zur Architekturentscheidung: [ADR-001](../../docs/adr/ADR_001_trajektorienplanung_pipeline_projektstart.md)
