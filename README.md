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

Gazebo und LIO-SAM wurden unabhängig entwickelt und sprechen an drei Stellen nicht dieselbe Sprache. Drei kleine Adapter-Nodes schließen diese Lücken:

### 1. `gazebo_odom_tf` — TF-Transform für LIO-SAM

**Problem:** Gazebo liefert die Fahrzeugposition nur als Odometrie-Message, aber keinen TF-Transform. LIO-SAM braucht jedoch den Transform `odom → chassis` im TF-Baum, bevor es überhaupt starten kann.

**Lösung:** Dieser Node liest die Odometrie von Gazebo und broadcastet daraus kontinuierlich den TF-Transform. Sobald LIO-SAMs eigene SLAM-Schätzung stabil läuft, überschreibt sie diesen Transform automatisch.

---

### 2. `imu_world_orientation_relay` — IMU-Orientierung im Weltframe

**Problem:** Gazebo liefert die IMU-Orientierung relativ zur **Startpose des Sensors** (also immer Identität = Nullrotation bei t=0), obwohl in der SDF `WORLD`-Koordinaten konfiguriert sind. LIO-SAM liest diese Quaternion beim Start um die initiale Fahrtrichtung in der Karte zu bestimmen — mit Identität würde es immer mit Yaw = 0° starten, egal wohin das Fahrzeug tatsächlich zeigt.

**Lösung:** Dieser Node multipliziert die bekannte Spawn-Orientierung des Fahrzeugs (aus der SDF: yaw = 0.97 rad) vor die Gazebo-Quaternion:

```text
q_welt = q_spawn ⊗ q_sensor
```

---

### 3. `ouster_timestamp_relay` — Per-Punkt-Zeitstempel für LIO-SAM

**Problem:** Das echte Ouster-LiDAR liefert für jeden Punkt einen Zeitstempel `t`, der angibt wann genau dieser Punkt erfasst wurde (der Sensor dreht sich, Punkte am Anfang und Ende eines Scans haben unterschiedliche Zeitstempel). LIO-SAM nutzt diese Timestamps um Bewegungsverzerrungen im Scan zu korrigieren (Deskewing). Gazebos LiDAR-Plugin liefert dieses Feld nicht — ohne `t` verwirft LIO-SAM jeden einzelnen Scan.

**Lösung:** Dieser Node berechnet `t` synthetisch aus dem Azimutwinkel jedes Punktes und hängt das Feld an die Punktwolke an.

---

## Konfigurationsdateien

| Datei | Zweck |
|-------|-------|
| `config/params_sim.yaml` | LIO-SAM-Parameter (Topics, Frames, Sensor-Konfiguration) |
| `config/robot_sim.urdf.xacro` | URDF-Robotermodell für die Simulation |
| `config/rviz2_sim_Aeris.rviz` | RViz2-Layout |
| `config/gazebo_gui.config` | Gazebo-GUI (Panel-Layout, Teleop-Plugin, Kamerapose) |
| `worlds/Sonoma_world.sdf` | Gazebo-Simulationswelt |

---

## Topics (Übersicht)

| Topic | Typ | Richtung |
|-------|-----|----------|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo → ROS |
| `/imu/data_world` | `sensor_msgs/Imu` | Adapter → LIO-SAM |
| `/ouster/points` | `sensor_msgs/PointCloud2` | Gazebo → ROS |
| `/ouster/points_timestamped` | `sensor_msgs/PointCloud2` | Adapter → LIO-SAM |
| `/model/prius_hybrid/odometry` | `nav_msgs/Odometry` | Gazebo → ROS |

---

## Abhängigkeiten

- `ros_gz_sim` — Gazebo (Ignition) Integration
- `ros_gz_bridge` — ROS ↔ Gazebo Topic-Bridge
- `lio_sam` — LiDAR-Odometrie und SLAM
- `robot_state_publisher`, `tf2_ros`
- `rviz2`

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
