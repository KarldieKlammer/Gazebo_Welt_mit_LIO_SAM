# autonomous_driving

ROS2-Paket für die Gazebo-Simulation und die Nav2-basierte Trajektorienplanungs-Pipeline des Aeris-Roboters. Enthält die Simulationswelt, Launch-Files, URDF-Modell und Hilfs-Nodes die Lücken zwischen dem Gazebo-Plugin und LIO-SAM schließen.

---

## Nav2-Pipeline (in Entwicklung)

Ziel: Vollständige Trajektorienplanung für autonomes Fahren in der Gazebo-Simulationsumgebung.
Architekturentscheidung: → [ADR-001](../../docs/adr/ADR_001_trajektorienplanung_pipeline_projektstart.md)

### Integrationsarchitektur

```
LIO-SAM /lio_sam/mapping/odometry  ──► Nav2 Lokalisierung (ersetzt AMCL)
LIO-SAM /lio_sam/mapping/map       ──► Nav2 Map Server

[noch zu entwickeln: Costmap-Quelle, eigenes Package]
    └─► Nav2 Costmap2D

Nav2 Planner Server    ──► globaler Pfad  (Algorithmus: noch offen)
Nav2 Controller Server ──► lokale Trajektorie (MPPI geplant)
    └─► /cmd_vel  ──► Fahrzeugsteuerung (Ackermann)
```

### Geplante Komponenten

| Komponente | Nav2-Rolle | Status |
|---|---|---|
| LIO-SAM Odometrie | Lokalisierung (map→odom TF) | zu integrieren |
| LIO-SAM Map | Statische Karte für globale Planung | zu integrieren |
| Costmap-Package | Costmap2D-Quelle | Package noch ausstehend |
| Nav2 Planner Server | Globale Pfadplanung | zu konfigurieren |
| Nav2 Controller Server | Lokale Trajektorie (MPPI) | zu konfigurieren |
| Nav2 BT Navigator | Verhaltenssteuerung | zu konfigurieren |

### Neue Packages
Sobald neue Packages für Nav2-Komponenten erstellt werden, werden sie hier verlinkt und in `src/CLAUDE.md` eingetragen.

Konfigurationsdatei: `config/params_sim.yaml`

---

## Nodes

### `autonomous_driving/gazebo_odom_tf.py`
**Zweck:** Publiziert den `odom → chassis` TF-Transform aus Gazebos AckermannSteering-Odometrie. Füllt die TF-Lücke bevor LIO-SAMs TransformFusion aktiv ist — danach überschreibt LIO-SAM diesen Transform automatisch.
**Schlüsselklasse:** `GazeboOdomTf`
**Eingaben:** `odom_topic` — Odometrie vom Gazebo-AckermannSteering-Plugin (`Odometry`)
**Ausgaben:** TF-Broadcast `odom → chassis`
**Parameter:** `odom_topic`, `odom_frame` (default: `odom`), `base_frame` (default: `chassis`)
**Letzte Änderung:** 2026-06-12

---

### `autonomous_driving/imu_world_orientation_relay.py`
**Zweck:** Korrigiert die Gazebo-IMU-Orientierung von sensorrelativ auf Weltframe. Gazebo liefert die Quaternion relativ zur Sensor-Startpose (Identität bei t=0) — LIO-SAM braucht aber die Orientierung im Weltframe für die Heading-Initialisierung.
**Transformation:** `q_world = q_spawn ⊗ q_sensor`
**Schlüsselklasse:** `ImuWorldOrientationRelay`
**Eingaben:** `input_topic` — IMU-Daten von Gazebo (`Imu`)
**Ausgaben:** `output_topic` — korrigierte IMU-Daten im Weltframe (`Imu`)
**Parameter:** `spawn_roll`, `spawn_pitch`, `spawn_yaw` (Spawn-Pose des Fahrzeugs in rad), `input_topic`, `output_topic`
**Letzte Änderung:** 2026-06-12

---

### `autonomous_driving/ouster_timestamp_relay.py`
**Zweck:** Fügt ein synthetisches `t`-Feld (uint32, Nanosekunden relativ zum Scan-Start) zur Gazebo-LiDAR-Punktwolke hinzu. Gazebos `gpu_lidar` liefert keine Per-Punkt-Timestamps — ohne sie setzt LIO-SAMs `imageProjection` jeden Scan als ungültig (`timeScanEnd == timeScanCur`).
**Timestamp-Berechnung:** Azimutwinkel → Spalte → anteiliger Zeitstempel innerhalb der Scan-Periode.
**Schlüsselklasse:** `OusterTimestampRelay`
**Eingaben:** `input_topic` — Punktwolke ohne `t`-Feld (`PointCloud2`)
**Ausgaben:** `output_topic` — Punktwolke mit `t`-Feld (`PointCloud2`)
**Parameter:** `scan_frequency` (default: 10.0 Hz), `num_columns` (default: 1024), `input_topic`, `output_topic`
**Letzte Änderung:** 2026-06-12

---

## Launch

### `launch/sim_Aeris.launch.py`
**Zweck:** Startet die vollständige Gazebo-Simulation für den Aeris-Roboter.
**Startet:**
- Gazebo (Ignition) mit `worlds/Sonoma_world.sdf`
- `ros_gz_bridge` — bridged `/clock`, `/ouster/points`, `/imu/data`, `/model/prius_hybrid/odometry`
- `robot_state_publisher` mit `config/robot_sim.urdf.xacro`
- `gazebo_odom_tf`, `imu_world_orientation_relay`, `ouster_timestamp_relay`
- `teleop_twist_keyboard` in eigenem xterm-Fenster (Parameter aus `config/teleop_params.yaml`)
- RViz2 mit `config/rviz2_sim_Aeris.rviz`
**Aufruf:** `ros2 launch autonomous_driving sim_Aeris.launch.py`
**Letzte Änderung:** 2026-06-12

---

## Konfiguration (`config/`)

| Datei | Zweck |
|---|---|
| `params_sim.yaml` | Parameter für alle Sim-Nodes (Spawn-Pose, Topics, LiDAR-Einstellungen) |
| `teleop_params.yaml` | Startwerte für teleop_twist_keyboard (`speed`, `turn`) |
| `robot_sim.urdf.xacro` | URDF/Xacro-Robotermodell für die Simulation |
| `rviz2_sim_Aeris.rviz` | RViz2-Konfiguration für die Simulation |
| `gazebo_gui.config` | Gazebo-GUI-Layout |

## Simulation (`worlds/`)

| Datei | Zweck |
|---|---|
| `Sonoma_world.sdf` | Gazebo-Simulationswelt (Außengelände) |

---

## Anweisung für Claude

**Wenn du eine Datei in diesem Paket änderst:**
1. Suche den Eintrag dieser Datei in dieser `CLAUDE.md`.
2. Aktualisiere Zweck, Parameter, Eingaben oder Ausgaben wenn nötig.
3. Setze `Letzte Änderung` auf das heutige Datum.

**Wichtiger Zusammenhang:** Die drei Hilfs-Nodes (`gazebo_odom_tf`, `imu_world_orientation_relay`, `ouster_timestamp_relay`) existieren ausschließlich um Inkompatibilitäten zwischen Gazebo-Plugins und LIO-SAM zu überbrücken — Änderungen dort erfordern Verständnis beider Systeme.
