# autonomous_driving

ROS2-Paket für die Gazebo-Simulation und die Nav2-basierte Trajektorienplanungs-Pipeline des Aeris-Roboters. Enthält die Simulationswelt, Launch-Files, URDF-Modell und die Hilfsnode die eine Lücke zwischen Gazebo `gpu_lidar` und LIO-SAM schließt.

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
**Status: deaktiviert** — Node existiert noch im Repository, wird aber nicht mehr gestartet.
War als TF-Lückenfüller (`odom → chassis`) gedacht bevor LIO-SAMs `imuPreintegration` aktiv wird. Erzeugte TF-Konflikte weil zwei Quellen denselben Transform gleichzeitig publizierten. LIO-SAMs `imuPreintegration` übernimmt `odom → chassis` ab dem ersten IMU-Scan selbst.
**Letzte Änderung:** 2026-06-23

---

### `autonomous_driving/imu_world_orientation_relay.py`
**Status: deaktiviert** — Node existiert noch im Repository, wird aber nicht mehr gestartet.
War als Workaround gedacht weil `<localization>WORLD</localization>` im SDF nicht unterstützt wurde. Nach Analyse: gz-sensors8 kennt keinen `WORLD`-Enum-Wert — der korrekte SDF-Wert ist `ENU`. Mit `<localization>ENU</localization>` im SDF liefert Gazebo Harmonic 8.14 die IMU-Orientierung direkt im Weltframe, der Relay ist überflüssig.
**Letzte Änderung:** 2026-06-23

---

### `autonomous_driving/ouster_timestamp_relay.py`
**Zweck:** Fügt ein synthetisches `t`-Feld (uint32, Nanosekunden relativ zum Scan-Start) zur Gazebo-LiDAR-Punktwolke hinzu. Gazebos `gpu_lidar` liefert keine Per-Punkt-Timestamps — ohne sie berechnet LIO-SAMs `imageProjection` `timeScanEnd == timeScanCur`, Deskewing wird deaktiviert und das SLAM divergiert.
**Warum notwendig:** Gazebo `gpu_lidar` ist ein generisches Plugin ohne Ouster-spezifische Felder (`t`, `reflectivity`, `ambient`, `range`). Der echte `ouster-ros`-Driver liefert `t` nativ als Hardware-Timestamp. Diese Lücke ist eine fundamentale Gazebo-Limitation, kein Konfigurationsfehler.
**Timestamp-Berechnung:** Azimutwinkel → Spaltenindex → anteiliger Zeitstempel innerhalb der Scan-Periode (`col * scan_period_ns / num_columns`).
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

- Gazebo (Ignition/Harmonic 8.14) mit `worlds/Sonoma_world.sdf`
- `ros_gz_bridge` — bridged `/clock`, `/ouster/points`, `/imu/data`, `/model/prius_hybrid/odometry`; IMU-Queue `depth=200` via `qos_overrides` um Timestamp-Versatz zwischen IMU und LiDAR zu verhindern
- `robot_state_publisher` mit `config/robot_sim.urdf.xacro`
- `ouster_timestamp_relay` — fügt `t`-Feld zur Gazebo-Punktwolke hinzu
- TF-Aliase für Gazebo-Scoped-Framenamen (`ouster_lidar_link`, `imu_link`)
- Statischer `map → odom` Transform
- RViz2 mit `config/rviz2_sim_Aeris.rviz`
**Aufruf:** `ros2 launch autonomous_driving sim_Aeris.launch.py`
**Letzte Änderung:** 2026-06-23

---

## Konfiguration (`config/`)

| Datei | Zweck |
|---|---|
| `params_sim.yaml` | Parameter für alle Sim-Nodes (Spawn-Pose, Topics, LiDAR-Einstellungen) |
| `bridge_qos.yaml` | Nicht mehr verwendet (QoS jetzt direkt als `ros_arguments` in der Bridge-Node) |
| `teleop_params.yaml` | Startwerte für teleop_twist_keyboard (`speed`, `turn`) |
| `robot_sim.urdf.xacro` | URDF/Xacro-Robotermodell für die Simulation |
| `rviz2_sim_Aeris.rviz` | RViz2-Konfiguration für die Simulation |
| `gazebo_gui.config` | Gazebo-GUI-Layout |

## Simulation (`worlds/`)

| Datei | Zweck |
|---|---|
| `Sonoma_world.sdf` | Gazebo-Simulationswelt (Außengelände); IMU mit `<localization>ENU</localization>` konfiguriert |

---

## Gazebo-Limitierungen vs. echter Sensor

| Eigenschaft                              | Echter Ouster (ouster-ros)  | Gazebo gpu_lidar                                    |
| ---------------------------------------- | --------------------------- | --------------------------------------------------- |
| Per-Punkt-Timestamp `t`                  | Hardware-nativ              | fehlt → `ouster_timestamp_relay` nötig              |
| IMU-Weltframe-Orientierung               | nativ korrekt               | `<localization>ENU</localization>` im SDF nötig     |
| `odom → base` TF                         | LIO-SAM ab erstem Scan      | LIO-SAM ab erstem Scan (identisch)                  |
| Felder `reflectivity`, `ambient`, `range`| vorhanden                   | fehlen (für LIO-SAM nicht benötigt)                 |

---

## Anweisung für Claude

**Wenn du eine Datei in diesem Paket änderst:**
1. Suche den Eintrag dieser Datei in dieser `CLAUDE.md`.
2. Aktualisiere Zweck, Parameter, Eingaben oder Ausgaben wenn nötig.
3. Setze `Letzte Änderung` auf das heutige Datum.

**Wichtiger Zusammenhang:** `ouster_timestamp_relay` existiert ausschließlich wegen der fehlenden Per-Punkt-Timestamps in Gazebos `gpu_lidar`-Plugin — das ist eine fundamentale Gazebo-Limitation. Die beiden deaktivierten Nodes (`gazebo_odom_tf`, `imu_world_orientation_relay`) beruhten auf falschen Annahmen und wurden nach Analyse entfernt.
