# NavRover — Autonomous Surface Traversal & Route Algorithm

ASTRA est un rover physique autonome construit sur châssis 4 roues, capable de percevoir son environnement, de se localiser dans une carte qu'il construit lui-même, et de naviguer vers des waypoints GPS sans intervention humaine. Le projet repose sur une architecture ROS2, avec un Raspberry Pi 4 comme cerveau central et un Arduino comme contrôleur bas niveau.

---

## Hypothèses de conception

- **La carte prime sur l'algorithme.** A* est trivial à implémenter ; le vrai défi est la qualité de la grille d'occupation. Un mauvais scan LiDAR ou une mauvaise localisation fait échouer la navigation quel que soit le planificateur.
- **Architecture deux niveaux.** L'Arduino gère le temps-réel bas niveau (PWM moteurs, lecture ultrason, watchdog). Le Raspberry Pi gère la perception, la cartographie, la planification et la décision. Aucun calcul de navigation ne se fait sur l'Arduino.
- **Progression incrémentale.** Le rover est opérationnel à chaque étape — télécommande fonctionnelle avant d'ajouter l'évitement, évitement avant d'ajouter le GPS, etc. Aucune étape ne casse la précédente.
- **ROS2 comme middleware.** Chaque module (LiDAR, caméra, GPS, planificateur) est un nœud ROS2 indépendant. La communication passe par des topics/services. Cela permet de remplacer ou simuler n'importe quel composant sans toucher au reste.
- **Carte en grille d'occupation.** L'environnement est représenté comme une grille 2D de cases libres/occupées/inconnues, mise à jour à chaque scan LiDAR. C'est la représentation standard compatible avec `nav2` et A*.

---

## Matériel

| Composant | Rôle |
|---|---|
| Raspberry Pi 4 (4 GB) | Cerveau principal — ROS2, cartographie, planification |
| Arduino Mega / Uno | Contrôle bas niveau — moteurs DC, PWM, ultrasons |
| LiDAR RPLIDAR A1 / YDLiDAR | Scan 360° de l'environnement, construction de la grille |
| Caméra Raspberry Pi Camera v2 | Vision — détection d'obstacles, repères visuels |
| Module GPS u-blox NEO-M8N | Localisation globale, navigation par waypoints |
| Châssis 4WD | Structure physique — moteurs DC avec encodeurs |
| Pont en H L298N / TB6612 | Pilotage des moteurs depuis l'Arduino |
| Capteurs ultrason HC-SR04 (x4) | Sécurité proximité (avant, arrière, côtés) |
| Batterie LiPo 3S 5000 mAh | Alimentation principale |
| Powerbank USB (5V) | Alimentation Raspberry Pi isolée |

---

## Architecture logicielle

```
ASTRA/
├── astra_bringup/          → Launch files ROS2, configuration globale
├── astra_hardware/
│   ├── arduino/            → Firmware Arduino (serial protocol, moteurs, ultrasons)
│   └── ros2_bridge/        → Nœud ROS2 ↔ Arduino (pyserial)
├── astra_sensors/
│   ├── lidar/              → Traitement scans LiDAR, publication /scan
│   ├── camera/             → Pipeline caméra, détection obstacles visuels
│   └── gps/                → Parsing NMEA, publication /fix (NavSatFix)
├── astra_mapping/
│   ├── occupancy_grid/     → Construction et mise à jour de la grille d'occupation
│   └── slam/               → Intégration slam_toolbox (cartographie + localisation)
├── astra_navigation/
│   ├── path_planner/       → Implémentation A* sur la grille d'occupation
│   ├── local_planner/      → Évitement d'obstacles dynamiques (DWA)
│   └── waypoint_manager/   → Gestion de la liste de waypoints GPS
├── astra_control/
│   ├── velocity_controller/→ Traduction cmd_vel → commandes moteurs
│   └── safety_monitor/     → Watchdog, arrêt d'urgence sur proximité
├── docs/
│   ├── hardware_setup.md   → Schémas de câblage, pinouts
│   ├── ros2_architecture.md→ Graphe des nœuds et topics
│   └── calibration.md      → Calibration LiDAR, caméra, IMU
└── tests/
    ├── unit/               → Tests unitaires Python (A*, décodeur GPS, grille)
    └── simulation/         → Scènes Gazebo pour valider sans hardware
```

---

## Boucle de navigation autonome

À chaque cycle, le rover exécute la séquence suivante :

```
1. PERCEPTION   → LiDAR scan + ultrason → nuage de points 2D
2. MAPPING      → Mise à jour de la grille d'occupation (cases libres/occupées)
3. LOCALISATION → SLAM (scan matching) + fusion GPS → position estimée
4. PLANIFICATION→ A* sur la grille : position actuelle → prochain waypoint
5. CONTRÔLE     → Suivi de trajectoire via DWA → cmd_vel → Arduino → moteurs
6. SURVEILLANCE → Si obstacle à < 30 cm : arrêt immédiat + recalcul
```

Si un obstacle dynamique apparaît en cours de route, le planificateur local tente un évitement DWA. Si la déviation est trop importante, le planificateur global recalcule un nouveau chemin A* depuis la position courante.

---

## Algorithme A*

A* est appliqué directement sur la grille d'occupation 2D. Chaque case est un nœud ; les voisins sont les 8 cases adjacentes (mouvement diagonaux inclus). Les cases marquées `OCCUPIED` ou `UNKNOWN` sont considérées infranchissables.

```python
# astra_navigation/path_planner/astar.py
def find_path(grid: OccupancyGrid, start: Cell, goal: Cell) -> list[Cell]:
    open_set = [(0, start)]
    g_score = {start: 0}
    came_from = {}
    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor in get_neighbors(grid, current):
            tentative_g = g_score[current] + movement_cost(current, neighbor)
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heappush(open_set, (f, neighbor))
    return []  # Aucun chemin trouvé
```

La fonction heuristique utilise la distance euclidienne entre la case courante et le but, pondérée par la résolution de la grille en mètres/case.

---

## Protocole de communication Arduino ↔ Raspberry Pi

La communication passe par liaison série USB (115200 baud). Le protocole est minimaliste et orienté lignes ASCII pour faciliter le debug.

```
# Raspberry Pi → Arduino (commandes)
CMD:FWD:150:150\n      → avancer (PWM gauche:droite)
CMD:STOP\n             → arrêt immédiat
CMD:ROT:100:-100\n     → rotation sur place

# Arduino → Raspberry Pi (état)
ECHO:FWD:150:150\n     → confirmation de la commande reçue
US:12:45:89:67\n       → distances ultrason (avant, droite, arrière, gauche) en cm
ENC:1024:1020\n        → ticks encodeurs (gauche:droite)
BATT:11.8\n            → tension batterie en volts
```

Le nœud ROS2 `astra_hardware/ros2_bridge` traduit ces messages en topics (`/cmd_vel`, `/sonar`, `/odom`, `/battery_state`).

---

## Progression recommandée

### Étape 1 — Télécommande manuelle
Valider le câblage complet. Le rover répond aux commandes clavier via `teleop_twist_keyboard`. Rien d'autonome encore.

**Ce qu'on code :** firmware Arduino + nœud ros2_bridge + test de la chaîne série

### Étape 2 — Évitement d'obstacles basique
Les capteurs ultrason déclenchent un arrêt et une manœuvre d'évitement simple (reculer, tourner). Pas de carte, pas de planification.

**Ce qu'on code :** safety_monitor, logic d'évitement réactif

### Étape 3 — Intégration LiDAR et première carte
Le LiDAR construit une grille d'occupation locale. On visualise la carte dans RViz2 en temps réel. Le rover ne navigue pas encore, mais on valide la qualité de la perception.

**Ce qu'on code :** nœud lidar, occupancy_grid, visualisation RViz2

### Étape 4 — GPS et waypoints
Le rover se rend à un waypoint GPS en ligne droite, sans carte. On valide la chaîne GPS → coordonnées → cap à suivre → cmd_vel.

**Ce qu'on code :** nœud GPS, waypoint_manager, contrôleur de cap simple

### Étape 5 — Cartographie + A*
On combine la grille d'occupation avec A* pour planifier des chemins qui contournent les obstacles. C'est l'étape centrale du projet.

**Ce qu'on code :** path_planner (A*), intégration avec la grille, local_planner DWA

### Étape 6 — SLAM complet
Intégration de `slam_toolbox` pour une localisation robuste sans GPS (intérieur) ou en fusion avec le GPS (extérieur). La carte est persistante entre les sessions.

**Ce qu'on code :** configuration slam_toolbox, fusion odométrie + LiDAR

---

## Complexités volontairement mises de côté (v1)

Ces points ont été identifiés et documentés, mais écartés pour maintenir une première version fonctionnelle :

- **Fusion capteurs IMU** : l'IMU améliorerait l'estimation d'odométrie, notamment en terrain irrégulier. Non intégré en v1.
- **Détection d'obstacles par caméra** : la caméra est installée mais non utilisée pour la navigation. Prévu pour la v2 (YOLO ou segmentation légère).
- **Topographie du terrain** : la grille est strictement 2D. Une bosse ou une pente ne sont pas distinguées d'une surface plane.
- **Localisation multi-robots** : un seul rover opère à la fois. Pas de coordination entre agents.
- **Récupération en cas de blocage** : si le rover se coince physiquement, il n'a pas de comportement de récupération (marche arrière forcée, rotation aléatoire). À implémenter.

---

## Reste à développer

### Fusion visuelle (caméra + LiDAR)
Utiliser la caméra pour détecter et classifier des obstacles que le LiDAR ne distingue pas (barrières transparentes, sol glissant). Pipeline de détection léger (MobileNet SSD ou YOLO-Nano) tournant sur le Raspberry Pi.

### Exploration autonome
Au lieu de naviguer vers des waypoints prédéfinis, le rover explore un espace inconnu en cherchant à maximiser la découverte de zones `UNKNOWN` dans la carte (algorithme frontier-based exploration).

### Navigation en intérieur
En l'absence de GPS, utiliser uniquement le SLAM LiDAR pour se localiser. Requiert une carte initiale ou une phase d'exploration préalable.

### Interface de supervision
Dashboard web (ROS2 Web Bridge + React) affichant en temps réel : la grille d'occupation, la position du rover, le chemin planifié, l'état des capteurs et la tension batterie.

---

## Installation

```bash
# Prérequis : ROS2 Humble sur Ubuntu 22.04
git clone https://github.com/alassane8/ASTRA.git
cd ASTRA
pip install -r requirements.txt
colcon build
source install/setup.bash

# Flasher le firmware Arduino
cd astra_hardware/arduino
arduino-cli compile --fqbn arduino:avr:mega astra_firmware
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega astra_firmware

# Lancer le rover
ros2 launch astra_bringup astra.launch.py
```

---

## Stack technique

| Couche | Technologies |
|---|---|
| Middleware | ROS2 Humble |
| Cartographie / SLAM | slam_toolbox, nav2 |
| Planification | A* (custom), Nav2 planner |
| Contrôle bas niveau | Arduino (C++), pyserial |
| Vision | OpenCV, ROS2 image_pipeline |
| GPS | u-blox NEO-M8N, nmea_navsat_driver |
| Simulation | Gazebo Ignition, RViz2 |
| Tests | pytest, ROS2 launch_testing |
| Langage principal | Python 3.10+, C++ (Arduino) |