# MPC Demo Manual
## Launch files
ros2 launch usv_description rviz_launch.py
ros2 launch usv_control mpc_launch.py
ros2 launch usv_control obstacles_mpc_launch.py
ros2 launch usv_control control_sim_launch.py (if extreme mode, run dynamic_model_node instead and uncomment thruster_pub_ lines at approx line 540)

## Static avoidance:
- Avoidance weight set to 2.0

- In obstacles.yaml, change:

    - (line 3): dynamic_obstacles: false

    - (line 9): x: 3.0

          

## Dynamic avoidance

- Avoidance weight set to 5.0

- In obstacles.yaml, change:

    - (line 3): dynamic_obstacles: true

    - (line 9): x: 3000.0

---

# A* Planner Manual

Global 2D planner over a cost grid, in local NED (x = North, y = East, meters).
It never works with lat/lon: `usv_localization` does that conversion using a **datum** (origin).

## Architecture

| Component | File | What it does |
|---|---|---|
| `GridMap` | `planning/grid_map.cpp` | 0–254 cost grid. World ↔ cell conversion. |
| `CostmapBuilder` | `planning/costmap_builder.cpp` | Inflates buoys and geofence: lethal up close, gradient beyond. |
| `AStarPlanner` | `planning/a_star_planner.cpp` | 8-connected A*; each step costs `dist·(1 + w·cost/253)`. |
| `PathPostProcessor` | `planning/path_post_processor.cpp` | Shortcut: joins vertices with a straight line if it doesn't cross cost > threshold. |
| `ObstacleTracker` | `planning/obstacle_tracker.cpp` | Memory of perception obstacles in NED (by uuid or proximity), with optional timeout. Also `bodyToNed()`. |
| `PlannerNode` | `a_star_planner_node.cpp` | ROS wrapper: reads parameters, replans at `publish_rate`, publishes `Path` or STOP. |

All except `PlannerNode` are pure C++ (no ROS) and have tests in `test/`.

**Topics**
- In: `usv/loc` (`UsvLoc`): boat pose and yaw.
- In: `usv/planning/goal` (`PoseStamped`): goal in NED.
- In: `objects_topic` (`ObjectList`, default `/obj_list_global` from `global_obstacle_register_node`): detected buoys.
- Out: `usv/planning/path` (`nav_msgs/Path`).

**Lethal start:** the start is always accepted. If the boat is inside a lethal zone (neighbors included),
the path first goes in a straight line to the nearest free cell and continues with A* from there.

**Costmap layers**
- *Static*: geofence + `buoys` from the YAML. Built once at startup.
- *Dynamic*: what perception sees. When an `ObjectList` arrives (or an object expires), the node does
  `map = static` and stamps the tracker's obstacles on top.

**`ObjectList` frame** (it has no header; the `objects_in_world` parameter decides):
- `true` (default, with `/obj_list_global`): x, y are already in local NED. The register computes them with
  `usv/state/pose`, published by `localization_node` with the same origin as `usv/loc`.
- `false` (e.g. `/obj_list`): relative to the boat, x toward the bow, y toward starboard. They are transformed with
  the latest pose and yaw from `usv/loc`, after adding `sensor_x`, `sensor_y`.
- Radius: `Object.type` is looked up in `obstacle_types`/`obstacle_radii`; if not found, `buoy_radius`.
- Padding entries are discarded: `color` -1 or 5, `type` `"ignore"` or `"NaN"`.

**STOP** = `Path` with zero poses. Published if there is no goal, no pose (or `loc_timeout` elapsed),
the position is not valid (if `require_valid_position: true`), the boat or the goal is outside the map,
or A* finds no path. The reason is logged as `STOP: ...`.

## Configuration

Two files, loaded in this order (if a parameter is repeated, the last one wins):

1. `config/a_star_planner.yaml` — **general**, the same at every site: boat, inflation, A*, shortcut, rate.
2. `config/sites/<site>.yaml` — **per site**: datum, map bounds, geofence, buoys.
   It has one section for `localization_node` and another for `a_star_planner_node`; each node reads its own.

Rule of thumb: incoming data (pose, goal) says **what** to do and goes through topics;
parameters say **how** to do it and go in YAML. Derived values (number of cells,
`lethal_radius`) are computed in code, not written in the YAML.

### General parameters

| Parameter | Unit | What it does |
|---|---|---|
| `frame_id` | — | Label for the Path `header`. Doesn't transform anything. |
| `publish_rate` | Hz | Every tick **fully replans** (A* + shortcut) and publishes. |
| `loc_timeout` | s | No `usv/loc` for longer than this → STOP. |
| `require_valid_position` | — | STOP if `position_valid` is false. `false` only in simulation. |
| `resolution` | m/cell | Cell size. Computation grows with 1/res². |
| `boat_width` | m | Beam. `width/2` = inscribed radius. |
| `boat_length` | m | Length. Only for validation; not part of the lethal radius. |
| `safety_margin` | m | Buffer for localization and control error. |
| `influence_radius` | m | How far the cost gradient extends (from the obstacle edge). |
| `decay` | 1/m | k in `253·e^(−k(d − lethal))`: how fast the cost drops. |
| `objects_topic` | — | `ObjectList` topic. `/obj_list_global` (NED) or `/obj_list` (boat-relative). |
| `objects_in_world` | — | `true` if `objects_topic` is in NED; `false` if boat-relative. |
| `obstacle_types`, `obstacle_radii` | —, m | Radius per `Object.type` (parallel lists, same size). |
| `buoy_radius` | m | Radius for a type not in `obstacle_types`. |
| `sensor_x`, `sensor_y` | m | Sensor position on the boat (bow, starboard), for relative objects. |
| `merge_distance` | m | Two detections without uuid closer than this are treated as the same object. |
| `obstacle_timeout` | s | Forget objects not seen for this long. `0` = never (fixed buoys). |
| `cost_weight` | — | 0 = ignores the gradient; high = gives buoys a wide berth. |
| `cost_threshold` | 0–253 | Maximum cost a shortcut segment may cross. |

Computed lethal radius: `lethal = boat_width/2 + safety_margin + resolution/2` (logged at startup).
If gaps between buoys "close up", lower `safety_margin`.

### Per-site parameters

| Parameter | Node | What it does |
|---|---|---|
| `use_fixed_datum` | localization | `true` = uses `datum_lat/lon`; `false` = averages the first fixes. |
| `datum_lat`, `datum_lon` | localization | Fixed origin [deg]. |
| `origin_samples` | localization | How many valid fixes to average in automatic mode. |
| `x_min`, `x_max`, `y_min`, `y_max` | planner | Grid bounds [m]. Must cover the geofence. |
| `geofence` | planner | `[x1, y1, x2, y2, ...]` vertices in order. Outside = lethal; the wall is inflated. |
| `buoys` | planner | `[x, y, radius, x, y, radius, ...]` |

Array rules: every number with a decimal point (`5.0`, not `5`) and no empty lists (`[]`);
if there are no buoys or no geofence, comment the line out.

## Running

```bash
colcon build --packages-select usv_control usv_localization --symlink-install
ros2 launch usv_localization localization_launch.py site:=<site>
ros2 launch usv_control a_star_planner_launch.py site:=<site>     # default: sim
```

- Editing an existing YAML: **restart the node** (parameters are only read at startup). With `--symlink-install` there's no need to rebuild.
- Creating a new site file: you **do** need to run `colcon build` again so it gets installed.

Test with a goal:
```bash
ros2 topic pub --once usv/planning/goal geometry_msgs/msg/PoseStamped "{pose: {position: {x: 10.0, y: 0.0}}}"
ros2 topic echo usv/planning/path
```

Test with a fake buoy, 5 m ahead of the boat:
```bash
# with objects_in_world: false and objects_topic: /obj_list
ros2 topic pub --once /obj_list usv_interfaces/msg/ObjectList \
  "{obj_list: [{x: 5.0, y: 0.0, type: 'round', uuid: 'test1'}]}"
```

## New site (e.g. the dam)

The idea: fix **one origin** and describe the place **relative to that origin**.

1. **Create the file.** Copy `config/sites/presa.yaml` as a template and run `colcon build`.
2. **Fix the datum.**
   - Boat **stationary** at a recognizable spot (dock), with a valid SBG fix.
   - Launch localization with `use_fixed_datum: false` and `origin_samples: 250` (~5 s).
   - The log prints `Datum AUTO listo: datum_lat: ...  datum_lon: ...`. Copy them into the YAML and set `use_fixed_datum: true`.
   - Restart. The log should say `Datum FIJO`, and at the same spot `usv/loc` should give x, y ≈ 0.
   - ⚠️ This is the only irreversible step: if the datum is wrong, everything you measure afterwards is shifted.
3. **Measure with the boat.** With the datum fixed, `usv/loc` already gives NED. Take the boat to each corner of the safe
   area (→ `geofence`) and to each buoy (→ `buoys`) and write down x, y.
   The position is the **sensor's**, not the bow's: correct by ~0.5–1 m if you touch the buoy with the bow.
4. **Bounds.** Set `x_min..y_max` so they cover the geofence with a few meters to spare.
5. **Launch the planner** with `site:=<site>`. The log reports how many buoys and vertices it loaded and the map size.
6. **Test** with a goal (see above). If the path is empty, read the `STOP: ...` in the log.
7. **Tune** `safety_margin`, `cost_weight`, etc. Edit, restart, repeat.

## Tests

### 1. Without simulation (gtest)

```bash
colcon build --packages-select usv_control && colcon test --packages-select usv_control
colcon test-result --verbose
# or a single one: ./build/usv_control/test_scenarios
```

`test/test_scenarios.cpp` runs the full pipeline (`build → stampGeofence → plan → shortcut`) with the
default YAML parameters. If you change a parameter and a scenario fails, decide whether the change in
behavior is the one you wanted and update the test.

### 2. In Gazebo, fixed buoys

Each scenario has a file in `config/sites/tests/`. The goal and expected result are at the top of each file.
With the boat at the origin, the result must match the gtest one.

```bash
ros2 launch usv_control a_star_planner_launch.py site:=tests/<scenario>
ros2 topic pub --once usv/planning/goal geometry_msgs/msg/PoseStamped "{pose: {position: {x: 20.0, y: 0.0}}}"
ros2 topic echo --once usv/planning/path | grep -A2 "position:"
```

| Scenario | What it tests | Goal | Expected |
|---|---|---|---|
| `s01_libre` | Baseline | (20, 0) | 2 points, straight |
| `s02_boya_unica` | Going around | (20, 0) | 3 points, > 0.7 m from the edge |
| `s03_puerta_ancha` | 3 m gate | (20, 0) | 2 points, crosses straight |
| `s04_puerta_2m` | 2 m gate | (20, 0) | Crosses, with ~13 points near the gate |
| `s05_canal_puerta_cerrada` | Inflation that closes the gap + geofence | (20, 0) | STOP: no path found |
| `s06_slalom` | Alternating buoys | (32, 0) | ~5 points in a zigzag |
| `s07_pared_con_hueco` | Find the only opening | (20, 0) | Crosses x = 10 with 4 < y < 6 |
| `s08_goal_en_boya` | Invalid goal | (10, 0) | STOP: no path found |
| `s09_bote_junto_a_boya` | Escape from lethal start | (10, 0) | Moves away from the buoy, then goes to the goal |
| `s10_goal_fuera_geocerca` | Geofence | (20, 0) | STOP: no path found |
| `s11_callejon_u` | Get out of a dead end | (16, 0) | Exits through x < 0 and goes around |

### 3. In Gazebo, perception buoys (dynamic)

With `site:=sim` (or `tests/s01_libre`), publish objects by hand. Without `global_obstacle_register_node` running:
if it is running, it publishes `/obj_list_global` at 20 Hz and overwrites whatever you send.

**a) NED frame.** Buoy at world (10, 0), goal (20, 0) → the path goes around it.
```bash
ros2 topic pub --once /obj_list_global usv_interfaces/msg/ObjectList \
  "{obj_list: [{x: 10.0, y: 0.0, type: 'round', uuid: 'n1'}]}"
```

**b) Boat frame and sign of y.** First check the yaw: `ros2 topic echo --once usv/loc | grep yaw`.
Launch the planner with `objects_in_world: false` and `objects_topic: /obj_list`, and publish a buoy 8 m ahead and 2 m to starboard:
```bash
ros2 topic pub --once /obj_list usv_interfaces/msg/ObjectList \
  "{obj_list: [{x: 8.0, y: 2.0, type: 'round', uuid: 'b1'}]}"
```
With yaw ≈ 0 (bow pointing North), the buoy should end up at NED ≈ (8, 2), **to the right** of the boat.
A goal at (8, 2) should give STOP (goal on top of the buoy). If instead the STOP happens with (8, −2),
the sign of y is flipped between perception and the planner.

**c) Memory (no timeout).** After (a), stop publishing: the path **keeps** going around the buoy.

**d) Forgetting (timeout).** `site:=tests/s12_timeout` (`obstacle_timeout: 5.0`). Publish (a) once:
the path goes around it and, about 5 s later, becomes straight again.

**e) Association without uuid.** Publish (a) without `uuid` and then with `x: 10.3, y: 0.2`. They are the same buoy
(closer than `merge_distance`), so the path barely moves. With `x: 13.0` a second buoy appears.

**f) Replanning while moving.** With LOS following the path, publish a buoy (`ned` frame) on the path
ahead of the boat: on the next tick (≤ 1/`publish_rate` s) the path should change to go around it.

## Debugging, in this order

1. No path? → read `STOP: ...`. Most often the boat or the goal is outside the map or in a lethal zone.
2. Path goes through buoys or looks weird? → axis convention (NED vs ENU) between publisher and consumer.
3. Won't pass between buoys? → `safety_margin`, `resolution` and inflation. A narrow gate is expensive and A* may prefer to go around it:
   to force it through, send the center of the gate as an intermediate goal.
4. Path jumps between runs? → the datum is not fixed.
5. A class fails? → `./build/usv_control/test_<class>` to isolate it.