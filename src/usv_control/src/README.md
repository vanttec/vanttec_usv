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

Planner global en 2D sobre un grid de costos, en NED local (x = Norte, y = Este, metros).
Nunca trabaja con lat/lon: eso lo convierte `usv_localization` usando un **datum** (origen).

## Arquitectura

| Pieza | Archivo | Qué hace |
|---|---|---|
| `GridMap` | `planning/grid_map.cpp` | Grid de costos 0–254. Conversión mundo ↔ celda. |
| `CostmapBuilder` | `planning/costmap_builder.cpp` | Infla boyas y geocerca: letal cerca, gradiente después. |
| `AStarPlanner` | `planning/a_star_planner.cpp` | A* 8-conectado; el paso cuesta `dist·(1 + w·costo/253)`. |
| `PathPostProcessor` | `planning/path_post_processor.cpp` | Shortcut: une vértices con línea recta si no cruza costo > umbral. |
| `ObstacleTracker` | `planning/obstacle_tracker.cpp` | Memoria de obstáculos de percepción en NED (por uuid o cercanía), con timeout opcional. También `bodyToNed()`. |
| `PlannerNode` | `a_star_planner_node.cpp` | Envoltura ROS: lee parámetros, replanea a `publish_rate`, publica `Path` o STOP. |

Todas menos `PlannerNode` son C++ puro (sin ROS) y tienen pruebas en `test/`.

**Tópicos**
- Entra `usv/loc` (`UsvLoc`): pose y yaw del bote.
- Entra `usv/planning/goal` (`PoseStamped`): goal en NED.
- Entra `usv/perception/objects` (`ObjectList`): boyas detectadas.
- Sale `usv/planning/path` (`nav_msgs/Path`).

**Inicio letal:** el inicio siempre se acepta. Si el bote está dentro de zona letal (vecinas incluidas),
el path sale primero en línea recta a la celda libre más cercana y desde ahí sigue con A*.

**Capas del costmap**
- *Estática*: geocerca + `buoys` del YAML. Se construye una vez al arrancar.
- *Dinámica*: lo que ve percepción. Cuando llega un `ObjectList` (o un objeto expira), se hace
  `map = estática` y se estampan encima los obstáculos del tracker.

**Frame de `ObjectList`** (por `header.frame_id`):
- `"ned"` (igual a `frame_id`): x, y ya están en NED local.
- Cualquier otro: relativos al bote, x hacia la proa, y hacia estribor. Se transforman con la pose
  y el yaw más recientes de `usv/loc`, sumando antes `sensor_x`, `sensor_y`.
- `radius = 0` → se usa `buoy_radius`.
**STOP** = `Path` con cero poses. Se publica si no hay goal, no hay pose (o pasó `loc_timeout`),
la posición no es válida (si `require_valid_position: true`), el bote o el goal están fuera del mapa,
o A* no encuentra camino. El motivo sale en el log como `STOP: ...`.

## Configuración

Dos archivos, cargados en este orden (si un parámetro se repite, gana el último):

1. `config/a_star_planner.yaml` — **general**, igual en todos los sitios: barco, inflado, A*, shortcut, rate.
2. `config/sites/<sitio>.yaml` — **por sitio**: datum, límites del mapa, geocerca, boyas.
   Tiene una sección para `localization_node` y otra para `a_star_planner_node`; cada nodo lee la suya.

Criterio: los datos que llegan (pose, goal) dicen **qué** hacer y van por tópico;
los parámetros dicen **cómo** hacerlo y van en YAML. Los valores derivados (número de celdas,
`lethal_radius`) se calculan en el código, no se escriben en el YAML.

### Parámetros generales

| Parámetro | Unidad | Qué hace |
|---|---|---|
| `frame_id` | — | Etiqueta del `header` del Path. No transforma nada. |
| `publish_rate` | Hz | Cada tick **replanea completo** (A* + shortcut) y publica. |
| `loc_timeout` | s | Sin `usv/loc` más tiempo que esto → STOP. |
| `require_valid_position` | — | STOP si `position_valid` es false. `false` solo en simulación. |
| `resolution` | m/celda | Tamaño de celda. El cómputo crece con 1/res². |
| `boat_width` | m | Manga. `width/2` = radio inscrito. |
| `boat_length` | m | Eslora. Solo para validar; no entra al radio letal. |
| `safety_margin` | m | Colchón por error de localización y control. |
| `influence_radius` | m | Hasta dónde llega el gradiente de costo (desde el borde del obstáculo). |
| `decay` | 1/m | k en `253·e^(−k(d − lethal))`: qué tan rápido baja el costo. |
| `buoy_radius` | m | Radio de un objeto de percepción que llega con `radius = 0`. |
| `sensor_x`, `sensor_y` | m | Dónde está el sensor en el bote (proa, estribor), para objetos relativos. |
| `merge_distance` | m | Dos detecciones sin uuid más cerca que esto se tratan como el mismo objeto. |
| `obstacle_timeout` | s | Olvida objetos que no se ven en este tiempo. `0` = nunca (boyas fijas). |
| `cost_weight` | — | 0 = ignora el gradiente; alto = rodea lejos de las boyas. |
| `cost_threshold` | 0–253 | Costo máximo que puede cruzar un atajo del shortcut. |

Radio letal calculado: `lethal = boat_width/2 + safety_margin + resolution/2` (sale en el log al arrancar).
Si los huecos entre boyas se "cierran", baja `safety_margin`.

### Parámetros por sitio

| Parámetro | Nodo | Qué hace |
|---|---|---|
| `use_fixed_datum` | localización | `true` = usa `datum_lat/lon`; `false` = promedia los primeros fixes. |
| `datum_lat`, `datum_lon` | localización | Origen fijo [deg]. |
| `origin_samples` | localización | Cuántos fixes válidos promediar en modo automático. |
| `x_min`, `x_max`, `y_min`, `y_max` | planner | Límites del grid [m]. Deben cubrir la geocerca. |
| `geofence` | planner | `[x1, y1, x2, y2, ...]` vértices en orden. Fuera = letal; la pared se infla. |
| `buoys` | planner | `[x, y, radio, x, y, radio, ...]` |

Reglas de los arreglos: todos los números con punto (`5.0`, no `5`) y nada de listas vacías (`[]`);
si no hay boyas o geocerca, comenta la línea.

## Correr

```bash
colcon build --packages-select usv_control usv_localization --symlink-install
ros2 launch usv_localization localization_launch.py site:=<sitio>
ros2 launch usv_control a_star_planner_launch.py site:=<sitio>     # default: sim
```

- Editar un YAML existente: **reiniciar el nodo** (los parámetros se leen solo al arrancar). Con `--symlink-install` no hace falta recompilar.
- Crear un archivo de sitio nuevo: **sí** hay que volver a hacer `colcon build` para que se instale.

Probar con un goal:
```bash
ros2 topic pub --once usv/planning/goal geometry_msgs/msg/PoseStamped "{pose: {position: {x: 10.0, y: 0.0}}}"
ros2 topic echo usv/planning/path
```

Probar con una boya falsa, 5 m a proa del bote:
```bash
ros2 topic pub --once usv/perception/objects usv_interfaces/msg/ObjectList \
  "{header: {frame_id: 'base_link'}, obj_list: [{x: 5.0, y: 0.0, radius: 0.3, uuid: 'test1'}]}"
```

## Sitio nuevo (ej. la presa)

La idea: fijar **un origen** y describir el lugar **respecto a ese origen**.

1. **Crear el archivo.** Copia `config/sites/presa.yaml` como plantilla y haz `colcon build`.
2. **Fijar el datum.**
   - Barco **quieto** en un punto reconocible (muelle), con fix válido de la SBG.
   - Lanza la localización con `use_fixed_datum: false` y `origin_samples: 250` (~5 s).
   - El log imprime `Datum AUTO listo: datum_lat: ...  datum_lon: ...`. Cópialos al YAML y pon `use_fixed_datum: true`.
   - Reinicia. El log debe decir `Datum FIJO`, y en el mismo punto `usv/loc` debe dar x, y ≈ 0.
   - ⚠️ Es el único paso irreversible: si el datum está mal, todo lo que midas después queda corrido.
3. **Medir con el barco.** Con el datum fijo, `usv/loc` ya da NED. Lleva el barco a cada esquina del área
   segura (→ `geofence`) y a cada boya (→ `buoys`) y anota x, y.
   La posición es la del **sensor**, no la de la proa: corrige ~0.5–1 m si tocas la boya con la proa.
4. **Límites.** Ajusta `x_min..y_max` para que cubran la geocerca con unos metros de sobra.
5. **Lanzar el planner** con `site:=<sitio>`. El log dice cuántas boyas y vértices cargó y el tamaño del mapa.
6. **Probar** con un goal (ver arriba). Si sale vacío, lee el `STOP: ...` del log.
7. **Afinar** `safety_margin`, `cost_weight`, etc. Edita, reinicia y repite.

## Pruebas

### 1. Sin simulación (gtest)

```bash
colcon build --packages-select usv_control && colcon test --packages-select usv_control
colcon test-result --verbose
# o una sola: ./build/usv_control/test_scenarios
```

`test/test_scenarios.cpp` corre el pipeline completo (`build → stampGeofence → plan → shortcut`) con los
parámetros por defecto del YAML. Si cambias un parámetro y un escenario falla, decide si el cambio de
comportamiento es el que querías y actualiza la prueba.

### 2. En Gazebo, boyas fijas

Cada escenario tiene un archivo en `config/sites/tests/`. Arriba de cada archivo vienen el goal y el resultado esperado.
Con el bote en el origen, el resultado tiene que coincidir con el de gtest.

```bash
ros2 launch usv_control a_star_planner_launch.py site:=tests/<escenario>
ros2 topic pub --once usv/planning/goal geometry_msgs/msg/PoseStamped "{pose: {position: {x: 20.0, y: 0.0}}}"
ros2 topic echo --once usv/planning/path | grep -A2 "position:"
```

| Escenario | Qué prueba | Goal | Esperado |
|---|---|---|---|
| `s01_libre` | Base | (20, 0) | 2 puntos, recto |
| `s02_boya_unica` | Rodear | (20, 0) | 3 puntos, a > 0.7 m del borde |
| `s03_puerta_ancha` | Puerta de 3 m | (20, 0) | 2 puntos, cruza recto |
| `s04_puerta_2m` | Puerta de 2 m | (20, 0) | Cruza, con ~13 puntos cerca de la puerta |
| `s05_canal_puerta_cerrada` | Inflado que cierra + geocerca | (20, 0) | STOP: no se encontró camino |
| `s06_slalom` | Boyas alternadas | (32, 0) | ~5 puntos en zigzag |
| `s07_pared_con_hueco` | Encontrar el único paso | (20, 0) | Cruza x = 10 con 4 < y < 6 |
| `s08_goal_en_boya` | Goal inválido | (10, 0) | STOP: no se encontró camino |
| `s09_bote_junto_a_boya` | Escape del inicio letal | (10, 0) | Se aleja de la boya y luego va al goal |
| `s10_goal_fuera_geocerca` | Geocerca | (20, 0) | STOP: no se encontró camino |
| `s11_callejon_u` | Salir de un callejón | (16, 0) | Sale por x < 0 y rodea |

### 3. En Gazebo, boyas de percepción (dinámicas)

Con `site:=sim` (o `tests/s01_libre`), publica objetos a mano en `usv/perception/objects`.

**a) Frame NED.** Boya en (10, 0) del mundo, goal (20, 0) → el path la rodea.
```bash
ros2 topic pub --once usv/perception/objects usv_interfaces/msg/ObjectList \
  "{header: {frame_id: 'ned'}, obj_list: [{x: 10.0, y: 0.0, radius: 0.3, uuid: 'n1'}]}"
```

**b) Frame del bote y signo de y.** Primero mira el yaw: `ros2 topic echo --once usv/loc | grep yaw`.
Publica una boya 8 m a proa y 2 m a estribor:
```bash
ros2 topic pub --once usv/perception/objects usv_interfaces/msg/ObjectList \
  "{header: {frame_id: 'base_link'}, obj_list: [{x: 8.0, y: 2.0, radius: 0.3, uuid: 'b1'}]}"
```
Con yaw ≈ 0 (proa al Norte), la boya debe quedar en NED ≈ (8, 2), **a la derecha** del bote.
Un goal en (8, 2) debe dar STOP (goal encima de la boya). Si en cambio el STOP sale con (8, −2),
el signo de y está invertido entre percepción y el planner.

**c) Memoria (sin timeout).** Después de (a), deja de publicar: el path **sigue** rodeando la boya.

**d) Olvido (timeout).** `site:=tests/s12_timeout` (`obstacle_timeout: 5.0`). Publica (a) una vez:
el path la rodea y, unos 5 s después, vuelve a ser recto.

**e) Asociación sin uuid.** Publica (a) sin `uuid` y luego con `x: 10.3, y: 0.2`. Son la misma boya
(están a < `merge_distance`), así que el path apenas se mueve. Con `x: 13.0` aparece una segunda boya.

**f) Replaneo en movimiento.** Con LOS siguiendo el path, publica una boya (frame `ned`) sobre el path
delante del bote: en el siguiente tick (≤ 1/`publish_rate` s) el path debe cambiar para rodearla.

## Debug, en este orden

1. ¿No sale path? → lee `STOP: ...`. Lo más común es que el bote o el goal estén fuera del mapa o en zona letal.
2. ¿El path atraviesa boyas o se ve raro? → convención de ejes (NED vs ENU) entre quien publica y quien consume.
3. ¿No pasa entre boyas? → `safety_margin`, `resolution` e inflado. Una puerta estrecha cuesta mucho y A* puede preferir rodearla:
   para obligar a cruzarla, manda el centro de la puerta como goal intermedio.
4. ¿El path brinca entre corridas? → el datum no es fijo.
5. ¿Una clase falla? → `./build/usv_control/test_<clase>` para aislarla.