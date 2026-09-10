# Beluga-MH: submapeo con grafo bipartito

**Latest revision:** see [LATE_RUN_REVIEW.md](LATE_RUN_REVIEW.md) for confirmed
tracking recovery, immediate reference handover, cache retention and PGO scheduling.
The descriptions below document the preceding implementation.


The current performance revision preserves insertion semantics and uses generation markers, cached publication and bounded parallel matching; see [PERFORMANCE.md](PERFORMANCE.md).
Esta versión reemplaza el pose graph directo `submapa -> submapa` por una
estructura inspirada en Cartographer:

- cada scan aceptado por el filtro de movimiento crea un `TrajectoryNode`;
- cada inserción genera una restricción `submapa -> nodo` de tipo intra-submapa;
- un loop closure genera otra restricción `submapa histórico -> nodo` de tipo
  inter-submapa;
- Ceres optimiza conjuntamente las poses de nodos y submapas;
- las aristas entre nodos consecutivos conservan el prior de la trayectoria local.

El ciclo de vida de los submapas es el de Cartographer y lo gobierna un solo número.
Se crea un submapa; cuando recibió `submap_num_range_data` scans se crea el siguiente,
y a partir de ahí los dos reciben todos los scans; el más viejo se congela al llegar al
doble de esa cuenta, que es exactamente cuando el más nuevo llegó a la cuenta y arranca
el que sigue. Cada submapa congelado vio `2 * submap_num_range_data` scans y el solape
entre consecutivos es de la mitad.

Las grillas de submapa crecen sobre demanda (`LogOddsGrid::grow_to_include`), así que el
ciclo de vida no depende de ninguna extensión espacial: depende solo de la cuenta de
scans, como en Cartographer. Cada hipótesis tiene como máximo dos submapas activos y
todo scan aceptado se inserta en los dos.

## Tracking e inserción

- Cada hipótesis predice su pose continua con odometría y la corrige contra un
  único submapa de referencia: el más viejo del par de inserción.
- Las poses candidatas se transforman al frame del submapa con
  `T_submap_robot = inverse(T_world_submap) * T_world_robot`; se consulta su
  grilla nativa y se devuelve la pose corregida al frame global. No hay grilla
  compuesta de tracking ni límite espacial heredado del mapa de publicación.
- El submapa más nuevo también recibe cada inserción aceptada. No se combina
  su score con el viejo: los scans del solape están correlacionados y esa mezcla
  requeriría una decisión de modelado adicional.
- El submapa que acaba de congelarse puede seguir siendo referencia hasta la
  próxima inserción aceptada. Su ID se resuelve dentro de cada hipótesis, de modo
  que las copias y correcciones de pose no dejen un puntero a la rama anterior.
- El motion filter se evalúa **después** del scan matching. Se acepta el primer
  scan y luego cuando se supera la distancia, el ángulo o el tiempo desde la
  última inserción. Igualar el umbral todavía se considera similar. Un timestamp
  que retrocede reinicia el intervalo de inserción; no reinicia todo el SLAM.
- Un scan filtrado no modifica las grillas, los contadores, los nodos ni la
  referencia del filtro. El tracking y las partículas sí se actualizan.
- `keyframe_max_time` usa segundos del timestamp del scan, no tiempo de pared.
  La API core es `update_occupancy_grid(scan, time_seconds)`.
- La callback ROS procesa el primer scan válido incluso estando quieto. Ya no
  descarta scans antes del tracking por los antiguos `min_update_*`; esos
  parámetros se aceptan por compatibilidad pero se ignoran con un aviso.

## Controles de costo

- El filtro de movimiento acota tanto las inserciones como el tamaño del grafo:
  cada scan aceptado crea un nodo y se inserta en los submapas activos. Por eso
  `submap_num_range_data` cuenta inserciones aceptadas, no scans recibidos.
- Cada scan node almacena como máximo `max_points_per_scan_node` endpoints.
- Los datos del scan son inmutables y se comparten entre hipótesis.
- Cuando un nodo ya no pertenece a ningún submapa activo se libera su point cloud;
  su pose y sus restricciones pequeñas permanecen en el grafo.
- Cada evento recupera como máximo `loop_max_candidates` submapas.
- Cada evento usa el último nodo insertado en el submapa query, para fijar
  una misma asociación temporal al compararla entre hipótesis.
- La registración usa distance fields precalculados y refinamiento con beam de 8,
  no una búsqueda exhaustiva de cuatro grillas completas.
- `loop_max_branches` y `max_hypotheses` acotan el branching.
- La optimización se ejecuta periódicamente en cada hipótesis, antes de verificar
  eventos de loop y en las pruebas temporales de candidatos.

## Parámetros iniciales para Intel Research Lab

| Parámetro | Valor inicial |
|---|---:|
| `submap_num_range_data` | 15 (congela a 30) |
| `keyframe_min_translation` | 0.15 m |
| `keyframe_min_rotation` | 0.0873 rad |
| `keyframe_max_time` | 5.0 s |
| `max_points_per_scan_node` | 180 |
| `loop_recent_submaps` | 5 |
| `loop_max_candidates` | 6 |
| `loop_candidate_distance` | 10 m |
| `loop_search_translation` | 3 m |
| `loop_search_rotation` | 0.7 rad |
| `loop_min_score` | 0.55 |
| `loop_min_overlap` | 0.35 |

Los umbrales de score y overlap deben calibrarse con verdaderos positivos y
negativos del dataset; no son constantes universales.

Al cambiar la frecuencia de inserción, `15` implica una extensión recorrida mayor
que antes; es un punto inicial para comparar, no un valor óptimo demostrado.
Ahora se hace tracking en cada scan válido: medir latencia y acumulación de la
cola al reproducir el dataset. También cambia la frecuencia de propagación y de
actualización de pesos del PF, por lo que los parámetros de ruido y likelihood
pueden necesitar recalibración.

## Pruebas

`submap_tracking_test` comprueba el tracking fuera del antiguo límite global,
el uso exclusivo de la referencia, la ausencia de escritura durante matching,
el filtro completo, la acumulación desde la última inserción, el wrap angular,
el solape, el traspaso de referencia y las copias entre hipótesis.
`motion_filter_test` comprueba los tres umbrales y sus bordes sin dependencias
de ROS. `submap_graph_test` conserva las pruebas existentes de crecimiento,
cropping, inserción y copy-on-write.

Fuentes de referencia:

- [Cartographer LocalTrajectoryBuilder2D](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc)
- [Cartographer MotionFilter](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/motion_filter.cc)

## Diferencia deliberada con Cartographer

La topología del grafo local es parecida, pero Beluga-MH conserva varias copias
pequeñas del estado optimizable cuando una asociación de loop es ambigua. Las
grillas congeladas y los point clouds se comparten; las hipótesis difieren en
poses, restricciones inter-submapa, submapas activos copy-on-write y partículas.

Se conserva el matcher discreto de endpoints de Beluga y su actualización de
pesos. No es el matcher continuo Ceres de Cartographer, ni una demostración de
que los pesos sean un posterior calibrado. El verificador añadido marginaliza compatibilidades de trayectoria sobre las
hipótesis, pero su calibración sigue siendo una cuestión experimental.

Loop closure y PGO ahora están habilitados por defecto. Para aislar nuevamente
submapping, usar `enable_loop_closure:=false enable_pgo:=false` en el launch.
Ver `LOOP_PGO.md` para el verificador sobre todas las hipótesis, la separación de
poses locales/globales, las correcciones rígidas de los submapas activos y las
limitaciones probabilísticas.
