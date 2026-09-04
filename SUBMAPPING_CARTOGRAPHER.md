# Beluga-MH: submapeo con grafo bipartito

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

## Controles de costo

- El filtro de movimiento acota el tamaño del grafo, no la calidad de las grillas:
  todos los scans se insertan en los submapas activos, pero solo los keyframes crean
  nodo de trayectoria y restricciones.
- Cada scan node almacena como máximo `max_points_per_scan_node` endpoints.
- Los datos del scan son inmutables y se comparten entre hipótesis.
- Cuando un nodo ya no pertenece a ningún submapa activo se libera su point cloud;
  su pose y sus restricciones pequeñas permanecen en el grafo.
- Cada evento recupera como máximo `loop_max_candidates` submapas.
- De cada submapa query se prueban hasta tres nodos representativos.
- La registración usa distance fields precalculados y refinamiento con beam de 8,
  no una búsqueda exhaustiva de cuatro grillas completas.
- `loop_max_branches` y `max_hypotheses` acotan el branching.
- La optimización se dispara al agregar una restricción inter-submapa, no en cada scan.

## Parámetros iniciales para Intel Research Lab

| Parámetro | Valor inicial |
|---|---:|
| `submap_num_range_data` | 15 (congela a 30) |
| `keyframe_min_translation` | 0.15 m |
| `keyframe_min_rotation` | 0.0873 rad |
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

## Diferencia deliberada con Cartographer

La topología del grafo local es parecida, pero Beluga-MH conserva varias copias
pequeñas del estado optimizable cuando una asociación de loop es ambigua. Las
grillas congeladas y los point clouds se comparten; las hipótesis difieren en
poses, restricciones inter-submapa, submapas activos copy-on-write y partículas.
