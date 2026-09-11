# Corrección del origen del láser y evaluación de MIT Stata

Esta entrega contiene los archivos modificados respecto de `beluga2.5(4).zip`.
Copiar su carpeta `beluga2.5` encima de `~/ros2_ws/src/beluga2.5`.
Se conserva el resto del proyecto, incluidos los cambios anteriores de PF,
hipótesis, tracking y loop closure. No hace falta descargar otra versión de GitHub.

## Compilar y probar

Después de extraer el ZIP sobre el repositorio:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select belugaslam_core belugaslam_node belugaslam_example belugaslam_benchmark --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
bash src/beluga2.5/tools/run_mit_frame_test.sh
```

El último comando usa el bag y GT en:

- `belugaslam_example/bags/mit_rosbag/mit_bag_ros2`
- `belugaslam_example/bags/mit_rosbag/gt.txt`

Requiere el `.db3` real, que no estaba en el ZIP recibido. El script detecta si
solamente está el `metadata.yaml`. Reproduce el bag completo con los parámetros
MIT existentes (5–30 partículas), guarda la salida en una carpeta nueva dentro
de `mit_frame_runs`, espera la exportación final y ejecuta la evaluación.
El cierre de launch permite hasta 120 segundos antes de enviar SIGTERM al nodo,
para dar tiempo a su optimización final y exportación.

Para usar otras rutas:

```bash
bash src/beluga2.5/tools/run_mit_frame_test.sh /ruta/al/bag /ruta/al/gt.txt base_laser_link
```

**El tercer argumento declara el punto físico que representa el GT.** El script
usa `base_laser_link` para el GT del láser. Si tu GT fue convertido previamente
a la base del robot, usá `base_footprint`. El archivo numérico `gt.txt` no
identifica por sí mismo su frame: esa procedencia debe comprobarse. Si la TF
registrada tiene otro nombre de frame, el evaluador informa los nombres reales
y rechaza la conversión solicitada. No adivina otra TF ni ajusta un desplazamiento
contra el ground truth.

## Qué cambia

1. El nodo transforma tanto los impactos como los orígenes del láser al marco
   `base_footprint`. Usa la TF real del scan. Para los impactos aplica la TF 3D
   antes de proyectar a XY; el estado SLAM sigue siendo 2D.
2. Cuando hay odometría disponible hasta el final del scan, el deskew transforma
   **cada impacto y su origen con el mismo movimiento**, conservando el índice
   temporal original incluso al descartar rangos inválidos. Se mantiene el
   fallback anterior sin deskew cuando falta la TF final, con aviso en el log.
3. El core lleva los orígenes a cada submapa. La grilla crece para incluirlos y
   los rayos que marcan espacio libre parten de esos orígenes. La limpieza del
   footprint continúa centrada en la base. Se mantienen prioridad de impactos
   y una actualización por celda por scan.
4. `/best_pose`, el estado de las partículas, los nodos y las exportaciones
   originales siguen representando la base. Junto a `final_trajectory_path` se
   genera `<ruta>.frames.csv`: timestamp exacto, nombres de frames, TF planar
   base<-scan y estado del deskew. Si solo se configura
   `optimized_trajectory_path`, el sidecar se genera junto a ese archivo.
   Si se configuran ambos, se genera junto al CSV final.
5. `evaluate_frame_trajectory.py` aplica, en cada timestamp exacto:
   `T_map_scan = T_map_base * T_base_scan`. Después alinea los marcos globales
   mediante SE(2), sin escala y sin ajustar la calibración. También admite
   evaluar en el frame base sin aplicar la TF. Rechaza extrínsecas faltantes,
   datos no finitos o cambios de nombres de frames durante una corrida.

Los callers offline que no proporcionan orígenes conservan la convención
anterior de sensor en el origen del robot. No se ha supuesto un offset numérico
del PR2 ni se ha convertido el GT proporcionado a ciegas.

## Archivos de salida

Cada corrida crea:

- `final.csv`: trayectoria de la base, columnas online y optimized.
- `final.csv.frames.csv`: TF medidas por scan; debe conservarse con la trayectoria.
- `performance.csv` y `launch.log`: diagnóstico de recepción, TF y tracking.
- `evaluation/estimated_base.tum`: exportación de la base.
- `evaluation/estimated_reference_frame.tum`: exportación en el cuerpo del GT,
  antes de la alineación global.
- `evaluation/metrics.json`: RMSE XY y yaw, RPE, cobertura y frames declarados.

La evaluación automática usa la columna `optimized`. Para evaluar también
la columna online del mismo CSV:

```bash
python3 tools/evaluate_frame_trajectory.py \
  --reference belugaslam_example/bags/mit_rosbag/gt.txt \
  --reference-frame base_laser_link \
  --estimate 'mit_frame_runs/run_XXXXXXXX/final.csv#online' \
  --output-dir mit_frame_runs/run_XXXXXXXX/evaluation_online
```

Reemplazar `run_XXXXXXXX` por la carpeta que imprime el script. Este ejemplo se
ejecuta desde la raíz del repositorio. La columna online conserva la definición
previa del exportador final; para una comparación con `/best_pose` grabado hay
que respetar la diferencia entre esa salida y la trayectoria del grafo elegido.

Para convertir un TUM optimizado cuando también se había exportado el CSV,
usar `--extrinsics /ruta/final.csv.frames.csv` explícitamente. Las trayectorias
viejas sin sidecar necesitan extrínsecas verificadas; este cambio no puede
reconstruirlas del GT. `compute_rmse.py` y `evaluate_trajectory.py` siguen siendo
evaluadores genéricos: para MIT usá el nuevo script o un TUM ya convertido al
mismo cuerpo que la referencia.

## Validación y límites

- Aprobadas las 13 pruebas C++ independientes de `tools/run_standalone_tests.sh`.
- Aprobadas las 89 pruebas Python, incluidas 7 nuevas sobre TF y evaluación.
- El test de grilla conserva 2400 comprobaciones de equivalencia de la API
  anterior y agrega casos de sensor desplazado, distintos orígenes por rayo,
  prioridad de impactos y rechazo de tamaños incompatibles antes de mutar.
- Agregadas 3 pruebas de integración de submapas: offset, rotación/crecimiento y
  equivalencia para un sensor sin desplazamiento. Se ejecutan con ROS/Sophus y
  GTest instalados; no se pudieron ejecutar en el entorno de esta entrega.
- Verificados sintaxis Python/Bash, el CLI de evaluación con datos sintéticos
  y limpieza del diff.

No se compiló el nodo ROS completo ni se midió RMSE sobre MIT aquí: el entorno
no dispone de ROS 2 y el ZIP no incluye el bag. Las pruebas sintéticas verifican
geometría y regresiones; no demuestran una mejora de precisión en MIT.

Para ejecutar la integración después de compilar con tests habilitados:

```bash
cd ~/ros2_ws
colcon test --packages-select belugaslam_core --ctest-args -R submap_graph_test --output-on-failure
colcon test-result --verbose
```

Para revisar resultados de esta corrida, compartir la carpeta `mit_frame_runs/run_...`.

Referencia de procedencia del GT: https://projects.csail.mit.edu/stata/downloads.html
