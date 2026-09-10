# Puntos 1 + 2: matcher Ceres y prior de odometría

Esta entrega conserva el punto 1 (probabilidades interpoladas, Ceres y filtro
espacial) y añade el punto 2: covarianza de movimiento de 3×3 para regularizar
la pose local. También conserva la actualización bayesiana anterior.

Para activar ambos cambios en ROS utiliza:

```bash
tracking_matcher:=probability_ceres tracking_prior_mode:=odometry
```

Los valores predeterminados siguen siendo `distance` y `fixed`, para mantener
las ejecuciones anteriores reproducibles. Los comandos siguientes seleccionan
explícitamente los puntos 1 y 2. La ventana deslizante del punto 3 sigue pendiente.

## Instalar y compilar

El ZIP completo sustituye la carpeta `beluga2.5` del proyecto. El ZIP de archivos
modificados contiene todos los cambios acumulados respecto a `beluga2.5(2).zip`:
copia su carpeta `beluga2.5/` sobre la tuya conservando las rutas. Necesita los
archivos originales restantes. Conserva tus cambios propios antes de sustituirlos.

En tu workspace de ROS 2 Humble:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select belugaslam_core belugaslam_node belugaslam_example belugaslam_benchmark \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select belugaslam_core --event-handlers console_direct+
colcon test-result --verbose
```

Ejecuta el replay después de que compilación y pruebas terminen correctamente.
Se entregan fuentes; no se incluyen binarios ROS compilados.

## Comparar punto 1 frente a puntos 1 + 2

Desde `~/ros2_ws`, con `install/setup.bash` cargado:

```bash
python3 src/beluga2.5/tools/run_accuracy_replay.py \
  --prior-comparison --tracking-matcher probability_ceres \
  --frontend-pose-mode frontend --loops off --hypotheses 1 \
  --particles 300 --seed 42
```

Produce dos corridas nuevas, `ceres_fixed` y `ceres_odometry`, con el mismo
matcher Ceres, filtro, datos, partículas, semilla y parámetros restantes. Solo
cambia el modo de prior. El primer ensayo desactiva loops y PGO para aislar el
frontend. Al terminar, imprime `SEND THIS ZIP` con el ZIP de resultados.

Para medir RMSE, añade `--reference /ruta/absoluta/referencia.tum` y
`--reference-clock acquisition` o `logger`, según el reloj de esa referencia.
La referencia se usa únicamente después del SLAM. Los resultados separan
trayectoria online y optimizada, y verifican cobertura de timestamps común.
Si solo quieres comprobar que arranca, añade `--max-scans 200`; esa corrida
parcial no demuestra una mejora de RMSE en el recorrido completo.

Para comparar después con los loops bayesianos activos:

```bash
python3 src/beluga2.5/tools/run_accuracy_replay.py \
  --prior-comparison --tracking-matcher probability_ceres \
  --frontend-pose-mode frontend --loops belief --loop-update-mode bayes \
  --hypotheses 4 --particles 300 --seed 42
```

Para una ejecución ROS con visualización:

```bash
ros2 launch belugaslam_example intel_dataset_belugaslam.xml \
  record_bag:=false tracking_matcher:=probability_ceres \
  tracking_prior_mode:=odometry frontend_pose_mode:=frontend \
  max_particles:=300 max_hypotheses:=4 random_seed:=42
```

## Qué hace el punto 2

El centro del prior sigue siendo la pose local anterior propagada por odometría.
Se cambia su covarianza, no se sustituye la pose por la partícula ganadora ni
por una media nueva de propuestas. Con `frontend_pose_mode:=frontend`, el PF
no selecciona directamente la pose que se inserta en el submapa.

Para cada predicción se calcula:

\[
\Sigma = F D F^\top + G Q G^\top,
\qquad D=\operatorname{diag}(\sigma_t^2,\sigma_t^2,\sigma_\theta^2).
\]

`D` representa un suelo configurable de incertidumbre de la pose local anterior.
`Q` modela el ruido independiente de rotación inicial, desplazamiento y rotación
final del incremento de odometría. `F` y `G` son sus Jacobianos en el marco del
submapa usado para matching. Se conservan términos cruzados entre traslación y
orientación; el resultado se transforma con la orientación de cada hipótesis.

Para desplazamiento `d` y rotaciones `r1`, `r2`, las varianzas de movimiento son:

\[
v_1=\alpha_1\bar r_1^2+\alpha_2d^2,\quad
v_d=\alpha_3d^2+\alpha_4(\bar r_1^2+\bar r_2^2),\quad
v_2=\alpha_1\bar r_2^2+\alpha_2d^2.
\]

Las rotaciones usadas para el ruido contemplan marcha atrás, evitando interpretar
una traslación hacia atrás como dos giros ruidosos de π. Bajo el umbral de
traslación se trata el giro como giro en el sitio; un giro puro de π mantiene
su incertidumbre. El nodo toma los coeficientes `alpha1..alpha4` y el umbral
de los mismos parámetros con los que construye el modelo de movimiento del PF.
El replay utiliza sus coeficientes existentes `.1, .05, .1, .05` y umbral `.01 m`.

La contribución del prior al objetivo es ahora
`0.5 * tracking_prior_information_scale * deltaᵀ Sigma⁻¹ delta`.
Se aplica mediante Cholesky, sin invertir explícitamente la matriz. Ceres
y el optimizador de distancias admiten este mismo coste y sus derivadas.

Un prior más fuerte puede dificultar encontrar una buena inicialización.
Por eso, en modo adaptativo también se conserva la inicialización del punto 1.
Ambas candidatas se evalúan con el mismo objetivo final de ocupación y la misma
covarianza adaptativa antes de resolver con Ceres. Se mantienen los límites de
corrección y controles de solapamiento; el arranque adicional añade tiempo de CPU.

Esta es una aproximación local de un paso: `D` se reinicia para cada predicción.
No es una covarianza posterior calibrada, una propagación recursiva de toda la
trayectoria ni una optimización por ventana. La búsqueda amplia de recuperación
conserva su prior fijo más débil; las confirmaciones posteriores usan el matcher
y prior seleccionados. El likelihood del PF y la evidencia de loops conservan
su formulación anterior. Cambiar la trayectoria puede cambiar los mapas y las
decisiones futuras, aunque esas fórmulas no se modifiquen.

## Parámetros y diagnóstico

| Parámetro ROS | Opción de replay | Valor inicial |
|---|---|---|
| `tracking_prior_mode` | `--tracking-prior-mode` | `fixed`; nuevo modo `odometry` |
| `tracking_odom_translation_sigma` | `--tracking-odom-translation-sigma` | `0.10` m |
| `tracking_odom_rotation_sigma` | `--tracking-odom-rotation-sigma` | `0.05` rad |
| `tracking_prior_information_scale` | `--prior-information-scale` | `1.0` |

Los dos nuevos sigmas son valores experimentales de partida, no calibrados con
tu recorrido. Aumentarlos permite más corrección LiDAR; reducirlos aumenta la
confianza en odometría y puede introducir sesgo. Los sigmas fijos existentes
siguen definiendo la variante `fixed` y su inicialización auxiliar.

`tracking.csv` añade `tracking_prior_mode`, `prior_evaluated` y los seis elementos
independientes `prior_cov_xx`, `prior_cov_xy`, `prior_cov_xyaw`, `prior_cov_yy`,
`prior_cov_yyaw`, `prior_cov_yawyaw`. Describen la covarianza efectiva del prior
usado, en el marco del submapa y dividida por `tracking_prior_information_scale`.
En bootstrap `prior_evaluated=0`. No son la covarianza de salida del PF.

## Validación y límites

- Pasan 13 programas C++ independientes, con 106 comprobaciones del prior nuevo.
  Incluyen transporte de marco, blanqueo, marcha atrás, giro puro, Jacobianos y
  comparación con 80.000 muestras del modelo de movimiento no lineal.
- Pasan 76 pruebas Python, incluidas las comparaciones que conservan el punto 1.
- Pasan 9 casos con Ceres nativo mediante `pyceres`, usando el kernel y la
  inicialización C++ de producción. Los cinco casos anteriores con prior fijo
  conservan exactamente sus poses. Los casos nuevos comprueban el prior adaptativo,
  duplicación de rayos, cruce de ±π y un pasillo con un eje no observable.
- El test del prior pasa AddressSanitizer y UndefinedBehaviorSanitizer; el entorno
  no permite LeakSanitizer, que se desactiva para ese ensayo.
- Se amplían las pruebas del adaptador C++ Ceres y del core, pero no se han
  compilado/ejecutado aquí: faltan las dependencias completas C++/ROS. Los ensayos
  mediante `pyceres` no reemplazan esas pruebas de integración.

Los detalles están en `validation/ADAPTIVE_PRIOR_VALIDATION.json` y sus logs.
Las comprobaciones independientes se repiten con `bash tools/run_standalone_tests.sh`.
Todavía no se ha medido una mejora de RMSE de SLAM completo frente al punto 1
o frente a Cartographer. El comparador genera los archivos para comprobarlo.
