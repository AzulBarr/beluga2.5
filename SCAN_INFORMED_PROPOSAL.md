# Propuesta de movimiento informada por el frontend

Activada por defecto para probar la modificación solicitada. Implementación sobre
`beluga2.5(5).zip`, sin usar ground truth en el SLAM.

**Resultado de esta revisión:** en Intel completo, semilla 42, la variante final
no mejoró el RMSE frente a la referencia incluida: online 0.220191 m frente a
0.175590 m, y después de PGO 0.083919 m frente a 0.080840 m con odometría. Se
entrega como cambio experimental con pesos verificados, no como mejora de
precisión demostrada. `scan_informed_proposal:=false` conserva la propuesta
original. Ver el informe y los replays completos al final de este documento.

## Qué cambia

Cada hipótesis ejecuta su frontend normal (predicción por odometría, scan matching,
refinamiento GN/LM o Ceres según la configuración, e ICP si está activado y se
acepta). Antes de inserción, loop closure y PGO se calcula

```
F_h = inverse(local_pose_anterior_h) * local_pose_frontend_actual_h
O   = inverse(odometria_anterior) * odometria_actual
```

Las dos poses de `F_h` se leen en el mismo marco global vigente. No se usa la
hipótesis seleccionada globalmente ni se incorpora una corrección de PGO como
movimiento. Un cambio de submapa se resuelve a través de sus poses globales.

Para cada propuesta original `D_odom ~ p_odom`, se obtiene su ruido relativo
`epsilon = inverse(O) * D_odom`. La rama informada propone

```
x_nueva = x_ancestro * F_h * epsilon
```

Conserva los parámetros `alpha1`–`alpha4`, las correlaciones del modelo diferencial,
la distancia con signo y el ruido angular periódico. No reemplaza el modelo de
movimiento por una gaussiana cartesiana aproximada ni agrega sigmas nuevos.

La distribución efectiva es una mezcla de frontend y odometría, con hasta 80%
frontend por **propuesta**. La fracción se reduce cuando el incremento del frontend
entra en las colas del prior de movimiento. La selección es aleatoria independiente;
no fuerza una proporción exacta en cada población finita. Conserva todas las poses ancestrales: no
reubica las partículas en una única pose del frontend.

## Corrección de importancia

En coordenadas relativas al ancestro, para un incremento candidato `D`:

```
q_front(D) = p_odom(O * inverse(F_h) * D)
q(D)       = (1-a) * p_odom(D) + a * q_front(D)
r(D)       = log p_odom(D) - log q(D)
ell(D)     = log L(scan | D, mapa_previo) + r(D)
```

La transformación de SE(2) tiene Jacobiano de Haar uno. La densidad original del
modelo diferencial se evalúa con sus dos preimágenes polares (distancias positiva
y negativa), el Jacobiano `1/|distancia|` y las normales angulares envueltas. El
muestreador y el evaluador consumen los mismos parámetros del modelo original.
El cálculo de las normales envueltas trunca únicamente colas numéricamente
irrelevantes; no aproxima la densidad por el ángulo más cercano.

Antes del muestreo se fija la fracción por hipótesis:

```
a_h = a_config * exp(0.25 * min(0, log p_odom(F_h) - log p_odom(O)))
```

Es una regla heurística de compatibilidad, calculada sin ground truth ni muestras
recicladas del paso actual. Para gaussianas de igual covarianza tiene la escala
de su afinidad de Bhattacharyya; no se afirma que sea esa afinidad exacta para el
modelo diferencial no gaussiano. Evita gastar la mayoría de propuestas en una
región que la corrección de importancia va a rechazar. Si subdesborda a cero, el
paso conserva la propuesta de odometría. El denominador `q` utiliza exactamente
este `a_h`. Con `scan_proposal_adapt_to_prior:=false` se usa la fracción fija.

Para las `K = motion_proposal_samples` propuestas de un ancestro:

1. Se elige una propuesta proporcionalmente a `exp(ell_j)`.
2. El incremento de peso del ancestro es `logsumexp(ell_j) - log(K)`.
3. Se actualizan y normalizan los pesos condicionales dentro de cada hipótesis.
4. Su normalizador actualiza la masa de la hipótesis, según las reglas existentes.

La media de las propuestas es esencial: ni el score máximo ni el score de la
propuesta elegida constituyen la evidencia predictiva. También se usa `ell` en
la estimación de pose/covarianza a partir de todas las propuestas.

Para cualquier función `f`, el peso medio multiplicado por `f` de la propuesta
seleccionada tiene esperanza `integral L(D) p_odom(D) f(D) dD`. Esta identidad
explica por qué usar el scan en la propuesta no agrega una segunda medición.

Durante una validación bayesiana de loop, se sustituye **solamente** `log L` por
el likelihood del mapa histórico congelado y se conserva `r(D)`. No se multiplica
además por el score del mapa activo. Sin cobertura histórica común suficiente,
se usa `q=p_odom`, con corrección exactamente cero y sin evidencia artificial.

Con `a_h <= 0.8`, `p_odom/q <= 5`. Es una cota de la mezcla, no un recorte de pesos.
Las probabilidades pequeñas se conservan en logaritmos. Un candidato con peso
cero no se selecciona, pero sigue contando en el divisor `K`.

## Respaldo y límites

Se conserva la propagación original cuando:

- el parámetro está desactivado o `a=0`;
- no hay frontend previo válido o el tracking está débil/rechazado/en recuperación;
- falta evidencia común durante la ventana bayesiana;
- el prior diferencial es singular o prácticamente singular: reposo, ciertos
  giros puros, ruido cero o desviaciones de sus variables latentes <= 1e-9;
- no existe una actualización de movimiento pendiente compatible con la población.

No se inventa ruido para convertir un prior determinista en uno continuo. Si la
odometría afirma movimiento exactamente cero, esta revisión no permite que una
propuesta continua contradiga esa masa de Dirac: conserva el comportamiento
anterior. Un modelo explícito de deslizamiento/reposo requeriría otro cambio.

La corrección es respecto del modelo de movimiento y del potencial de medición
**existentes**. El likelihood field usa un score robusto y `effective_beams`; esto
no lo convierte en una densidad calibrada de scans completos. Tampoco hace que
los mapas compartidos por hipótesis equivalgan a un RBPF con un mapa independiente
por partícula.

En modo `frontend`, la pose publicada sigue siendo la pose continua de la
hipótesis. Este cambio afecta el posterior de partículas, sus momentos y la
inferencia entre hipótesis; por sí solo no garantiza menor RMSE de la pose
publicada. Los modos opcionales `proposal_mean` y `proposal_seed` mantienen sus
reglas anteriores. La propuesta actual utiliza el frontend antes de esos
readouts, evitando una dependencia circular del mismo conjunto de muestras.

## Parámetros

| Parámetro | Por defecto | Significado |
|---|---:|---|
| `scan_informed_proposal` | `true` | Activa la propuesta informada con respaldo |
| `scan_proposal_fraction` | `0.8` | Fracción máxima de la rama frontend; rango `[0, 0.95]` |
| `scan_proposal_adapt_to_prior` | `true` | Reduce la fracción cuando el frontend contradice fuertemente al prior |
| `motion_proposal_samples` | `8` | Propuestas por ancestro; reposo conserva una |
| `alpha1`–`alpha4` | Sin cambios | Ruido del modelo de movimiento original |

Disponibles en el nodo, launch general y launches Intel/MIT. Los ajustes de ICP,
matcher, submapping, loop closure y PGO conservan su configuración original.

En `tracking_diagnostics_path`, las nuevas columnas son `scan_proposal_status`,
`scan_proposals`, `frontend_proposals`, `scan_proposal_fraction`,
`log_p_over_q_min`, `log_p_over_q_max`.
`scan_proposal_fraction` registra la fracción efectiva de esa hipótesis y scan.
`mixture` indica que se usó la nueva propuesta; los otros estados explican el
respaldo. Los mínimos/máximos son del logaritmo de `p/q`, antes del likelihood.

## Instalar y ejecutar

Extraer el ZIP en `~/ros2_ws/src`, donde ya existe `beluga2.5`. El ZIP contiene la
carpeta `beluga2.5/` y no incluye `.git`, `build`, `install` ni `log`.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select belugaslam_core belugaslam_node belugaslam_example belugaslam_benchmark --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select belugaslam_core
colcon test-result --verbose
```

MIT, nueva propuesta:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch belugaslam_example mit_rosbag_belugaslam.xml \
  record_bag:=false random_seed:=42 \
  scan_informed_proposal:=true \
  tracking_diagnostics_path:=/tmp/mit_scan_proposal_on.csv \
  optimized_trajectory_path:=/tmp/mit_scan_proposal_on.tum
```

Comparación, después de detener el proceso anterior:

```bash
ros2 launch belugaslam_example mit_rosbag_belugaslam.xml \
  record_bag:=false random_seed:=42 \
  scan_informed_proposal:=false \
  tracking_diagnostics_path:=/tmp/mit_scan_proposal_off.csv \
  optimized_trajectory_path:=/tmp/mit_scan_proposal_off.tum
```

Intel usa `intel_dataset_belugaslam.xml` con los mismos parámetros de propuesta.
El replay independiente también acepta `--scan-informed-proposal on|off`:

```bash
python3 src/beluga2.5/tools/run_accuracy_replay.py \
  --particles 30 --hypotheses 4 --loops belief \
  --frontend-pose-mode frontend --scan-informed-proposal on
```

Repetir con `off`, mismos ajustes y misma secuencia completa. El cambio de
propuesta consume números aleatorios distintos; la misma semilla da
reproducibilidad por modo, no las mismas muestras entre modos. Comparar varias
semillas para concluir sobre RMSE y fallos. No usar un replay parcial como
prueba de superioridad de precisión.

Los resultados efectivamente ejecutados en esta revisión se documentan en
`validation/scan_informed/RESULTS.md`.
