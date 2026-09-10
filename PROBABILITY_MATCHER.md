# Prueba de precisión de pose: probability_ceres

Esta revisión incorpora un refinamiento local Ceres sobre probabilidades de ocupación
interpoladas. Incluye la actualización bayesiana jerárquica de la entrega anterior.
Es una variante experimental para comparar con `distance`, que sigue siendo el
valor predeterminado. Los comandos de abajo activan explícitamente la nueva variante.

## Compilar en tu workspace ROS 2

Coloca la carpeta `beluga2.5` de este ZIP en `src/` de tu workspace, sustituyendo
la revisión anterior. Conserva una copia de tus cambios propios antes de sustituirla.
No dejes las dos revisiones dentro de `src/`: contienen los mismos paquetes ROS.
Usa el entorno ROS 2 y las dependencias que ya utilizabas para compilar el proyecto.
Ceres ya era una dependencia del backend; no se añade una dependencia de Cartographer.

Desde la raíz del workspace, con ROS 2 cargado:

```bash
colcon build --packages-select belugaslam_core belugaslam_node belugaslam_example belugaslam_benchmark \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select belugaslam_core --event-handlers console_direct+
colcon test-result --verbose
```

Revisa que compilación y pruebas terminen sin errores antes del replay. Esta entrega
contiene fuentes, no binarios ROS compilados en tu plataforma.

## Comparación automática con los mismos datos

Primero aísla el frontend desactivando loops y PGO. Desde la raíz del workspace:

```bash
python3 src/beluga2.5/tools/run_accuracy_replay.py \
  --matcher-comparison --frontend-pose-mode frontend \
  --loops off --hypotheses 1 --particles 300 --seed 42
```

Ejecuta dos procesos nuevos: `distance` y `probability_ceres`. Tienen el mismo
dataset, semilla, población, prior y presupuesto de puntos. La segunda variante
añade el filtrado espacial y el refinamiento Ceres. No se omiten scans.
Al terminar imprime `SEND THIS ZIP` y la ruta al ZIP de resultados.
Para una comprobación rápida puedes añadir `--max-scans 200`; esa ejecución parcial
no sirve para concluir que mejoró el RMSE del recorrido completo.

Para calcular RMSE añade tu referencia TUM:

```bash
python3 src/beluga2.5/tools/run_accuracy_replay.py \
  --matcher-comparison --frontend-pose-mode frontend \
  --loops off --hypotheses 1 --particles 300 --seed 42 \
  --reference /ruta/absoluta/referencia.tum --reference-clock acquisition
```

Usa `--reference-clock logger` solamente si tu referencia usa ese reloj del
archivo Intel. El programa lee la relación temporal del registro; no la ajusta
para mejorar el resultado. La referencia se utiliza después del SLAM.

Después compara el sistema completo, conservando los loops bayesianos:

```bash
python3 src/beluga2.5/tools/run_accuracy_replay.py \
  --matcher-comparison --frontend-pose-mode frontend \
  --loops belief --loop-update-mode bayes --hypotheses 4 --particles 300 --seed 42
```

También puedes añadir la referencia a este comando. Repite la comparación con
otras semillas antes de elegir una configuración. Para atribuir el efecto al
refinamiento sin el filtro, repite el A/B con `--tracking-voxel-size 0`.

El ZIP de resultados contiene trayectorias online y optimizada, mapas, tiempos,
parámetros, hashes de fuentes/binario, cobertura y diagnósticos. Si proporcionas
referencia, incluye `rmse_comparison.json` con APE de posición y orientación,
RPE y asociación temporal común. En `tracking.csv`, las nuevas columnas
`tracking_matcher` y `tracking_points` registran variante y puntos del frontend.
`mean_log_likelihood` conserva la puntuación del campo de distancias usada por
los controles de tracking; no es el coste Ceres.

Para ejecutarlo en ROS con visualización:

```bash
ros2 launch belugaslam_example intel_dataset_belugaslam.xml \
  tracking_matcher:=probability_ceres frontend_pose_mode:=frontend \
  max_particles:=300 max_hypotheses:=4 random_seed:=42
```

## Qué cambia

1. Cada submapa puede construir un campo de probabilidad inmutable a partir de
   sus log odds. Se conserva la información de confianza que el umbral binario
   del campo de distancias descartaba. La caché se invalida al modificar o recortar
   la cuadrícula; las ramas conservan sus propias vistas al escribir.
2. Los puntos del frontend se agrupan en voxels de 5 cm y se usan sus centroides.
   Se conserva el orden de primera aparición y el presupuesto de 180 puntos.
   Este filtrado no consume números aleatorios.
3. El matcher de distancias proporciona una pose inicial. Ceres refina con un
   campo bicúbico y derivadas analíticas respecto a `x`, `y` y `yaw`.
4. Se mantiene el prior original de odometría durante toda la optimización,
   incluso cuando `proposal_seed` aporta otra inicialización. El coste usado
   para aceptar esa propuesta corresponde al matcher seleccionado.
5. Se mantienen los controles de solapamiento, número de inliers y corrección
   máxima. Si el refinamiento no aporta una solución admisible de menor coste,
   se conserva la mejor inicialización admisible. Si el matching se rechaza,
   se devuelve la predicción y sus puntuaciones.

El objetivo de Ceres es:

\[
\frac{1}{2}\sum_{j=1}^{N}\left[\frac{\lambda}{\sqrt{N}}
(1-p(T_xz_j))\right]^2
+\frac{s}{2}\left(\frac{\Delta x^2+\Delta y^2}{\sigma_t^2}
+\frac{\Delta\theta^2}{\sigma_r^2}\right).
\]

`lambda` es `tracking_occupied_space_weight`. `s`, `sigma_t` y `sigma_r`
son los parámetros de prior existentes. La normalización por `sqrt(N)` evita
que duplicar todos los puntos cambie el equilibrio entre datos y prior.
Se usa `DENSE_QR`, un hilo y el límite existente de 20 iteraciones Ceres.
La inicialización por distancias añade trabajo respecto a ejecutar Ceres solo;
por tanto hay que comparar también latencia, no únicamente error.

Las probabilidades observadas se limitan a `[0.1, 0.9]`. Las celdas desconocidas
y exteriores tienen probabilidad de correspondencia `0.1`; esto evita atraer
retornos hacia espacio sin observar. La interpolación también se limita a ese
intervalo, con derivada cero en la zona saturada. Esta es una convención del
frontend, no una probabilidad calibrada para actualizar masas de hipótesis.

El PF conserva su subconjunto de rayos y su likelihood; la evidencia bayesiana
continúa evaluándose contra los snapshots de validación. Las poses y mapas que
se produzcan pueden cambiar, y con ellos la evidencia futura. La búsqueda amplia
de recuperación sigue usando distancias y se confirma en scans posteriores con
el matcher seleccionado. El registro/verificador de loops no se modifica aquí.

## Parámetros

| Parámetro ROS | Opción de replay | Valor inicial |
|---|---|---|
| `tracking_matcher` | `--tracking-matcher` | `distance`; nuevo modo: `probability_ceres` |
| `tracking_occupied_space_weight` | `--tracking-occupied-space-weight` | `5.0` |
| `tracking_voxel_size` | `--tracking-voxel-size` | `0.05` m; `0` desactiva el filtro |
| `tracking_prior_information_scale` | `--prior-information-scale` | `1.0` |

Los parámetros nuevos de filtro/coste solo actúan en `probability_ceres`.
El peso `5.0` es un punto de partida experimental, no una calibración realizada
con tus trayectorias. Evita cambiar simultáneamente partículas, prior, resolución
y loops en el primer A/B: no permitiría identificar el efecto de esta revisión.

La formulación está inspirada en el
[matcher Ceres 2D de Cartographer](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.cc).
Es una implementación propia adaptada a los submapas y al prior de Beluga;
no es una copia exacta del frontend de Cartographer. Su configuración de
[filtros y pesos](https://github.com/cartographer-project/cartographer/blob/master/configuration_files/trajectory_builder_2d.lua)
no se traslada automáticamente porque el predictor y las actualizaciones de
ocupación de este proyecto son diferentes.

## Validación de esta entrega

- Pasan los 12 programas C++ independientes, incluidos 556 controles nuevos
  de interpolación, Jacobiano, normalización y filtrado.
- Pasan 72 pruebas Python, incluida la ejecución del comparador con un binario
  simulado para comprobar argumentos, aislamiento de las corridas y ZIP final.
- Pasan cinco casos con Ceres nativo mediante `pyceres` y el kernel C++ de
  producción compilado: paredes observables, duplicación de rayos, outliers,
  continuidad angular y pasillo con eje no observable.
- El kernel nuevo pasa AddressSanitizer y UndefinedBehaviorSanitizer. Se desactivó
  LeakSanitizer porque el entorno impide inspeccionar `/proc`; no se verificaron fugas.
- Se añaden un ejecutable de prueba del adaptador C++ Ceres y tres pruebas de
  integración con el core. **No se han compilado ni ejecutado aquí**: faltan
  las dependencias C++/ROS completas. Los ensayos con `pyceres` no sustituyen
  esa compilación. Consulta `validation/PROBABILITY_VALIDATION.json` y los logs.

Para repetir las comprobaciones sin ROS:

```bash
bash tools/run_standalone_tests.sh
```

La validación Ceres aislada requiere `numpy` y `pyceres` y se ejecuta desde la
carpeta del proyecto:

```bash
g++ -std=c++17 -O2 -fPIC -shared -I belugaslam_core/include \
  tools/probability_kernel_c_api.cpp -o /tmp/beluga_probability_kernel.so
python3 tools/validate_probability_ceres.py /tmp/beluga_probability_kernel.so
```

No se ha medido RMSE de SLAM completo ni una ventaja frente a Cartographer en
esta entrega. Para afirmar que Beluga lo supera, compara ambos contra la misma
referencia independiente, con iguales sensores, timestamps y reglas de alineación.
Una trayectoria generada por Cartographer puede servir para medir discrepancia,
pero por sí sola no demuestra cuál de los dos tiene menor error real.
