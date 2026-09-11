# Verificación ejecutada — 11 septiembre 2026

## Pruebas

- Compilación del núcleo real de producción con GCC 13, Eigen 3.4, Sophus 1.22.10, Ceres 2.2, TBB y GoogleTest.
- **116/116 pruebas de integración** registradas en CMake (incluidas seis nuevas del cambio).
- **90/90 pruebas de Python**. Se actualizó el contrato del ejecutable de replay para el parámetro on/off.
- **14 ejecutables numéricos independientes**: los 13 anteriores y el nuevo de propuesta informada.
- El nuevo test usa 720.000 muestras para cambio de medida, 180.000 adicionales para fracción adaptativa y 45.000 ensayos multi-try. Comprueba normalización, momentos, evidencia, selección, Jacobiano polar, distancias con signo, periodicidad y la fórmula de composición del ruido.
- Verificación de declaración y forwarding de parámetros en nodo, launch general, Intel y MIT; sintaxis Python/XML y `git diff --check`.

La prueba previa que exigía pesos idénticos al cambiar el prior del frontend se mantiene explícitamente en modo bootstrap (`scan_informed_proposal=false`). En el nuevo modo la propuesta depende deliberadamente del frontend; sus pesos se contrastan contra la fórmula completa de importancia en las pruebas nuevas.

No se compiló ni ejecutó el nodo ROS 2 Humble: este entorno no tiene ROS 2. La compilación y los replays usan la clase de producción BelugaSLAM y dependencias reales, sin mocks de Ceres. El archivo antiguo `loop_refinement_integration_test.cpp` no está registrado en el CMake del ZIP original y no forma parte de la batería registrada.

## Replay completo Intel

Se ejecutaron **13.631 scans** por modo, N máximo 30, H máximo 4, semilla 42, verificación belief/bayes, loop closure y PGO activos, matcher probability_ceres, prior odometry y lectura de pose frontend. ICP conserva el valor false del replay del núcleo; el nodo puede tener su propia configuración de ICP. No se usó referencia en la entrada del SLAM.

| Modo | Poses online / optimizadas | Error máximo de suma de masas | Fracción frontend entre propuestas de pasos con mezcla |
|---|---:|---:|---:|
| Odometría original | 13631 / 13631 | 3.33e-16 | 0.0000 |
| Mezcla fija 80% (primera prueba) | 13631 / 13631 | 3.33e-16 | 0.7997 |
| Mezcla adaptativa (entregada) | 13631 / 13631 | 3.33e-16 | 0.2437 |

Los tres replays finalizaron con `REPLAY_COMPLETE` y PGO final usable. Todos los estados de pose, masas y diagnósticos p/q comprobados fueron finitos, y se respetó la cota de importancia. Los tiempos de estos procesos no son un benchmark comparativo de rendimiento: se ejecutaron bajo cargas diferentes.

## Comparación contra la referencia incluida

La referencia es `corrected_gt_acquisition_clock.txt` del ZIP, con 9.722 poses comunes asociadas a todos los resultados. Se usa alineación rígida SE(2), sin ajuste de escala, tolerancia temporal 0.05 s. Su independencia y exactitud como ground truth no están establecidas aquí. Los números no prueban superioridad frente a Cartographer ni generalización a MIT.

| Modo | RMSE XY online (m) | RMSE XY después de PGO (m) |
|---|---:|---:|
| Odometría original | 0.175590 | 0.080840 |
| Mezcla fija 80% (primera prueba) | 0.223618 | 0.091139 |
| Mezcla adaptativa (entregada) | 0.220191 | 0.083919 |

La variante entregada cambia el RMSE optimizado en +3.81% respecto de odometría en esta secuencia y semilla. Es una observación de una sola comparación, no una garantía de menor RMSE.

La primera mezcla fija empeoró la comparación. La adaptación posterior usa únicamente la compatibilidad del desplazamiento del frontend con el prior de odometría; no usa ground truth para las poses, pesos ni el cálculo de la fracción. Se conserva la primera prueba en `fixed_on/`. Antes de afirmar mejoras generales hacen falta más secuencias y semillas independientes.

`replay_summary.json` y `reference_comparison.json` contienen las métricas completas. Cada carpeta de replay contiene tracking, evidencia bayesiana, loops, rendimiento, trayectoria y log de finalización.

Comando de la variante entregada (sobre el ejecutable compilado y el input exportado del CLF):

```bash
intel_accuracy_replay intel_full.input full_on 30 4 42 belief frontend 20 15 1 bayes probability_ceres 5 .05 odometry .10 .05 on
```

La comparación de odometría cambia el directorio de salida y el argumento final a `off`. La primera variante fija antecede a la adaptación; se conserva su resultado y se puede reproducir la regla fija con `scan_proposal_adapt_to_prior:=false` en el launch ROS.
