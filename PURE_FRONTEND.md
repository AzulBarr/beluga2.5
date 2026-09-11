# Variante 100% frontend — experimental, NO recomendada

**Resultado:** con Intel completo, semilla 42 e ICP activado en ambos modos,
la propuesta pura empeoró el RMSE online de 0.242879 m a 16.181990 m y el RMSE
optimizado de 0.079730 m a 15.912918 m, frente a la referencia incluida en el ZIP.
No se recomienda como mejora. Se conserva como experimento reproducible. La
normalización correcta de los pesos no garantiza una buena aproximación del
posterior con un número finito de partículas.

Esta variante no elige entre odometría y frontend. Cuando hay tracking válido y
un prior de movimiento no singular, **cada propuesta de cada partícula** usa

```
x_t = x_ancestro * inverse(local_pose_anterior_h) * local_pose_frontend_h * epsilon
```

El frontend incluye los refinamientos configurados y el resultado de ICP cuando
se acepta. Se conserva el ruido original del modelo diferencial y se aplica
`log p_odom - log q_frontend` al likelihood antes de la selección multi-try y de
la actualización de pesos/evidencia. No se reduce adaptativamente la fracción.
La densidad pura no tiene la cota de importancia de la mezcla defensiva anterior.

**Excepciones explícitas:** arranque sin frontend, tracking rechazado/débil,
validación sin evidencia común y movimiento singular (por ejemplo, reposo sin
ruido). En esos casos se conserva la propagación de odometría existente. Una
masa de Dirac no permite mover partículas a otro lugar mediante una propuesta
continua y conservar pesos de importancia válidos para el mismo prior.

Se corrigió también el forwarding de `icp_refine` en el launch general: antes
el XML de Intel pasaba ese argumento pero el launch Python no lo conectaba al
parámetro del nodo.

## Ejecutar exactamente esta variante

Después de copiar el contenido del ZIP a `~/ros2_ws/src/beluga2.5`, compilar:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select belugaslam_core belugaslam_node belugaslam_example belugaslam_benchmark --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch belugaslam_example intel_dataset_belugaslam_pure_frontend.xml record_bag:=false
```

Para MIT, usar `mit_rosbag_belugaslam_pure_frontend.xml`. Los dos presets activan
`icp_refine:=true`, `scan_informed_proposal:=true`, `scan_proposal_fraction:=1.0`
y `scan_proposal_adapt_to_prior:=false`. Los launches anteriores conservan sus
ajustes anteriores; estos presets son los que seleccionan la variante pura.

La comparación con odometría, conservando ICP activo, es el mismo preset con
`scan_informed_proposal:=false`. En el ejecutable de replay, el modo final puede
ser `pure`, y el argumento opcional siguiente activa ICP con `on`.

## Verificación

117 pruebas de integración y 91 pruebas de Python aprobadas; pruebas numéricas
adicionales de selección 100% frontend, composición del ruido y cambio de medida.
La comparación Intel utiliza 13.631 scans, semilla 42, N máximo 30, H máximo 4,
Ceres, prior odometry, lectura frontend y loop closure/PGO activos. **ICP está
activado en ambos modos.** El ground truth no entra en la ejecución del SLAM.

Las métricas y los logs están en `validation/pure_frontend/`. El nodo ROS 2 no
se ejecutó en este entorno; se compiló y ejecutó el núcleo real de producción.
Los resultados de la variante adaptativa anterior siguen en
`validation/scan_informed/` y no corresponden a esta prueba pura con ICP.
