# belugaslam_core

## [derived_cache.hpp](../belugaslam_core/include/belugaslam_core/derived_cache.hpp)

Define una sola clase plantilla, `DerivedCache<Payload>`, para guardar datos derivados que son caros de calcular, no siempre hacen falta y hay que poder descartar cuando se supera el límite de memoria. La usa cada submapa congelado para su `LoopMatchingData`.

---
### acquire(builder)
- **Entrada:** una lambda que sabe construir el dato.
- **Salida:** `shared_ptr<const Payload>` con el dato.
- Construye el dato solo si todavía no existe; si ya está, devuelve el que tiene.
- Registra el momento de uso, que es lo que después ordena el descarte.

---
### statistics()
- **Entrada:** ninguna.
- **Salida:** los bytes ocupados y el número de último uso.
- Es lo que consume la política de descarte para decidir qué liberar.

---
### release()
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Suelta la referencia del caché al dato.

---
## [fastslam_oc_grid_core.hpp](../belugaslam_core/include/belugaslam_core/fastslam_oc_grid_core.hpp)

Define la clase `BelugaSLAM`, que contiene el algoritmo completo: el frontend, que en cada scan predice con odometría, hace scan matching, pesa las partículas y las inserta en los submapas; y el backend, que al cerrarse un submapa busca cierres de lazo, los verifica y optimiza el grafo de poses. Cada partícula guarda un puntero a su `Hypothesis`, así que muchas partículas comparten el mismo mapa y las hipótesis se dividen solo cuando aparecen dos interpretaciones distintas del entorno.

### BelugaSLAM()
- **Entrada:** el modelo de movimiento, el modelo de medición y el struct `FastSLAMParams` con la configuración.
- **Salida:** la instancia lista para recibir el primer scan.
- Valida la configuración: lanza excepción cuando el valor no tiene un arreglo obvio y lo recorta al mínimo funcional cuando sí lo tiene.
- Abre los CSV de diagnóstico si hay ruta configurada y escribe sus encabezados.
- Crea una única hipótesis inicial y le asigna todas las partículas, en el origen y con peso uniforme.
- Deja la grilla de publicación con la extensión fija de `grid_config.hpp`, hasta que exista el primer submapa.

---
### particles()
- **Entrada:** ninguna.
- **Salida:** el vector de partículas, con la pose, el peso y el puntero a la hipótesis de cada una.
- Tiene dos versiones, `const` y no `const`.

<!--
La versión no const solo la usan los tests.
-->

---
### get_active_hypotheses_count()
- **Entrada:** ninguna.
- **Salida:** cuántas hipótesis hay activas.

<!--
Se registra en el CSV, y como se lee después de resample() arrastra el mismo desfase que el conteo de partículas.
-->

---
### get_submaps_count()
- **Entrada:** ninguna.
- **Salida:** cuántos submapas hay en el historial.
- No se usa en ningún lado, ni en el nodo ni en los tests.

<!--
Además de estar sin usar, si se usara tendría dos problemas: lee hypotheses_.front(), que es la primera de la lista y no la mejor, mientras que todos los demás accesores de ese bloque usan best_hypothesis_; y cuenta solo history, así que ignora los hasta dos submapas activos.
-->
---
### sample_motion_model(u)
- **Entrada:** el control de movimiento, o sea la pose de odometría actual y la anterior.
- **Salida:** ninguna.
- Guarda el desplazamiento de odometría del scan.
- Genera varias propuestas de pose por partícula muestreando el modelo de movimiento.
- Si el robot no se movió, genera una sola propuesta y no le agrega ruido.
- Deja la primera propuesta como pose provisional; la elección definitiva la hace [measurement_model_map(z)](#measurement_model_mapz).

<!--
La cantidad de propuestas por partícula es motion_proposal_samples. Con valor 1 el filtro se comporta como un bootstrap filter clásico: la propuesta es el modelo de movimiento y el peso es la verosimilitud del sensor.

Las propuestas quedan guardadas en motion_proposals_ y las consume el paso siguiente. Los dos pasos están acoplados: llamar a sample_motion_model() sin llamar después a measurement_model_map() deja las propuestas colgadas y las partículas en la primera muestra, que no es la elegida por el sensor.

El umbral de "quieto" es 1e-24 en norma al cuadrado (1e-12 m) y 1e-12 rad. Existe para que el robot detenido no acumule ruido de odometría scan tras scan.
-->

---
### measurement_model_map(z)
- **Entrada:** el scan en coordenadas del robot.
- **Salida:** ninguna.
- Submuestrea el scan con [select_tracking_points(scan, limit)](#select_tracking_pointsscan-limit).
- Corre el scan matching de cada hipótesis contra su propio submapa de referencia.
- Puntúa cada propuesta de movimiento con [tracking_score(field, scan, pose, options)](#tracking_scorefield-scan-pose-options).
- Elige una propuesta por partícula con [select_motion_proposal(logs, generator)](#select_motion_proposallogs-generator) y actualiza el peso con la evidencia de esa elección.
- Normaliza los pesos y escribe el CSV de tracking si se pidió.

<!--
La verosimilitud se multiplica por effective_beams (20 por defecto) antes de usarla como peso. Los puntos de un scan no son mediciones independientes: si se usara el log-likelihood de los cientos de puntos tal cual, una sola partícula se llevaría todo el peso y el filtro colapsaría. El factor trata al scan como si fueran 20 mediciones independientes.

El scoring de las propuestas es paralelo, pero la selección y la reducción de pesos son seriales a propósito: las extracciones del generador aleatorio tienen que ocurrir en orden fijo para que la corrida sea reproducible con la misma semilla.

El peso de la partícula es su peso previo por la verosimilitud MEDIA de sus propuestas, no la de la propuesta elegida. Usar la elegida sesgaría el filtro, porque la selección ya favorece a la mejor.
-->

---
### update_occupancy_grid(z, stamp)
- **Entrada:** el scan en coordenadas del robot, el timestamp en segundos y, opcionalmente, el timestamp en nanosegundos.
- **Salida:** los submapas que quedaron terminados en este scan.
- Le asigna un número de secuencia al scan.
- Decide por hipótesis si el scan es keyframe, con el umbral de [motion_filter_accepts(elapsed_seconds, translation, rotation, max_time_seconds, max_translation, max_rotation)](#motion_filter_acceptselapsed_seconds-translation-rotation-max_time_seconds-max_translation-max_rotation).
- Si no es keyframe, solo registra la muestra de trayectoria y sigue.
- Crea un submapa nuevo cuando el más reciente alcanzó submap_num_range_data inserciones.
- Inserta el scan en los dos submapas activos y agrega al grafo el nodo y sus restricciones.

<!--
Todo el paso usa hypothesis->local_pose, no la partícula de mayor peso. Tomar la mejor partícula en cada scan haría que la trayectoria salte entre partículas distintas, y ese salto se insertaría en la grilla como si fuera movimiento real: paredes dobles o gruesas que ninguna optimización posterior puede arreglar, porque el PGO mueve submapas pero no repara su interior.

Si la hipótesis todavía no tiene local_pose se saltea entera y ese scan no deja ni nodo ni muestra de trayectoria. Por eso las secuencias del archivo final pueden no arrancar en cero.

Un scan rechazado por el tracking tampoco modifica grillas ni contadores, pero sí registra su muestra de trayectoria. Así la trayectoria exportada queda completa aunque el mapa no se haya tocado.

El parámetro stamp_ns tiene default: si no se pasa, se deriva del timestamp en segundos redondeando a nanosegundos.
-->

---
### resample()
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Detecta divergencia espacial entre partículas y divide la población en hipótesis nuevas si corresponde.
- Calcula el ESS y no remuestrea si supera la mitad de las partículas y ninguna hipótesis está agotada por dentro.
- Ordena las hipótesis por masa y recorta a max_hypotheses.
- Reinstala la población repartiendo cuotas de partículas y vuelve a elegir la salida.

<!--
El chequeo de agotamiento es doble: el ESS global y, además, uno por hipótesis. Una hipótesis chica puede quedar con todas sus partículas en una sola pose aunque el ESS global esté alto, y sin ese segundo chequeo se quedaría congelada.

Las cuotas garantizan un mínimo de partículas por hipótesis. Eso es una asignación de cómputo, no masa de creencia: el llamador tiene que dividir el peso de la hipótesis entre su cuota, si no las hipótesis con pocas partículas ganarían peso solo por existir.

Se cachea la covarianza de cada hipótesis ANTES de remuestrear. Después del remuestreo las partículas están duplicadas y la covarianza mediría el remuestreo en vez de la incertidumbre real.
-->

---
### post_update(finished_events)
- **Entrada:** los submapas que terminaron en este scan.
- **Salida:** ninguna.
- Busca candidatos a loop closure entre los submapas terminados.
- Corre el PGO de base de cada hipótesis, salteándolo si el grafo no cambió desde la última optimización.
- Verifica cada candidato y acepta o descarta el loop.
- Vuelve a elegir la hipótesis de salida.

<!--
La firma real recibe también el scan, pero no lo usa: está descartado con (void)z. Queda por simetría con los otros pasos del ciclo.

El PGO de base se saltea cuando el grafo no tiene restricciones inter-submapa. Sin loops, el grafo local se construye a partir de las mismas poses locales inmutables y ya tiene solución de residuo cero: armar el problema de Ceres no agregaría información.

El orden importa. La búsqueda de candidatos va primero y el PGO de base después, porque la verificación necesita comparar contra un grafo ya optimizado. Pero armar Ceres antes de saber si hay algún candidato sería trabajo perdido, así que la búsqueda se hace primero y el solve solo si hace falta.
-->

que se encarga de disparar loop closure con los submapas recién cerrados, correr PGO si hay nuevas restricciones, elegir la hipótesis que se publica y armar el mapa global.

- La elección de la hipótesis está en una función aparte, `refresh_output_selection()`, y depende del parámetro `output_selection_mode`: con `map` (el valor por defecto) gana la de mayor peso total, con desempate por id; con `pose_risk` gana la que minimiza el riesgo cuadrático de posición, es decir la que está más cerca del resto de las hipótesis ponderadas por su masa.

<!-- 
z no se usa: la primera línea del cuerpo es literalmente (void)z. El parámetro está en la firma pero descartado, así que la medición no interviene acá.
Los pasos de loop closure están detrás de #if BELUGASLAM_ENABLE_LOOP_CLOSURE. Con loop closure deshabilitado, post_update() se reduce a elegir hipótesis, fijar pose y componer el mapa.
La lógica de select_output_pose vive en output_selection.hpp. Publica siempre una pose que ya existe en alguna hipótesis, en vez de una media entre hipótesis: así la pose que sale queda emparejada con un mapa completo y coherente. El "riesgo" que reporta es una pérdida cuadrática interna en m2, no un RMSE medido contra ground truth.
-->

---
### best_occupancy_grid()
- **Entrada:** ninguna.
- **Salida:** la grilla de ocupación de la hipótesis seleccionada, en valores de nav_msgs.
- Reconstruye la vista de publicación si algo cambió desde la última llamada.

<!--
La reconstrucción es perezosa: se marca con publication_dirty_ y solo ocurre en la primera lectura después de un cambio. Componer la vista dibuja todos los submapas de la hipótesis en una grilla única, así que cuesta proporcional al mapa entero. Llamarla en cada scan sería caro; el nodo la llama desde un timer aparte.
-->

---
### best_log_odds_grid()
- **Entrada:** ninguna.
- **Salida:** la grilla de log-odds de la hipótesis seleccionada.
- Igual que [best_occupancy_grid()](#best_occupancy_grid), pero sin convertir a la escala de nav_msgs.

<!--
Las dos comparten la misma reconstrucción, así que pedir las dos seguidas no cuesta el doble. La usa el cálculo de entropía, que necesita la probabilidad continua y no el valor discretizado a 0-100.
-->

---
### loop_closure_poses()
- **Entrada:** ninguna.
- **Salida:** la pose del robot en cada cierre de lazo aceptado, en orden de ocurrencia.
- Es un registro histórico que solo crece, no un estado actual.
- Existe para dibujar marcadores en RViz.

<!--
Son las poses de cuando ocurrió cada evento, según la hipótesis en ese momento; no se corrigen si una optimización posterior mueve la trayectoria. Es el mismo caso que el topic /trajectory.

publish_visualization() no redibuja los marcadores en cada tick: compara el tamaño del vector contra el que recordaba y solo republica si creció. Como el vector nunca se achica ni se reordena, el tamaño alcanza como detector de cambios, la misma idea que usa inter_constraint_count() para saltear el PGO.
-->

---
### spatial_split_poses()
- **Entrada:** ninguna.
- **Salida:** la pose del robot en cada división de hipótesis por divergencia espacial.
- Igual que [loop_closure_poses()](#loop_closure_poses): solo crece y existe para RViz.



---
### `write_optimized_trajectory(out)`
- **Entrada:** el stream donde escribir.
- **Salida:** cuántas poses escribió.
- Escribe la trayectoria optimizada de la hipótesis seleccionada, en formato TUM.
- Antepone dos líneas de comentario con la hipótesis elegida y la cantidad de nodos.
- Toma el timestamp de cada muestra y la pose corregida del grafo.
- Lanza excepción si falta la pose de alguna secuencia o si los timestamps no son crecientes.

<!--
Es el mismo dato que la columna optimizada de [final_trajectory()](#final_trajectory): las dos recorren trajectory_samples y sacan la pose del mismo llamado a pose_at_sequence(). Las diferencias son el formato, que acá es TUM listo para evo, y que esta no exporta la pose online.

El timestamp sale de la propia muestra, no del nodo de ROS, así que no depende de que el nodo lleve su registro de secuencia a timestamp.

Escribe los nanosegundos con setw(9) y relleno de ceros para no perder precisión: formatear el timestamp como double redondearía a unos 200 ns.
-->

---
### final_trajectory()
- **Entrada:** ninguna.
- **Salida:** un punto por cada scan procesado de la hipótesis seleccionada, con la pose que se publicó en ese momento y la pose después de la optimización.
- Los keyframes toman la pose de su nodo del grafo; los scans intermedios se apoyan en el submapa contra el que se registraron.
- Pensada para llamarse una sola vez, al final del recorrido.

<!--
La diferencia entre las dos poses es todo lo que el PGO corrigió después de haber publicado la pose. Medir RMSE contra output_x/output_y de performance.csv usa solo la columna online, así que ningún cierre de lazo llega al resultado.

Se guarda una muestra (TrajectorySample) por scan procesado y no se podan nunca: de los nodos viejos solo se libera la nube de puntos, no la pose. Por eso al final del dataset está la trayectoria completa y alcanza con leerla una vez.

La muestra guarda T_submap_robot, así que cuando el PGO mueve el submapa la pose se mueve con él. La pose online se guarda aparte, en frontend_pose, porque T_global_local cambia en cada optimización y después no se puede reconstruir.

Devuelve la trayectoria de una sola hipótesis, la que quedó seleccionada al final. La trayectoria publicada durante el recorrido puede saltar entre hipótesis, así que las dos poses de una misma fila no siempre vienen de la misma cadena de decisiones.
-->

---
### scan_sequence_count()
- **Entrada:** ninguna.
- **Salida:** cuántos scans numeró el core; el próximo va a recibir ese número como secuencia.
- Sirve para saber qué secuencia le tocó al scan recién procesado.

<!--
No coincide con la cuenta de scans recibidos del nodo: el core numera solo los que llegan a update_occupancy_grid() con puntos y timestamp válidos, no los que se descartan antes.
-->

---
## [grid_config.hpp](../belugaslam_core/include/belugaslam_core/grid_config.hpp)

Constantes de la grilla generadas automáticamente. No se edita a mano: lo regenera el proceso de build. No define funciones, solo los valores de ocupación de nav_msgs, el radio del robot y el tamaño, la resolución y el origen de la grilla por defecto.

<!--
Los valores de kGridRows, kGridCols, kOriginX y kOriginY corresponden a un mapa fijo de 350x350 celdas. El SLAM no los usa para sus submapas, que se agrandan solos a medida que entran scans; quedan para las herramientas que necesitan una grilla de tamaño conocido de antemano.

kRobotRadius vale 0.01 m, mucho menos que un robot real. Es el radio que se fuerza a libre alrededor del sensor al insertar un scan, así que un valor chico es conservador: limpia menos celdas.
-->

## [grid_update.hpp](../belugaslam_core/include/belugaslam_core/grid_update.hpp)

Escribe un scan en una grilla de log-odds: marca los impactos y traza los rayos de espacio libre entre el sensor y cada impacto. Está separado del submapa para poder testear la actualización de celdas sin construir un mapa.

---
### `apply_scan_cells(cells, width, height, origin_x, origin_y, hit, miss, clamp, scratch, hits, misses)`
- **Entrada:** la grilla, sus dimensiones, la celda del sensor, los incrementos de impacto y de espacio libre, el límite de saturación, el buffer reutilizable y los dos vectores de salida.
- **Salida:** ninguna; deja en hits y misses los índices tocados.
- Primero suma el incremento de impacto en cada celda extremo de rayo.
- Después traza los rayos desde el sensor y suma el incremento de espacio libre.
- Toca cada celda una sola vez por scan.
- Satura los valores en más y menos clamp.

<!--
Que los impactos vayan antes que los rayos no es un detalle de orden: es lo que le da prioridad al impacto. Una celda marcada como ocupada por un rayo puede estar atravesada por otro, y sin esta prioridad el segundo rayo la borraría. Como la marca de "ya visitada" es única por scan, el mismo mecanismo resuelve la deduplicación y la prioridad.

La celda del sensor se saltea explícitamente: el robot no observa su propia posición como espacio libre a través de un rayo.

El buffer usa épocas en vez de limpiarse: guarda un contador por celda y lo compara contra el de este scan. Así no hay que recorrer toda la grilla en cada inserción. Cuando el contador llega al máximo de 32 bits, ahí sí limpia todo y vuelve a empezar.
-->

## [loop_belief.hpp](../belugaslam_core/include/belugaslam_core/loop_belief.hpp)

Decide si un loop closure se acepta. La pregunta no es si el loop encaja geométricamente, sino cuánto deforma la trayectoria que ya se tenía y si el conjunto de hipótesis está de acuerdo.

---
### `wrap_angle(angle)`
- **Entrada:** un ángulo en radianes.
- **Salida:** el mismo ángulo en el rango de menos pi a pi.

---
### `aligned_trajectory_change(prior, trial)`
- **Entrada:** la trayectoria antes del loop y la misma trayectoria después de aplicarlo.
- **Salida:** el RMSE de traslación y de rotación entre las dos, o inválido si no se pudo alinear.
- Alinea la trayectoria de prueba contra la anterior con una transformación rígida.
- Mide cuánto se movió cada pose después de esa alineación.
- Devuelve inválido si las trayectorias tienen distinto largo, menos de tres poses, algún valor no finito o extensión casi nula.

<!--
La alineación es SE(2) y no una similitud: no ajusta escala. Un LiDAR mide metros, así que la escala es conocida. Permitir que la alineación la ajuste dejaría pasar un loop que colapsa o estira el mapa, porque el estiramiento se absorbería en el factor de escala en vez de aparecer como error.

Alinear antes de medir es lo que separa "el loop movió el mapa entero" de "el loop deformó el mapa". Lo primero es inofensivo, porque el marco global es arbitrario; lo segundo es el daño que hay que detectar.

El corte por extensión menor a 1e-8 evita dividir por cero cuando el robot casi no se movió: sin desplazamiento no hay forma de determinar la rotación de la alineación.
-->

---
### `trajectory_compatibility(change, translation_scale, rotation_scale)`
- **Entrada:** la deformación medida y las escalas de tolerancia de traslación y rotación.
- **Salida:** un número entre 0 y 1.
- Convierte la deformación en compatibilidad con una gaussiana sobre los dos errores normalizados.
- Devuelve 0 si la deformación es inválida o si alguna escala no es positiva.

<!--
Las escalas son loop_translation_scale y loop_rotation_scale. No son umbrales duros: fijan a qué deformación la compatibilidad cae a exp(-0.5), o sea alrededor de 0.61. La decisión de aceptar la toma el umbral de creencia sobre el valor marginalizado.
-->

---
### `normalize_masses(masses)`
- **Entrada:** las masas de las hipótesis, sin normalizar.
- **Salida:** las mismas masas sumando 1, o todos ceros si la suma es cero.
- Lanza excepción si alguna masa es negativa o no finita, o si la suma desborda.

---
### `marginalize_compatibilities(masses, compatibility)`
- **Entrada:** las masas de las hipótesis y la compatibilidad de cada una con el loop.
- **Salida:** la evidencia pesada por masa, la de la hipótesis MAP y la uniforme.
- Recorta cada compatibilidad al rango 0 a 1 y trata como 0 las que no son finitas.
- Lanza excepción si los dos arreglos tienen distinto largo.

<!--
Una hipótesis que no soporta el loop, o para la que no se pudo evaluar, entra con compatibilidad cero y sigue contando en el denominador. Renormalizar solo sobre las que sí lo soportan convertiría una masa posterior mínima en certeza: si una sola hipótesis de peso 0.01 acepta el loop, renormalizar daría evidencia 1.

Devuelve las tres variantes para que el modo del verificador elija cuál mirar. La pesada es la que corresponde al posterior; la MAP y la uniforme están para comparar.
-->

---
### `allocate_particle_quotas(masses, budget)`
- **Entrada:** las masas de las hipótesis y el total de partículas disponibles.
- **Salida:** cuántas partículas le tocan a cada hipótesis.
- Le da al menos una partícula a cada hipótesis y reparte el resto proporcional a la masa.
- Asigna las partículas sobrantes por resto decreciente.
- Lanza excepción si el presupuesto es menor que la cantidad de hipótesis.

<!--
La cuota mínima es una asignación de cómputo, no masa de creencia. El llamador tiene que darle a cada partícula del modo h un peso W_h dividido su cuota; si no, una hipótesis casi descartada ganaría peso solo por tener su partícula garantizada.

El reparto por resto decreciente (método de Hamilton) hace que la suma de las cuotas dé exactamente el presupuesto, cosa que redondear cada una por separado no garantiza.
-->

## [loop_search.hpp](../belugaslam_core/include/belugaslam_core/loop_search.hpp)

Busca dónde encaja un scan dentro de un submapa ya construido. Es el paso de recuperación del loop closure: propone poses candidatas, que después verifica el resto del sistema.

---
### `LoopFieldView::sample(x, y)`
- **Entrada:** una coordenada en el marco del submapa.
- **Salida:** la distancia al obstáculo más cercano y el puntaje de esa posición.
- Interpola bilinealmente entre los centros de las cuatro celdas vecinas.
- Devuelve distancia infinita y puntaje cero fuera de la grilla.

<!--
Interpolar en centros de celda y no en esquinas elimina las mesetas de media celda que aparecen al puntuar por índice redondeado. Sin eso el refinamiento fino se queda trabado: varias poses distintas dan exactamente el mismo puntaje.

La vista no copia nada, solo referencia los arreglos ya cacheados en el submapa. Por eso el submapa tiene que seguir vivo mientras se use la vista.
-->

---
### `score_loop_scan(field, scan, pose)`
- **Entrada:** el campo del submapa, el scan y la pose candidata.
- **Salida:** el puntaje medio y la fracción de puntos que caen cerca de un obstáculo.
- Transforma cada punto del scan a la pose candidata y lo consulta contra el campo.
- Cuenta como superposición los puntos a 30 cm o menos de un obstáculo.

---
### `separated_loop_modes(a, b, distance, angle)`
- **Entrada:** dos poses y los umbrales de separación.
- **Salida:** true si están suficientemente separadas como para ser modos distintos.

---
### `search_loop_modes(initial, options, score)`
- **Entrada:** la pose inicial, las opciones de búsqueda y la función que puntúa una pose.
- **Salida:** hasta max_modes poses candidatas, ordenadas por puntaje.
- Barre una grilla gruesa alrededor de la pose inicial y descarta lo que no llega al umbral de superposición.
- Se queda con hasta beam_width semillas separadas entre sí.
- Refina cada semilla siete niveles, partiendo el paso a la mitad en cada uno.
- Devuelve solo las que superan los umbrales de puntaje y superposición, y que están separadas entre sí.
- Lanza excepción si alguna opción está fuera de rango.

<!--
El trabajo es fijo por ventana de búsqueda: como mucho 41x41x47 poses gruesas, ocho caminos de refinamiento y siete niveles. No depende del tamaño del mapa ni de la trayectoria, así que el costo del loop closure no crece a lo largo de la corrida.

Es una heurística acotada, no una búsqueda con garantía de óptimo. No hay branch and bound: nada asegura que la mejor pose de la ventana esté entre las devueltas.

Cada semilla gruesa conserva su propio camino de refinamiento. Sin eso, dos semillas que convergen a la misma pose fina consumirían dos de las ocho ranuras y se perderían modos alternativos, que es justo lo que hay que detectar para saber si el loop es ambiguo.

El desempate por distancia al centro de la ventana existe para que una dirección geométricamente plana no elija una esquina de la ventana solo porque se enumeró primero. Es desempate, no evidencia.
-->

---
### `LoopQueryLedger::consume(query)`
- **Entrada:** la secuencia del scan consultado.
- **Salida:** true si es la primera vez que se consume.
- Registra que la evidencia de ese scan ya se usó para decidir loop o no loop.

<!--
La decisión de un scan se consume una sola vez para toda la creencia, incluidos los descendientes que decidieron no cerrar el loop. La evidencia pertenece a la creencia viva y no a una rama individual: si cada rama pudiera volver a consumirla, el mismo scan sumaría evidencia varias veces.

Guarda un entero por evento consumido, igual que el grafo. No retiene el scan.
-->

## [motion_filter.hpp](../belugaslam_core/include/belugaslam_core/motion_filter.hpp)

Decide si un scan merece entrar al mapa. Es el umbral de keyframe: sin él cada scan crearía un nodo del grafo y el problema crecería sin necesidad mientras el robot está quieto.

---
### `motion_filter_accepts(elapsed_seconds, translation, rotation, max_time_seconds, max_translation, max_rotation)`
- **Entrada:** el tiempo, la traslación y la rotación desde el último scan insertado, y los tres umbrales.
- **Salida:** true si el scan se inserta.
- Acepta si se superó cualquiera de los tres umbrales.
- Acepta también si el tiempo transcurrido es negativo.

<!--
El desplazamiento se mide contra la última pose INSERTADA y después del scan matching, no contra el scan anterior ni contra la predicción de odometría. Si se midiera contra el scan anterior, un robot que avanza despacio nunca insertaría nada.

Aceptar cuando el tiempo es negativo es deliberado: un timestamp que retrocede abre un intervalo de inserción nuevo en vez de dejar el filtro trabado esperando un tiempo que ya pasó.

La comparación es estricta (>) en los tres umbrales, así que un desplazamiento exactamente igual al umbral se considera "similar" y se rechaza. Es el mismo criterio que MotionFilter::IsSimilar de Cartographer.
-->

## [output_selection.hpp](../belugaslam_core/include/belugaslam_core/output_selection.hpp)

Elige cuál de las hipótesis se publica. La decisión es sobre poses ya calculadas: no corre un paso del filtro ni promedia hipótesis.

---
### `select_output_pose(hypotheses)`
- **Entrada:** las hipótesis con masa positiva, cada una con su identificador, su masa y su posición.
- **Salida:** el índice de la hipótesis MAP, el de mínimo riesgo, y el riesgo de cada una.
- La MAP es la de mayor masa; los empates se resuelven por identificador más chico.
- El riesgo de una hipótesis es la suma de las distancias al cuadrado a todas las demás, pesada por sus masas.
- Arranca la búsqueda de mínimo riesgo desde la MAP, así un empate exacto conserva la pose ya publicada.

<!--
Publicar una hipótesis existente en vez del promedio de todas es a propósito: el promedio de dos poses separadas cae en un lugar donde no hay mapa, y la pose publicada quedaría desalineada del mapa que se publica junto a ella. Publicar una hipótesis entera mantiene pose y mapa consistentes.

El "riesgo" es una pérdida cuadrática interna en metros cuadrados sobre las posiciones de las hipótesis. NO es el RMSE contra ground truth ni una estimación de él: no hay ninguna referencia externa en el cálculo.

Los acumuladores son long double y hay un chequeo de desborde, porque la suma pesada de cuadrados crece rápido si dos hipótesis se separan mucho.

El llamador tiene que pasar las hipótesis ordenadas por identificador estable, si no los empates de riesgo se resuelven según el orden de llegada y la corrida deja de ser reproducible.
-->

## [particle_proposal.hpp](../belugaslam_core/include/belugaslam_core/particle_proposal.hpp)

Las dos operaciones aleatorias del filtro de partículas: elegir entre las propuestas de movimiento de una partícula, y elegir qué partículas sobreviven al remuestreo.

---
### `select_motion_proposal(logs, generator)`
- **Entrada:** los log-likelihood de las propuestas de una partícula y el generador aleatorio.
- **Salida:** el índice elegido y el log de la evidencia incremental.
- Elige una propuesta al azar, con probabilidad proporcional a su verosimilitud.
- Devuelve como evidencia el log de la verosimilitud MEDIA, no la de la elegida.
- Lanza excepción si el conjunto está vacío o si alguna verosimilitud no es finita.

<!--
Que la evidencia sea la media y no el máximo es lo que hace que el filtro siga siendo correcto. La propuesta se eligió mirando el sensor, así que ya está sesgada hacia lo que el sensor prefiere; usar su verosimilitud como peso contaría esa información dos veces. La media es el estimador insesgado de la verosimilitud marginal bajo la propuesta.

Con una sola propuesta la media es esa misma propuesta y el resultado es exactamente el filtro bootstrap.

La resta del máximo antes de exponenciar evita el desborde: los log-likelihood de un scan completo son números muy negativos y exp() de eso daría cero.
-->

---
### `systematic_indices(weights, count, generator)`
- **Entrada:** los pesos de las partículas, cuántas hay que sacar y el generador aleatorio.
- **Salida:** los índices elegidos, con repetición.
- Usa un solo desplazamiento aleatorio y recorre la distribución acumulada a paso fijo.
- Acepta pesos sin normalizar y saltea los de peso cero.
- Lanza excepción si algún peso es negativo o no finito, o si la masa total es cero.

<!--
El remuestreo sistemático tiene menos varianza que sacar cada partícula por separado de una categórica: con una sola extracción aleatoria, una partícula de peso w recibe siempre floor(N*w) o ceil(N*w) copias, nunca menos ni más. Con extracciones independientes podría recibir cero por azar.

Como el desplazamiento es uno solo para toda la población, las copias quedan correlacionadas entre sí. Eso es aceptable acá porque el paso siguiente vuelve a aplicar el modelo de movimiento, que las separa.
-->

## [particle.hpp](../belugaslam_core/include/belugaslam_core/particle.hpp)

La grilla de log-odds que usan los submapas, con las dos operaciones que cambian su tamaño: agrandarla cuando un scan se sale, y recortarla cuando el submapa se termina.

---
### `crop_to_known_cells(margin_cells)`
- **Entrada:** cuántas celdas de espacio desconocido dejar alrededor de lo observado.
- **Salida:** true si la grilla cambió de tamaño.
- Busca la caja de celdas observadas, o sea las que tienen log-odds distinto de cero.
- Recorta la grilla a esa caja más el margen.
- Solo mueve el origen: las celdas que quedan conservan sus coordenadas.

<!--
Es la contraparte de [grow_to_include(min_x, min_y, max_x, max_y)](#grow_to_includemin_x-min_y-max_x-max_y). Mientras el submapa está activo la grilla se agranda hasta donde lleguen los scans, que a lo largo de un submapa entero es mucho más de lo que el sensor llegó a observar. Se llama al terminar el submapa, igual que ComputeCroppedGrid() en Cartographer.

Que solo mueva el origen es lo que mantiene válidas las poses y las restricciones medidas contra ese submapa. Si el recorte reindexara las celdas, todas las restricciones del grafo apuntarían al lugar equivocado.

El margen existe para que una consulta apenas afuera de una pared caiga en una celda real y no se salga de la grilla.
-->

---
### `grow_to_include(min_x, min_y, max_x, max_y)`
- **Entrada:** la caja en coordenadas del marco de la grilla que tiene que quedar adentro.
- **Salida:** true si la grilla cambió de tamaño.
- No hace nada si la caja ya entra.
- Agranda en bloques de 32 celdas, no en la cantidad exacta que falta.
- Solo mueve el origen: las celdas existentes conservan sus coordenadas.
- Corta en 4000 celdas por lado.

<!--
Agrandar de a bloques evita reasignar la grilla en cada scan cuando el robot avanza despacio y el borde se corre de a poco.

El límite de 4000 celdas por lado es una defensa contra una pose divergente: sin él, una partícula que se fue lejos pediría una grilla enorme y el proceso se quedaría sin memoria.
-->

## [pose_graph_cost.hpp](../belugaslam_core/include/belugaslam_core/pose_graph_cost.hpp)

Envuelve el residuo del grafo para Ceres. Ofrece las dos variantes, la analítica y la de diferenciación automática, para poder compararlas entre sí.

---
### `AnalyticPoseGraphCost::Evaluate(p, r, j)`
- **Entrada:** los dos bloques de parámetros, y los punteros de salida del residuo y las jacobianas.
- **Salida:** false si el residuo no es finito.
- Delega todo en [PoseGraphResidual::evaluate(a, b, r, ja, jb)](#posegraphresidualevaluatea-b-r-ja-jb).
- Acepta que Ceres pida el residuo sin jacobianas, o solo una de las dos.

---
### `PoseGraphEdgeError::operator()(pose_i, pose_j, residuals)`
- **Entrada:** las dos poses de la arista y el puntero de salida del residuo.
- **Salida:** siempre true.
- Calcula el mismo residuo que la versión analítica, pero con tipos genéricos para que Ceres lo derive solo.

<!--
Existe para verificar la versión analítica, no para producción: hay un test que compara residuo y jacobianas de las dos y exige que coincidan en 1e-10. Si alguien toca las derivadas escritas a mano, ese test lo detecta.
-->

---
### `PoseGraphEdgeError::Create(dx, dy, dtheta, weight_translation, weight_rotation, offset_x, offset_y, offset_angle, use_analytic)`
- **Entrada:** la medición de la arista, sus pesos, el offset rígido y qué variante construir.
- **Salida:** la función de costo lista para agregar al problema de Ceres.
- Devuelve la variante analítica o la de diferenciación automática según el último argumento.

<!--
El llamador cede la propiedad del puntero a Ceres, que lo libera al destruir el problema. Por eso hay new sin delete.

Cuál se usa lo decide el parámetro pgo_analytic_jacobians. La analítica es más rápida; la otra queda como referencia y para el test de equivalencia.
-->

## [pose_graph_residual.hpp](../belugaslam_core/include/belugaslam_core/pose_graph_residual.hpp)

El residuo de una arista del grafo de poses, con sus derivadas escritas a mano. Está separado de [pose_graph_cost.hpp](../belugaslam_core/include/belugaslam_core/pose_graph_cost.hpp) para poder testearlo sin depender de Ceres.

---
### `PoseGraphResidual::evaluate(a, b, r, ja, jb)`
- **Entrada:** las dos poses [x, y, yaw] de la arista y los punteros de salida del residuo y las dos jacobianas.
- **Salida:** false si algún residuo no es finito.
- Calcula la pose de b en el marco de a y la compara contra la medición de la arista.
- Aplica el offset rígido al extremo a antes de comparar.
- Pesa los dos residuos de traslación y el de rotación por separado.
- Escribe las jacobianas solo si le pasaron los punteros.

<!--
El residuo angular pasa por atan2(sin, cos) en vez de restarse directo. Sin eso, una diferencia de 359 grados contaría como un error enorme en vez de uno de un grado, y el solver la corregiría dando toda la vuelta.

El offset rígido existe para que varios submapas compartan una sola variable del grafo: cada uno entra con su desplazamiento fijo respecto del grupo. El offset se rota al construirlo, no en cada evaluación.

Las jacobianas son constantes respecto de b y solo dependen del ángulo de a. Por eso el bloque jb no tiene términos cruzados.
-->

---
### `weighted_loop_residual_squared(translation_error, rotation_error, translation_weight, rotation_weight)`
- **Entrada:** los errores de traslación y rotación de un loop, y sus pesos.
- **Salida:** la suma de los dos errores pesados al cuadrado.

<!--
Es el mismo criterio de escala que usa el residuo de las aristas, expuesto aparte para que la verificación de loops mida con la misma vara que el optimizador. Si verificación y optimización usaran escalas distintas, un loop podría pasar la verificación y después empeorar el costo del grafo.
-->

## [proposal_pose.hpp](../belugaslam_core/include/belugaslam_core/proposal_pose.hpp)

Lectura alternativa de la pose de una hipótesis: en vez de quedarse con la pose del scan matching, usa la media de toda la nube de propuestas pesadas. Es opcional y está detrás de varios chequeos, porque la media solo tiene sentido si la nube es unimodal y concentrada.

---
### `normalized_proposal_weights(cloud)`
- **Entrada:** la nube de propuestas con su log-peso.
- **Salida:** los pesos normalizados, o vacío si la nube no sirve.
- Devuelve vacío si algún log-peso es NaN o más infinito, o si alguna pose no es finita.
- Resta el máximo antes de exponenciar.

<!--
Un log-peso de menos infinito sí se acepta: es una propuesta imposible, que queda con peso cero. Más infinito no, porque no hay forma de normalizar contra él.
-->

---
### `proposal_second_moment(cloud, weights, center)`
- **Entrada:** la nube, sus pesos normalizados y el centro respecto del cual medir.
- **Salida:** la matriz 3x3 de segundo momento, en orden por filas.
- Envuelve la diferencia de ángulo antes de acumular.

---
### `summarize_proposal_poses(cloud, weights, reference, translation_window, rotation_window)`
- **Entrada:** la nube, sus pesos, una pose de referencia y las ventanas de traslación y rotación.
- **Salida:** la media, la covarianza, el ESS, la masa local y los desvíos de posición y de ángulo.
- Acumula todo relativo a la pose de referencia.
- Promedia el ángulo de forma circular, no aritmética.
- Mide como masa local la fracción de peso que cae dentro de las dos ventanas.

<!--
Acumular relativo a la referencia y no en coordenadas absolutas evita la cancelación catastrófica: lejos del origen del mundo, las poses son números grandes y casi iguales, y restarlos al final pierde precisión.

El promedio circular es necesario porque los ángulos dan la vuelta: el promedio aritmético de 179 y -179 grados da cero, cuando la respuesta es 180.

La masa local es el indicador de unimodalidad. Si la nube tiene dos grupos separados, la masa local baja aunque el ESS siga alto, y la media caería entre los dos grupos, en un lugar donde no hay ninguna propuesta.
-->

---
### `check_proposal_pose(summary, field, scan, prediction, frontend_score, tracking, minimum_ess, minimum_local_mass, maximum_log_drop)`
- **Entrada:** el resumen de la nube, el campo del submapa, el scan, la predicción de odometría, el puntaje del frontend, las opciones de tracking y los tres umbrales.
- **Salida:** si se acepta la media, el motivo, y el puntaje de la media contra el mapa.
- Rechaza si el ESS es bajo, si la nube es difusa o multimodal, o si la media se fue de la ventana de movimiento.
- Puntúa la media contra el mapa nativo y la rechaza si ajusta peor que el frontend por más de maximum_log_drop.
- Devuelve el motivo del rechazo como texto, para diagnóstico.

<!--
Los chequeos seleccionan qué lectura publicar; NO modifican los pesos de las propuestas ni podan las colas. La distribución posterior queda igual: lo único que cambia es qué número se reporta como pose de la hipótesis.

El último chequeo es independiente de los anteriores: vuelve a medir la media contra el mapa, en vez de confiar en los estadísticos de la nube. Una nube puede ser concentrada y estar concentrada en el lugar equivocado.

Lo controla el parámetro frontend_pose_mode, que por defecto vale "frontend", o sea que toda esta ruta está apagada salvo que se pida explícitamente proposal_mean.
-->

---
## [robust_tracking.hpp](../belugaslam_core/include/belugaslam_core/robust_tracking.hpp)

El scan matching del frontend. Alinea cada scan contra el submapa de su hipótesis partiendo de la predicción de odometría, y decide si el resultado es confiable. También tiene la búsqueda de recuperación para cuando el tracking se pierde.

---
### `tracking_score(field, scan, pose, options)`
- **Entrada:** el campo de distancias del submapa, el scan, la pose a evaluar y las opciones.
- **Salida:** el log-likelihood medio, la superposición y la cantidad de inliers.
- Transforma cada punto del scan a la pose y mide su distancia al obstáculo más cercano.
- Combina una gaussiana sobre esa distancia con una probabilidad fija de outlier.
- Cuenta como inlier todo punto a menos de inlier_distance.

<!--
El término de outlier es lo que hace robusta la métrica: sin él, un solo punto lejano manda el log-likelihood a menos infinito y arruina la pose entera. Con él, cada punto aporta como mucho el log de la probabilidad de outlier.

Devuelve el log-likelihood MEDIO, no la suma. Así el valor no depende de cuántos puntos tenga el scan y se pueden comparar scans de distinto largo.
-->

---
### `tracking_objective(field, scan, pose, prior, options)`
- **Entrada:** el campo, el scan, la pose a evaluar, la pose previa de odometría y las opciones.
- **Salida:** el costo a minimizar.
- Suma el log-likelihood negativo de [tracking_score(field, scan, pose, options)](#tracking_scorefield-scan-pose-options) y una penalización gaussiana por alejarse de la predicción.

<!--
La penalización usa prior_translation_sigma y prior_rotation_sigma. Es la que impide que el matcher tire la pose a cualquier lado cuando el scan tiene poca estructura, por ejemplo en un pasillo largo donde deslizarse a lo largo no cambia el puntaje.
-->

---
### `solve_tracking_system(matrix, rhs, solution)`
- **Entrada:** la matriz 3x3 del sistema, el lado derecho y el vector de salida.
- **Salida:** false si el sistema es singular o la solución no es finita.
- Resuelve por eliminación gaussiana con pivoteo parcial.

<!--
Toma la matriz y el lado derecho por copia porque los modifica al eliminar. Rechaza pivotes menores a 1e-14, que es lo que pasa cuando la geometría del scan no restringe alguna dirección.
-->

---
### `match_tracking_scan(field, scan, prior, options, seed)`
- **Entrada:** el campo, el scan, la predicción de odometría, las opciones y opcionalmente una pose semilla.
- **Salida:** la pose alineada, su puntaje, si se acepta, y el costo inicial y final.
- Usa la semilla como punto de partida solo si mejora el costo y cae dentro de la ventana de movimiento.
- Itera hasta max_iterations resolviendo el sistema linealizado.
- Rechaza el resultado si no llega al mínimo de puntos, de superposición, o si se salió de la ventana.
- Si rechaza, devuelve la predicción de odometría sin tocar.

<!--
La ventana max_translation y max_rotation acota cuánto puede moverse la pose respecto de la predicción. Es lo que evita que un scan ambiguo teletransporte al robot.

Cuando el matching falla, devolver la predicción de odometría en vez de la mejor pose encontrada es deliberado: propagar odometría sola es más seguro que propagar un alineamiento que no pasó los chequeos.
-->

---
### `select_tracking_points(scan, limit)`
- **Entrada:** el scan completo y el máximo de puntos.
- **Salida:** el scan submuestreado.
- Toma puntos espaciados de forma pareja a lo largo del scan.

<!--
El submuestreo es determinista, por índice, y no aleatorio: dos llamadas con el mismo scan dan los mismos puntos, cosa necesaria para que la corrida sea reproducible.

El límite por defecto es 180 puntos. El costo del matching es lineal en la cantidad de puntos y multiplica por iteraciones y por partículas, así que este número es de los que más pesan en el tiempo por scan.
-->

---
### `recover_tracking_scan(field, scan, prior, normal, recovery)`
- **Entrada:** el campo, el scan, la última pose confiable, las opciones normales y las de recuperación.
- **Salida:** la pose recuperada y si se acepta.
- Ensancha la ventana de movimiento y afloja la penalización de odometría.
- Barre 11 posiciones por eje alrededor de la pose previa y ordena las semillas por costo.
- Refina las semillas separadas entre sí y compara los modos que compiten.
- Rechaza si dos modos quedan demasiado parecidos en costo.

<!--
Está separado del tracking normal a propósito. Aflojar la regularización de odometría es peligroso durante la operación normal, porque deja que el matcher se vaya; solo se justifica cuando ya se sabe que el tracking se perdió.

El margen de ambigüedad hace que un pasillo simétrico, donde dos poses opuestas explican igual de bien el scan, no produzca una recuperación con una pose elegida al azar entre las dos.

El resultado no se inserta en el mapa de inmediato: el llamador tiene que corroborarlo en scans posteriores, tantos como diga confirmations. Una recuperación equivocada que se inserta contamina el submapa de forma irreversible.

El barrido es de 11 posiciones por eje, o sea trabajo máximo fijo, independiente del tamaño de la trayectoria.
-->

## [submap.hpp](../belugaslam_core/include/belugaslam_core/submap.hpp)

Define el modelo de datos del mapa: qué es un submapa, cómo se le insertan los scans, y las estructuras del grafo de poses (nodos de trayectoria y restricciones). La estructura que contiene a todas las demás es `Hypothesis`. Cada una tiene sus propios submapas, su propio grafo de trayectoria y su propio estado de tracking.

---
### `insert_scan_into_submap_grid(grid, T_submap_robot, scan, params, hit_scratch, miss_scratch, reusable_updates)`
- **Entrada:** la grilla del submapa, la pose del robot en el marco del submapa, el scan en coordenadas del robot, los parámetros de inserción y buffers reutilizables.
- **Salida:** ninguna.
- Agranda la grilla primero, para cubrir la pose del robot y todos los impactos.
- Fuerza a libre las celdas dentro del radio del robot, antes de aplicar el scan.
- Convierte cada punto del scan a índices de celda y llama a [apply_scan_cells(cells, width, height, origin_x, origin_y, hit, miss, clamp, scratch, hits, misses)](#apply_scan_cellscells-width-height-origin_x-origin_y-hit-miss-clamp-scratch-hits-misses), que marca los impactos y traza los rayos de espacio libre.

<!--
La pose y el scan están los dos en el marco del robot (base_link), no en el del sensor: laser_to_cartesian() ya aplicó la extrínseca del láser antes de que el scan llegue acá. El origen de los rayos es entonces la posición del robot, no la del láser, que es una aproximación válida mientras el láser esté cerca del centro del robot.

Hasta el 9 de septiembre de 2026 el parámetro se llamaba T_submap_sensor y las dos líneas de \param decían "sensor". El nombre estaba mal pero la matemática no, porque la pose y el scan siempre estuvieron en el mismo marco.
-->

---
### `Submap(id, pose, width, height, resolution)`
- **Entrada:** el identificador, la pose del submapa en el marco global y las dimensiones iniciales de su grilla.
- **Salida:** el submapa, activo y vacío.
- Coloca el origen de la grilla en menos la mitad de su tamaño, así la coordenada local (0, 0) queda en el centro y no en una esquina.
- Crea la grilla sin rotación: la orientación del submapa vive en su pose global, no en la grilla.
- Arranca con rol provisional, sin inserciones y sin terminar.

---
### `id()`
- **Entrada:** ninguna.
- **Salida:** el identificador del submapa.

---
### `mutable_grid()`
- **Entrada:** ninguna.
- **Salida:** una referencia modificable a la grilla del submapa.
- Lanza una excepción si el submapa ya está terminado.
- Si la grilla tiene más de un dueño, hace una copia propia antes de devolverla.
- Descarta el campo de distancias, que queda desactualizado apenas se escriba la grilla.

<!--
La comprobación de dueño único es use_count() != 1. Antes era shared_ptr::unique(), que está deprecado desde C++17 y eliminado en C++20: compilaba solo porque el proyecto pide cxx_std_17, y habría dejado de compilar al subir el estándar. Se cambió el 9 de septiembre de 2026; las dos formas son equivalentes.

La copia perezosa es lo que permite que las hipótesis compartan submapas sin copiarlos: recién cuando una va a escribir se separa del resto.
-->

---
### `grid()`
- **Entrada:** ninguna.
- **Salida:** una referencia de solo lectura a la grilla del submapa.

---
### `tracking_field()`
- **Entrada:** ninguna.
- **Salida:** el campo de distancias del submapa.

---
### `global_pose()` / `set_global_pose(pose)`
- **Entrada:** la pose del submapa en el marco global.
- **Salida:** esa misma pose.

---
### `local_pose()` / `set_local_pose(pose)`
- **Entrada:** la pose que le asignó el frontend al submapa.
- **Salida:** esa misma pose.

---
### `anchor_sequence()` / `set_anchor_sequence(sequence)`
- **Entrada:** el número de scan en el que se creó el submapa.
- **Salida:** ese mismo número.

---
### `clone_for_pose()`
- **Entrada:** ninguna.
- **Salida:** un submapa nuevo que comparte grilla, campo de distancias y caché de lazos.
- Copia superficial, pensada para las pruebas de PGO, que mueven submapas pero no cambian su contenido.

---
### `role()` / `set_role(role)`
- **Entrada:** el rol del submapa.
- **Salida:** ese mismo rol.
- Arranca provisional en el constructor y pasa a autoritativo al congelarse.

<!--
El enum declara tres valores pero kRedundant no se asigna en ningún lado, y role() no se lee en ningún punto del código de producción: la única lectura está en un test. O sea que el rol se escribe pero no decide nada, y hoy es equivalente a is_finished(). Parece una distinción prevista para más adelante, quizá para marcar submapas redundantes tras un cierre de lazo y excluirlos de la composición del mapa, que quedó a medio implementar.
-->

---
### `num_insertions()` / `add_insertion()`
- **Entrada:** ninguna.
- **Salida:** cuántos scans se insertaron en este submapa.
- Gobierna el ciclo de vida: al llegar al umbral `submap_num_range_data`, el submapa se congela.

---
### `is_finished()`
- **Entrada:** ninguna.
- **Salida:** si el submapa está congelado.
- Es la condición que consulta [mutable_grid()](#mutable_grid) para lanzar la excepción, y la que usa [clone()](#clone) para decidir si copia la grilla o la comparte.

---
### `finish()`
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Congela el submapa: la transición de activo a inmutable y compartible.
- Se separa de la grilla si la comparte, y la recorta al rectángulo de celdas observadas con [crop_to_known_cells(margin_cells)](#crop_to_known_cellsmargin_cells), dejando 5 celdas de margen.
- Descarta el campo de distancias.
- Marca el submapa como terminado, calcula la firma radial y crea el caché de lazos, vacío.

---
### `LoopMatchingData::bytes()`
- **Entrada:** ninguna.
- **Salida:** cuánta memoria ocupan los dos arreglos del dato.

<!--
No cuenta el propio struct: las dos cabeceras de vector son unos 24 bytes cada una, más lo que agregue el bloque de control del shared_ptr. Es una subestimación de unos 50-80 bytes sobre cientos de kilobytes
-->

---
### `loop_matching_data()`
- **Entrada:** ninguna.
- **Salida:** el dato de emparejamiento de lazos, o un puntero vacío si el submapa todavía no está terminado.
- Delega en [acquire(builder)](#acquirebuilder), pasándole el cálculo como lambda.

---
### `loop_cache_identity()`
- **Entrada:** ninguna.
- **Salida:** la dirección del caché de lazos, como `const void*`.
- Es una llave, no un puntero para usar: el tipo elegido avisa que no hay que desreferenciarlo.

---
### `loop_cache_statistics()`
- **Entrada:** ninguna.
- **Salida:** lo que devuelve [statistics()](#statistics) del caché, o `{0, 0}` si no hay caché.

---
### `release_loop_cache()`
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Libera el caché de lazos, que es memoria reconstruible.
- Entra al caché compartido y lo vacía para todos los clones a la vez.

---
### `release_tracking_field()`
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Libera el campo de distancias, que es memoria reconstruible.
- Resetea el puntero de este objeto solamente: si dos clones comparten el campo, liberar en uno no afecta al otro.

---
### `distance_at(x, y)`
- **Entrada:** un punto en coordenadas locales del submapa.
- **Salida:** la distancia al obstáculo más cercano desde ese punto.
- Delega en [loop_cell_at(x, y, data)](#loop_cell_atx-y-data) y se queda con el primer elemento del par, descartando el puntaje.

<!--
Tampoco se usa en producción, solo en los tests.
-->

---
### `prepare_loop_matching()`
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Fuerza la construcción del caché descartando el resultado, para que el costo se pague antes de entrar a la parte paralela.

---
### `loop_cell_at(x, y)`
- **Entrada:** un punto en coordenadas locales del submapa.
- **Salida:** el par (distancia, puntaje) de esa celda.
- Pide el dato al caché y delega en [loop_cell_at(x, y, data)](#loop_cell_atx-y-data).

<!--
En producción no se usa; las únicas llamadas están en los tests.
-->

---
### `loop_cell_at(x, y, data)`
- **Entrada:** un punto en coordenadas locales del submapa y el dato de emparejamiento de lazos ya obtenido.
- **Salida:** el par (distancia al obstáculo más cercano, puntaje) de la celda que contiene ese punto.
- Convierte el punto a índice de celda y devuelve los dos valores precalculados. No hay cálculo real.
- Un punto fuera de la grilla devuelve distancia infinita y puntaje cero.

---
### `clone()`
- **Entrada:** ninguna.
- **Salida:** un submapa nuevo.
- Copia la grilla solo si el submapa está activo. Si está terminado la comparte, porque nadie la va a poder modificar.

---
### `radial_signature()`
- **Entrada:** ninguna.
- **Salida:** el histograma de celdas ocupadas por distancia al origen del submapa.

---
### `compute_radial_signature()`
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Recorre la grilla y, por cada celda ocupada, calcula su radio desde el origen local y suma uno a la banda correspondiente. Son 50 bandas de 0.5 m, o sea hasta 25 m.
- Divide todo por la cantidad de celdas contadas, así el vector suma 1 y se pueden comparar submapas con distinta cantidad de obstáculos.

---
### `compute_loop_matching_data()`
- **Entrada:** ninguna.
- **Salida:** las dos tablas que usa el emparejador de cierres de lazo: la distancia al obstáculo más cercano de cada celda y el puntaje que le corresponde.
- Marca en cero las celdas ocupadas y deja el resto en infinito.
- Propaga el mínimo con dos barridos sobre la grilla, uno de arriba-izquierda a abajo-derecha y otro al revés. Es el algoritmo de chamfer: tiempo lineal, sin buscar el obstáculo más cercano para cada celda.
- Convierte las distancias a metros y calcula el puntaje de cada una con una gaussiana de 20 cm.

<!--
Es casi el mismo código que TrackingField. El constructor de TrackingField (robust_tracking.hpp:36-56) implementa el mismo chamfer con la misma estructura de dos barridos. Lo que cambia es el posprocesado: aquel satura en 1 metro y conserva la distancia cruda para poder interpolar y derivar, este aplica la gaussiana y guarda el resultado.
La duplicación no es grave, pero si alguna vez se corrige algo en un chamfer hay que acordarse del otro.

Los umbrales de "ocupado" no coinciden. Esta función usa > 0.5F sobre log-odds; TrackingField usa > 0.65F sobre log-odds. Así que el emparejador de lazos considera ocupada una celda con menos evidencia que el tracker.
Puede ser deliberado —al buscar lazos conviene ser más permisivo para no perder candidatos— pero no está documentado en ningún lado, y viendo lo parecidas que son las dos implementaciones, también podría ser un descuido. Vale la pena confirmarlo antes de tocarlo.
-->
---
### `SubmapList` `matching_submap()`
- **Entrada:** ninguna.
- **Salida:** el submapa contra el que se hace el scan matching.
- Si hay una referencia elegida explícitamente devuelve esa; si no, el más viejo de los activos.

---
### `make_active_unique()`
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Reemplaza por un clon propio cada submapa activo que tenga más de un dueño. Si ya es exclusivo, no hace nada.

---
### `find_submap(id)`
- **Entrada:** el identificador de un submapa.
- **Salida:** el submapa, o `nullptr` si no existe.
- Busca primero entre los activos, recorriéndolos uno por uno. Son a lo sumo dos.
- Después busca en el historial por bisección, que puede tener cientos de submapas.

<!--
La bisección funciona porque el historial está ordenado por id. Es una invariante que nadie verifica en tiempo de ejecución.

lower_bound no busca un elemento, devuelve el primero que no es menor que el valor. Si el id no está, igual devuelve una posición válida, la del siguiente id más grande. Por eso hay que verificar además que el elemento encontrado sea el buscado; omitir ese chequeo es un error clásico que devolvería el submapa equivocado.
-->

---
### `find_node(id)`
- **Entrada:** el identificador de un nodo de trayectoria.
- **Salida:** un puntero al nodo, o `nullptr` si no existe.
- Recorre el vector de principio a fin.

<!--
Devuelve un puntero crudo al interior del vector. Si alguien agrega un nodo mientras conserva el puntero, el vector puede realocar y el puntero queda colgado. Hoy no pasa porque las consultas y las inserciones ocurren en fases distintas del ciclo, pero es una restricción implícita que ningún tipo hace cumplir.
-->

---
### `find_node_by_sequence(sequence)`
- **Entrada:** el número de scan del que salió un nodo.
- **Salida:** un puntero al nodo, o `nullptr` si ese scan no fue keyframe.
- Busca por bisección, que es válida porque el vector se mantiene ordenado por `sequence`.

<!--
La advertencia sobre el puntero crudo de find_node() aplica igual acá.
-->

---
### `find_sample(sequence)`
- **Entrada:** el número de scan de una muestra de trayectoria.
- **Salida:** un puntero a la muestra, o `nullptr` si ese scan nunca se registró.
- Busca por bisección sobre el vector de muestras.

<!--
La advertencia sobre el puntero crudo de find_node() aplica igual acá.
-->

---
### `pose_at_sequence(sequence, pose)`
- **Entrada:** el número de scan buscado y una referencia donde escribir la pose.
- **Salida:** `true` si se pudo determinar la pose, `false` si no.
- Si [find_node_by_sequence(sequence)](#find_node_by_sequencesequence) encuentra un nodo, devuelve directamente su `global_pose`, que es la que escribe el optimizador.
- Si no, busca la muestra con [find_sample(sequence)](#find_samplesequence), ubica su submapa con [find_submap(id)](#find_submapid) y compone la pose global del submapa con la pose relativa guardada.
- En los dos casos de falla deja el parámetro `pose` sin tocar.

---
### `insertion_nodes(submap_id)`
- **Entrada:** el identificador de un submapa.
- **Salida:** los identificadores de los nodos cuyo scan se insertó en ese submapa.
- Recorre el vector de restricciones nodo-submapa y se queda con las que apuntan a ese submapa y tienen tag `kIntraSubmap`.

---
### `finish_ready_submaps(max_insertions)`
- **Entrada:** la cantidad de scans a partir de la cual un submapa se cierra.
- **Salida:** los identificadores de los submapas que se cerraron en esta llamada.
- Recorre los submapas activos y, para los que llegaron a la cuenta, llama a [finish()](#finish), les asigna el rol `kAuthoritative`, los mueve al historial y los saca de la lista de activos.

---
### `make_room_for_new_submap(max_active)`
- **Entrada:** la cantidad máxima de submapas activos permitida.
- **Salida:** los identificadores de los submapas que se cerraron en esta llamada.
- Cierra los submapas activos más viejos hasta que quede lugar para uno nuevo, con el mismo procedimiento que [finish_ready_submaps(max_insertions)](#finish_ready_submapsmax_insertions).
- Cierra por posición en la lista, no por cantidad de scans, así que puede cerrar un submapa a medio llenar.

<!--
Con el ciclo actual no cierra nada nunca: las cuentas están sincronizadas y finish_ready_submaps ya cerró el submapa viejo en el scan anterior a que se cree el siguiente. Existe como garantía, para que "como mucho dos submapas activos" sea una invariante y no una consecuencia de que las cuentas den bien. Si se cambiara el criterio de creación, la invariante seguiría valiendo sin tocar nada más.
-->

---
### `bounding_box(min_x, min_y, max_x, max_y)`
- **Entrada:** cuatro referencias donde escribir los límites, en metros.
- **Salida:** `true` si hay al menos un submapa; `false` deja los cuatro parámetros sin tocar.
- Transforma al marco global las cuatro esquinas de la grilla de cada submapa, del historial y de los activos, y se queda con el mínimo y el máximo de cada eje.
- Devuelve el rectángulo alineado con los ejes globales que contiene a todos los submapas.

---
### `inter_constraint_count()`
- **Entrada:** ninguna.
- **Salida:** cuántas restricciones de cierre de lazo tiene el grafo.
- Recorre las restricciones nodo-submapa y cuenta las que tienen tag `kInterSubmap`.

---
### `trim_scan_data_outside_active_submaps()`
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Libera la nube de puntos de los nodos cuyo scan ya no pertenece a ningún submapa activo.
- Junta los ids de los submapas activos, se queda con los nodos que tienen una restricción `kIntraSubmap` hacia alguno de ellos y le hace `reset()` al `constant_data` de todos los demás.

---
### `weighted_mean_pose(poses, weights)`
- **Entrada:** un conjunto de poses SE(2) y sus pesos.
- **Salida:** la pose promedio.
- Promedia la traslación de forma normal y el ángulo con media circular.
- Sin poses devuelve la identidad; si los pesos suman cero o menos, promedia sin pesos.
- Si todas las poses son exactamente opuestas, conserva el ángulo de la primera.


# belugaslam_node

## [fastslam_oc_grid_node.cpp](../belugaslam_node/src/fastslam_oc_grid_node.cpp)

### `BelugaSLAMNode()`
- Constructor de BelugaSLAMNode.
- Obtiene los parámetros del nodo de ROS 2.
- Llama a la función [setup_slam()](#setup_slam).
- Configura TF, suscriptores y publicadores.

---
### `setup_slam()`
- **Entrada:** ninguna.
- **Salida:** ninguna. 
- Obtiene parámetros de ROS 2, los chequea y crea una instancia de FastSLAMParams.
- Crea una instancia de los modelos de medición y movimiento.
- Construye la instancia de BelugaSLAM con el constructor [BelugaSLAM()](#belugaslam).

---
### `laser_callback(msg)`
- **Entrada:** laser scan.
- **Salida:** ninguna. 

Se activa cada vez que se recibe un scan. Registra métricas de latencia y descarta el scan si llega fuera de orden, si no hay transformada odom → base_frame o si queda sin puntos válidos.

- Calcula el control de movimiento con la TF de odometría.
- Convierte el scan a puntos cartesianos en el frame del robot con [laser_to_cartesian(msg, current_odom)](#laser_to_cartesianmsg-current_odom), que además corrige la distorsión por movimiento (deskew).
- Ejecuta el ciclo del filtro: [sample_motion_model(u)](#sample_motion_modelu), [measurement_model_map(z)](#measurement_model_mapz), [update_occupancy_grid(z, stamp)](#update_occupancy_gridz-stamp), [post_update(finished_events)](#post_updatefinished_events) y [resample()](#resample).
- Mide la innovación de la pose de salida (diferencia entre la pose predicha por odometría y la que devuelve el filtro) y detecta si cambió la hipótesis ganadora.
- Calcula la covarianza y publica pose y TF con [compute_se2_covariance()](#compute_se2_covariance), [publish_best_pose(stamp)](#publish_best_posestamp) y [broadcast_map_to_odom(stamp, current_odom)](#broadcast_map_to_odomstamp-current_odom).
- Guarda el timestamp del scan junto con la secuencia que le asignó el core, que necesita [write_final_trajectory()](#write_final_trajectory) para fechar la trayectoria del final. Solo lo hace si se pidió ese archivo.
- Registra los tiempos de cada etapa y los contadores en el CSV de performance con [record_performance(stamp, status, start, timing)](#record_performancestamp-status-start-timing).

<!--
El umbral de movimiento (min_update_distance / min_update_angle) ya no se aplica acá: se movió al core, a motion_filter_accepts().
El mapa, las partículas, la entropía, la trayectoria y los marcadores ya no se publican en el callback; los publica publish_visualization() desde un wall timer aparte.
Los cuatro caminos de salida (out_of_order, empty_scan, tf_error, processed) quedan registrados en el CSV.
-->

---
### `record_performance(stamp, status, start, timing)`
- **Entrada:** timestamp del scan, status (`out_of_order`, `empty_scan`, `tf_error` o `processed`), instante de inicio del callback y el struct `ScanTiming` con las métricas del scan.
- **Salida:** ninguna.
- Mide el tiempo total del callback (`total_ms`).
- Escribe una fila del CSV de performance combinando cuatro fuentes: las métricas del scan (`timing`), los contadores acumulados del nodo (`scans_received_`, `tf_errors_`, `map_publications_`…), el estado actual del filtro ([particles()](#particles) y [get_active_hypotheses_count()](#get_active_hypotheses_count)) y la pose de salida con los datos de la selección de hipótesis.
- Las últimas nueve columnas (`output_x`, `output_y`, `output_yaw`, `output_selection_mode`, `map_hypothesis`, `map_position_risk_m2`, `selected_position_risk_m2`, `polish_solves`, `polish_work_ms`) solo se llenan si el status es `processed`; en las filas rechazadas quedan vacías.
- Imprime en consola un resumen con los tiempos principales y los contadores, como mucho una vez cada 5 s.

<!-- 
Las columnas de salida quedan vacías en las filas rechazadas porque un callback que descartó el scan no produjo una estimación nueva: repetir la anterior haría parecer que hubo una medición donde no la hubo.

record_performance corre al final de todo y lee el estado ahí mismo:
<< slam_->particles().size() << ',' << slam_->get_active_hypotheses_count() << ','
Pero matching_ms, insertion_ms y backend_ms de esa misma fila se midieron antes del remuestreo, con la población anterior. Como el remuestreo KLD cambia la cantidad de partículas, la fila puede decir "50 partículas, 12 ms de matching" cuando el matching en realidad corrió sobre 20.
-->

---
### `publish_visualization()`
- **Entrada:** ninguna.
- **Salida:** ninguna.

La llama un timer aparte (`visualization_publish_period`, 0.2 s), no el callback del láser.

- Publica partículas, entropía y trayectoria solo si el tópico tiene suscriptores, con [publish_particles(stamp)](#publish_particlesstamp) y [compute_entropy()](#compute_entropy).
- Publica los marcadores de loop closure y de split espacial solo cuando cambió la cantidad, con [publish_loop_closure_markers(stamp)](#publish_loop_closure_markersstamp) y [publish_spatial_split_markers(stamp)](#publish_spatial_split_markersstamp).
- Publica el mapa a su propio ritmo, más lento (`map_publish_period`, 1 s), con [publish_map()](#publish_map); y el de incertidumbre cada `uncertainty_map_publish_interval` mapas, con [publish_uncertainty_map()](#publish_uncertainty_map).
- Usa en todos los mensajes el timestamp del último scan procesado, no la hora actual, para que coincidan con el TF map → odom.
- Guarda sus propios tiempos (`last_map_ms_`, `last_visualization_ms_`, `visualization_ticks_`) para el CSV de performance.

<!--
/map se publica aunque no haya suscriptores porque el tópico es transient_local: el middleware guarda el último mensaje y se lo entrega a quien se conecte después.
El timer y la suscripción comparten el callback group por defecto (MutuallyExclusive), así que no corren en paralelo y el acceso a slam_ no necesita mutex.
-->

---
### `laser_to_cartesian(msg, current_odom)`
- **Entrada:** laser scan y la pose de odometría al inicio del scan.
- **Salida:** vector de puntos (x, y) en el frame del robot al inicio del scan.
- Descarta los rayos no finitos o fuera de rango. El máximo es el menor entre `msg->range_max` y el parámetro `range_max`; el mínimo nunca baja de 0.1 m, para no mapear el propio chasis del robot.
- Pasa cada uno de polar a cartesiano en el frame del sensor, con el ángulo reconstruido desde el índice.
- Los lleva a base_link con la extrínseca del sensor (`T_bl_laser`).
- Si `deskew_scan` está habilitado, corrige la distorsión por movimiento: mide el desplazamiento del robot durante el barrido con la odometría del final del scan e interpola por punto según su posición en el barrido.

<!--
La extrínseca se busca en cada scan. T_bl_laser es típicamente estática (viene de un static_transform_publisher o del URDF), así que se está pagando un lookup por scan para un valor que no cambia. No es caro, pero es cacheable si alguna vez perfilás y aparece.
-->

---
### `publish_map()`
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Toma la grilla de ocupación de la mejor hipótesis con [best_occupancy_grid()](#best_occupancy_grid).
- Copia los metadatos de la grilla en cada publicación (resolución, ancho, alto y origen), porque la grilla es dinámica y crece a medida que el robot explora.
- Arma el mensaje `OccupancyGrid` y lo publica en `/map`, estampado con el timestamp del último scan procesado.

<!--
best_occupancy_grid() parece un getter: el nombre es un sustantivo, devuelve una referencia constante y está marcado const. Pero recorre todos los submapas y aplica una sigmoide celda por celda, así que no conviene llamarlo desde el callback del láser.
-->

---
### `publish_best_pose(stamp)`
- **Entrada:** timestamp del scan.
- **Salida:** ninguna.
- Convierte la pose SE(2) del filtro (`slam_->best_pose()`) a una pose 3D: z en cero y el ángulo pasado a cuaternión con roll y pitch nulos.
- Mapea la covarianza SE(2) de 3x3 a la matriz 6x6 aplanada de ROS, que ordena las dimensiones como `[x, y, z, roll, pitch, yaw]`.
- Publica el `PoseWithCovarianceStamped` en `/best_pose`, en el frame map.
- Si `publish_trajectory` está habilitado, agrega la pose al Path de trayectoria y recorta desde el frente al superar `trajectory_max_poses`.

<!--
La covarianza viene de compute_se2_covariance(), que la calcula solo sobre las partículas de la hipótesis seleccionada. No describe la mezcla global.
El Path acumula las poses tal como se publicaron, así que un loop closure no corrige las viejas. Es un historial de lo publicado, no la trayectoria optimizada: para esa hay que reconstruir desde los nodos del grafo.
El erase desde el frente es O(n). Una vez lleno el buffer, cada scan desplaza 5000 elementos. No es dramático, pero un std::deque haría lo mismo en O(1) si alguna vez aparece en un perfilado.
-->

---
### `publish_particles(stamp)`
- **Entrada:** timestamp del scan.
- **Salida:** ninguna.
- Convierte cada pose SE(2) a 3D.
- Publica el `PoseArray` en `/particle_cloud`, en el frame map.

<!--
Publica las partículas de todas las hipótesis mezcladas. PoseArray no tiene color ni id por pose, así que en RViz no se distingue a qué hipótesis pertenece cada una ni cuál es la seleccionada.
-->

---
### `broadcast_map_to_odom(stamp, current_odom)`
- **Entrada:** timestamp del scan y la pose de odometría de ese scan.
- **Salida:** ninguna.
- Calcula la transformación que falta a partir de las dos que sí conoce y lo publica como `map → odom`.

<!--
Se estampa con el tiempo del scan, coherente con todas las demás publicaciones del nodo. Pero tiene una consecuencia práctica: entre un scan y el siguiente, la transformación más nueva del buffer tiene hasta 100 ms de antigüedad. Un consumidor que pida map → base_link en el instante actual va a recibir una excepción de extrapolación.

Los stacks que consultan con Time(0) (la última disponible) no se ven afectados. Pero AMCL y nav2 resuelven esto posdatando la transformación: le suman un transform_tolerance (típicamente 0.1 s) al stamp, declarando que sigue siendo válida un poco hacia adelante. Este nodo no tiene ese parámetro. Si en algún momento ves warnings de extrapolación en nav2, esa es la causa y la solución es sumar una tolerancia configurable acá
-->

---
### `compute_se2_covariance()`
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Recorre solo las partículas de la hipótesis seleccionada.
- Mide la desviación de cada una contra la pose que se va a publicar, no contra el promedio de las partículas.
- Acumula los errores pesados por el peso de cada partícula y divide por la masa de esa hipótesis.
- Si esa masa es cero, devuelve una diagonal de 1e3, que significa incertidumbre total.

<!--
Usar todas las partículas mediría la separación entre hipótesis, no la confianza en la pose publicada: con dos hipótesis a diez metros la matriz daría enorme aunque cada una esté bien localizada.

La covarianza estadística se define alrededor de la media. Si medís alrededor de otro punto p, lo que obtenés es el segundo momento respecto de ese punto, y los dos se relacionan así:
$$M_p = \text{Cov} + (\mu - p)(\mu - p)^\top$$
O sea: lo que calcula el código es la covarianza verdadera más un término extra que depende de cuán lejos esté la media de la pose publicada. Siempre da igual o mayor. Medir contra la media eliminaría ese término.

compute_se2_covariance() se llama en línea 343, después de resample() en la 330. Y usa los pesos
-->

---
### `compute_entropy()`
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Calcula la entropía de Shannon de los pesos de todas las partículas y lo publica en `/localization_entropy`.

<!--
Hay que leerla sabiendo que install_population() reasigna pesos uniformes dentro de cada hipótesis al remuestrear, y resample() corre al final de cada scan. La entropía queda cerca del máximo casi siempre, salvo cuando resample() sale temprano porque el ESS todavía es bueno.

Calcular la entropía en laser_callback, entre post_update y resample, y guardarla en un miembro (como covariance_).
Que compute_entropy —o mejor, publish_entropy— solo publique ese valor desde el timer, manteniendo el chequeo de suscriptores.
Agregarla como columna del CSV de performance, así queda registrada aunque nadie esté suscrito.
El costo en la ruta de tiempo real es despreciable: un recorrido sobre 50 partículas con un logaritmo cada una.

Dos detalles de implementación que importan
Normalizar los pesos. Antes del remuestreo no suman 1: el modelo de medición los escala por verosimilitud. Sin dividir por la suma, el número no es una entropía de Shannon y no es comparable entre scans. Hay que usar $w_i / \sum w$.

Dividir por log(N).
-->

---
### `publish_uncertainty_map()`
- **Entrada:** ninguna.
- **Salida:** ninguna.
- Toma la grilla de log-odds de la mejor hipótesis con [best_log_odds_grid()](#best_log_odds_grid)
- Convierte cada celda de log-odds a probabilidad con la sigmoide y la recorta a (1e-9, 1-1e-9) para no calcular `log(0)`.
- Calcula la entropía binaria de esa probabilidad y la escala a 0-100 dividiendo por `log(2)`, que la pasa a bits: la entropía binaria en bits vale como mucho 1, así que el rango entra justo en el del mensaje `OccupancyGrid`.
- Publica el resultado en `/map_uncertainty`.

---
### `publish_loop_closure_markers(stamp)`
- **Entrada:** timestamp del scan.
- **Salida:** ninguna.
- Sale sin hacer nada si todavía no hubo ningún cierre de lazo.
- Arma una esfera verde por cada pose de [loop_closure_poses()](#loop_closure_poses).
- Publica el `MarkerArray` completo en `/loop_closure_markers`.

---
### `publish_spatial_split_markers(stamp)`
- **Entrada:** timestamp del scan.
- **Salida:** ninguna.
- Igual que [publish_loop_closure_markers(stamp)](#publish_loop_closure_markersstamp), pero con las poses de [spatial_split_poses()](#spatial_split_poses). Publica en `/spatial_split_markers` esferas rojas.

---
### `write_final_trajectory()`
- **Entrada:** ninguna.
- **Salida:** ninguna.
- No hace nada si el parámetro `final_trajectory_path` está vacío.
- Pide la trayectoria del recorrido con [final_trajectory()](#final_trajectory) y la escribe en un CSV: una fila por scan con secuencia, timestamp, submapa, pose online y pose optimizada.
- Se llama desde el destructor del nodo, cuando termina el recorrido.

<!--
El archivo se abre al arrancar, no al escribir: si la ruta no sirve, el nodo falla antes de procesar el dataset en vez de después.

El destructor atrapa las excepciones porque una que se escape de un destructor aborta el proceso. Si el proceso se mata con SIGKILL el archivo queda vacío; ros2 launch manda SIGINT primero, así que en un recorrido normal alcanza.

Los timestamps salen de scan_sequence_stamps_, que laser_callback llena con el stamp de cada scan y la secuencia que le asignó el core. Las filas cuya secuencia nunca recibió un stamp se saltean: no hay contra qué compararlas.

tools/evaluate_trajectory.py lee este CSV. Con el subcomando export-final lo pasa a TUM, y en compare se elige la columna agregando #online o #optimized al final de la ruta.
-->





