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
- **Entrada:** 
- **Salida:** 

---
### measurement_model_map(z)
- **Entrada:** 
- **Salida:** 

---
### update_occupancy_grid(z, stamp)
- **Entrada:** 
- **Salida:** 

---
### resample()
- **Entrada:** 
- **Salida:** 

---
### post_update(finished_events)
- **Entrada:** 
- **Salida:** 

que se encarga de disparar loop closure con los submapas recién cerrados, correr PGO si hay nuevas restricciones, elegir la hipótesis que se publica y armar el mapa global.

- La elección de la hipótesis está en una función aparte, `refresh_output_selection()`, y depende del parámetro `output_selection_mode`: con `map` (el valor por defecto) gana la de mayor peso total, con desempate por id; con `pose_risk` gana la que minimiza el riesgo cuadrático de posición, es decir la que está más cerca del resto de las hipótesis ponderadas por su masa.

<!-- 
z no se usa: la primera línea del cuerpo es literalmente (void)z. El parámetro está en la firma pero descartado, así que la medición no interviene acá.
Los pasos de loop closure están detrás de #if BELUGASLAM_ENABLE_LOOP_CLOSURE. Con loop closure deshabilitado, post_update() se reduce a elegir hipótesis, fijar pose y componer el mapa.
La lógica de select_output_pose vive en output_selection.hpp. Publica siempre una pose que ya existe en alguna hipótesis, en vez de una media entre hipótesis: así la pose que sale queda emparejada con un mapa completo y coherente. El "riesgo" que reporta es una pérdida cuadrática interna en m2, no un RMSE medido contra ground truth.
-->

---
### best_occupancy_grid()
- **Entrada:** 
- **Salida:** 

---
### best_log_odds_grid()
- **Entrada:** 
- **Salida:** 

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

## [grid_update.hpp](../belugaslam_core/include/belugaslam_core/grid_update.hpp)

### `apply_scan_cells(cells, width, height, origin_x, origin_y, hit, miss, clamp, scratch, hits, misses)`
- **Entrada:** 
- **Salida:** 

## [loop_belief.hpp](../belugaslam_core/include/belugaslam_core/loop_belief.hpp)

## [loop_search.hpp](../belugaslam_core/include/belugaslam_core/loop_search.hpp)

## [motion_filter.hpp](../belugaslam_core/include/belugaslam_core/motion_filter.hpp)

## [output_selection.hpp](../belugaslam_core/include/belugaslam_core/output_selection.hpp)

## [particle_proposal.hpp](../belugaslam_core/include/belugaslam_core/particle_proposal.hpp)

## [particle.hpp](../belugaslam_core/include/belugaslam_core/particle.hpp)

### `crop_to_known_cells(margin_cells)`
- **Entrada:** 
- **Salida:** 

---
### `grow_to_include(min_x, min_y, max_x, max_y)`
- **Entrada:** 
- **Salida:** 

## [pose_graph_cost.hpp](../belugaslam_core/include/belugaslam_core/pose_graph_cost.hpp)

## [pose_graph_residual.hpp](../belugaslam_core/include/belugaslam_core/pose_graph_residual.hpp)

## [robust_tracking.hpp](../belugaslam_core/include/belugaslam_core/robust_tracking.hpp)

## [submap.hpp](../belugaslam_core/include/belugaslam_core/submap.hpp)

Define el modelo de datos del mapa: qué es un submapa, cómo se le insertan los scans, y las estructuras del grafo de poses (nodos de trayectoria y restricciones). La estructura que contiene a todas las demás es `Hypothesis`. Cada una tiene sus propios submapas, su propio grafo de trayectoria y su propio estado de tracking.

---
### `insert_scan_into_submap_grid(grid, T_submap_sensor, scan, params, hit_scratch, miss_scratch, reusable_updates)`
- **Entrada:** la grilla del submapa, la pose del robot en el marco del submapa, el scan en coordenadas del robot, los parámetros de inserción y buffers reutilizables.
- **Salida:** ninguna.
- Agranda la grilla primero, para cubrir el origen del sensor y todos los impactos.
- Fuerza a libre las celdas dentro del radio del robot, antes de aplicar el scan.
- Convierte cada punto del scan a índices de celda y llama a [apply_scan_cells(cells, width, height, origin_x, origin_y, hit, miss, clamp, scratch, hits, misses)](#apply_scan_cellscells-width-height-origin_x-origin_y-hit-miss-clamp-scratch-hits-misses), que marca los impactos y traza los rayos de espacio libre.

<!--
El parámetro se llama T_submap_sensor y el comentario dice que el scan viene en el marco del sensor, pero las dos cosas son incorrectas: el sitio de llamada pasa T_s_r (submapa ← robot) y el scan ya viene en base_link porque laser_to_cartesian() aplicó la extrínseca. La matemática está bien, porque los dos argumentos están en el mismo marco; el error es de nombre.
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
Usa shared_ptr::unique(), que está obsoleto desde C++17 y eliminado en C++20. Hoy compila porque el proyecto usa cxx_std_17. El reemplazo equivalente es use_count() != 1.
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





