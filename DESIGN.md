# Arquitectura detallada de módulos clave

Este documento define la arquitectura interna propuesta para los módulos `core.engine`, `core.thermo` y `core.valvetrain` del simulador 0D/1D de motores de combustión de altas rpm. Se describen clases, atributos principales, métodos públicos y relaciones con otros módulos.

## core.engine

### Clases principales

- **Engine**
  - Atributos: `n_cylinders`, `layout` (L/V/flat), `v_angle`, `firing_order` (lista de índices de cilindro), `rpm_schedule` (valor instantáneo o mapa), `banks` (lista de `Bank`), `shared_intake_manifold`, `shared_exhaust_manifold`, parámetros globales de operación (modo rápido/detallado, referencia de sincronización).
  - Métodos públicos: `from_config(config_dict)`, `build_standard(layout, displacement, n_cylinders, v_angle=None, stroke_to_bore=None)`, `get_cylinder(cid)`, `get_bank(bid)`, `volume_at(crank_angle, cid)`, `volume_derivative(crank_angle, cid)`, `piston_kinematics(crank_angle, cid)` (posición/velocidad/aceleración), `firing_angle(cid)` (ángulo absoluto de referencia), `synchronize_with(valvetrain)` (valida offset de levas) y helpers para iterar cilindros en orden de encendido.

- **Bank**
  - Atributos: `bank_id`, `bank_angle` (respecto al plano de referencia), `cylinders` (lista de `Cylinder`), referencias opcionales a árboles de levas o colectores específicos.
  - Métodos públicos: `add_cylinder(cylinder)`, `get_cylinders()`, `relative_firing_sequence()`.

- **Cylinder**
  - Atributos: `cylinder_id`, `bore`, `stroke`, `rod_length`, `compression_ratio`, `deck_clearance`, `wrist_pin_offset`, `crank_offset` (fase de referencia), `bank_id`, `clearance_volume`, parámetros geométricos derivados (área de pistón, desplazamiento unitario).
  - Métodos públicos: `instant_volume(crank_angle)`, `instant_volume_derivative(crank_angle)`, `piston_state(crank_angle)` (posición/velocidad/aceleración), `top_dead_center_angle()`, `bottom_dead_center_angle()`.

### Relaciones y uso por otros módulos

- `core.cycle` consulta a `Engine`/`Cylinder` para obtener volumen instantáneo y cinemática al avanzar el estado termodinámico por ángulo de cigüeñal.
- `core.valvetrain` se sincroniza mediante `engine.firing_angle` y los offsets de banco para alinear eventos de apertura/cierre con cada cilindro.
- `core.gasdynamics` consume datos de disposición (bancos, colectores, orden de encendido) para asignar condiciones de contorno 1D y la topología de conductos.
- `io.config` construye instancias vía `Engine.from_config` usando parámetros de JSON/YAML; `core.exploration` y `core.optimization` pueden generar variantes llamando a `build_standard` con diferentes dimensiones.

## core.thermo

### Clases y estructuras

- **CylinderState**
  - Atributos: `pressure`, `temperature`, `mass`, `burn_fraction`, `fuel_mass`, `equivalence_ratio`, `gamma_effective`, `cp`, `cv`, `exhaust_residual_fraction`, indicadores de modo (`fast_fill_model` vs `coupled_1d`).
  - Métodos públicos: `clone()`, `update_properties(thermo_table)`, `reset_burn()`, acceso a propiedades derivadas (densidad, energía interna específica).

- **CombustionModel** (por ejemplo, Wiebe parametrizable)
  - Atributos: `theta_start`, `duration`, `m_shape`, `n_shape`, `mass_fraction_burned_curve` (opcional pretabulada), `heat_release_lhv`, correcciones por rpm/carga.
  - Métodos públicos: `mass_fraction_burned(crank_angle)`, `heat_release_rate(crank_angle)`, `configure(params_dict)`, `shift(theta_offset)`, `scale_duration(factor)` para estrategias de optimización.

- **ThermoSolver**
  - Atributos: referencias a `Cylinder`, `CylinderState`, `CombustionModel`, parámetros del modo de cálculo (`fast_mode`, `detailed_mode`, resolución de ángulo), modelos de pérdidas mecánicas (para BMEP), banderas de acoplamiento 1D.
  - Métodos públicos: `step(crank_angle, delta_theta, mass_in, mass_out, enthalpy_in, enthalpy_out)` (actualiza estado 0D), `compute_pressure_trace(crank_window)`, `indicated_work()`, `imep()`, `bmep(mechanical_loss_model)`, `set_boundary_conditions(...)`, `enable_fast_fill(empirical_model)`.

### Relaciones y flujo de información

- `ThermoSolver` requiere geometría de `core.engine.Cylinder` para volumen/derivada en cada paso y usa la alzada/área efectiva de válvulas de `core.valvetrain` (vía `core.cycle`) para estimar masas entrantes/salientes.
- En modo rápido, `ThermoSolver` consume modelos empíricos de llenado proporcionados por `core.performance` o `core.exploration` para estimar `mass_in/mass_out` sin resolver 1D.
- En modo detallado, las condiciones de contorno y caudales provienen de `core.gasdynamics`, mientras que los resultados de presión/temperatura se devuelven a `core.gasdynamics` y `core.acoustics`.
- Configuración: parámetros de `CombustionModel` y estados iniciales se cargan desde JSON/YAML a través de `io.config` y se asignan a cada cilindro.

## core.valvetrain

### Clases principales

- **CamProfile**
  - Atributos: `angle_lift_table` (par tabla ángulo de cigüeñal → alzada), `duration_nominal`, `max_lift`, `lobe_separation_angle`, `centerline_timing` (IVC/EVC), metadatos de origen (CSV/JSON), parámetros de suavizado.
  - Métodos públicos: `lift_at(crank_angle)`, `derivative_at(crank_angle)`, `curtain_area(valve_diameter, crank_angle)`, `apply_advance(delta_deg)`, `apply_duration_scale(factor)`, `to_dict()`/`from_dict()` para import/export, `resample(resolution_deg)`.

- **Valve**
  - Atributos: `valve_id`, `type` (intake/exhaust), `diameter`, `seat_angle`, `cam_profile` (referencia a `CamProfile`), `stem_length`, `clearance`, `flow_coefficient_map` (opcional), restricciones de movimiento (VVL/VVT).
  - Métodos públicos: `lift(crank_angle)`, `effective_area(crank_angle, discharge_coeff_model=None)`, `apply_global_phase(delta_deg)`, `modify_lift(factor)`.

- **ValvetrainAssembly**
  - Atributos: `banks` (mapeo banco → árbol de admisión/escape), `cylinder_valves` (mapeo cilindro → lista de `Valve`), `cam_advance_intake`, `cam_advance_exhaust`, parámetros de VVT/VVL por modo, referencias a sincronización con `Engine` (offsets por cilindro).
  - Métodos públicos: `assign_valve_to_cylinder(valve, cylinder_id)`, `intake_lift(cylinder_id, crank_angle)`, `exhaust_lift(cylinder_id, crank_angle)`, `intake_effective_area(cylinder_id, crank_angle)`, `exhaust_effective_area(cylinder_id, crank_angle)`, `apply_global_intake_advance(delta_deg)`, `apply_global_exhaust_advance(delta_deg)`, `export_profiles(format)`, `import_profiles(data)`, `generate_basic_profile(duration, max_lift, lsa, advance)`.

### Relaciones y uso por otros módulos

- `core.cycle` consulta `ValvetrainAssembly` para obtener alzadas y áreas efectivas en cada ángulo al calcular `mass_in/mass_out` y establecer condiciones de contorno hacia `core.gasdynamics` o modelos empíricos.
- `core.gasdynamics` usa las áreas efectivas y coeficientes de descarga como entrada a condiciones de contorno 1D (válvulas como conectores entre cilindro y conductos).
- `core.engine` sincroniza ángulos de referencia de cilindros con `ValvetrainAssembly` para considerar offsets de banco y orden de encendido en la evaluación de perfiles.
- `core.exploration` y `core.optimization` ajustan globalmente `cam_advance_intake/ exhaust`, duración y alzada via métodos de `CamProfile`/`Valve` para estudios paramétricos.

## Interacción general entre módulos

1. `io.config` crea `Engine`, `ValvetrainAssembly`, `CombustionModel` y estados iniciales (`CylinderState`) desde JSON/YAML.
2. `core.cycle` itera ángulos: consulta `Engine` para volumen/derivadas, `ValvetrainAssembly` para áreas de válvula y `ThermoSolver` para avanzar el estado; intercambia caudales con `core.gasdynamics` en modo detallado.
3. `core.performance` integra los resultados de `ThermoSolver` (IMEP/BMEP) con rpm de `Engine` para curvas de par/potencia; `core.acoustics` usa presiones de escape si está acoplado.
4. `core.exploration`/`core.optimization` parametrizan geometrías y levas usando las APIs públicas para generar variantes y ejecutar pipelines de simulación.

## core.gasdynamics

### Clases y estructuras

- **Pipe**
  - Atributos: `length`, `inner_diameter`, `roughness`, `n_segments`, `wall_temperature`, `material`, discretización interna (`segments`: lista de `Segment`).
  - Métodos públicos: `discretize()`, `segment_at(index)`, `friction_factor(reynolds_number)`, `characteristic_impedance()`.

- **Segment/Cell**
  - Atributos: `pressure`, `temperature`, `density`, `velocity`, `mass_flux`, `enthalpy`, `position`, `area`, `time_step_local` (para CFL), `gamma_effective`, `speed_of_sound`.
  - Métodos públicos: `state_vector()`, `update_state(new_values)`, `compute_time_step(cfl_number)`, `as_boundary_conditions()`.

- **Junction**
  - Atributos: `junction_id`, `connected_pipes` (lista de referencias a `Pipe` + posición), `loss_coefficients`, `volume` (si aplica), `mixing_temperature_model`.
  - Métodos públicos: `apply_mass_energy_balance()`, `compute_reflections()`, `set_boundary_conditions()`.

- **Plenum**
  - Atributos: `plenum_id`, `volume`, `pressure`, `temperature`, `mass`, `gamma_effective`, `inlet_outlet_connections` (a `Pipe`/`Junction`), coeficientes de pérdida.
  - Métodos públicos: `update(mean_mass_flow, mean_enthalpy, dt)`, `as_boundary_conditions()`, `reset(initial_state)`.

- **GasNetwork**
  - Atributos: `pipes`, `junctions`, `plenums`, `boundary_nodes` (atmósfera, válvulas), `coupling_map` (cilindro ↔ nodo), parámetros numéricos (`cfl`, `scheme`, `time_base`: tiempo real o ángulo), historial de señales.
  - Métodos públicos: `from_config(network_config)`, `initialize_network()`, `step(delta_t, cylinder_boundaries, valve_areas)`, `pressure_history(node_id)`, `mass_flow_history(node_id)`, `export_state()`.

### Dinámica y esquema numérico

- Esquema: variantes explícitas de características o Lax-Wendroff 1D para gases ideales con correcciones de pérdidas (Fanno/isentropic); `compute_time_step` por CFL en cada `Segment`.
- Integración: `GasNetwork.step` avanza todas las celdas usando el paso global (mínimo de CFL) o un paso controlado por avance de ángulo; actualiza variables conservadas y aplica fuentes por fricción y transferencia de calor simplificada.
- Condiciones de contorno: en válvulas, intercambio de masa/energía con `core.thermo` via caudal choked/subsonic basado en `valve_areas` (de `core.valvetrain`); en plenos, balance 0D de masa/energía; salida a atmósfera mediante condición de presión fija y coeficiente de reflexión configurable.

### Flujo de datos

- Entradas: geometría de conductos/plenos desde `io.config`, áreas de válvula desde `core.valvetrain`, presiones/temperaturas de cilindro desde `core.thermo` (a través de `core.cycle`).
- Salidas: caudales y presiones en nodos de válvula devueltos a `core.cycle` para actualizar estados 0D; historial de presión en cola de escape para `core.acoustics` y curvas de VE/pérdidas para `core.performance`.

## core.cycle

### Clase principal

- **EngineSimulator** (o `CycleRunner`)
  - Atributos: referencias a `Engine`, `ValvetrainAssembly`, `ThermoSolver` por cilindro, `GasNetwork`, configuraciones de modo (`fast_mode`, `detailed_mode`), malla de ángulo (`delta_theta`), estado de rpm actual, buffers de resultados (presión por cilindro, par instantáneo, VE), tolerancias de convergencia.
  - Métodos públicos:
    - `run_steady_state(rpm)`: itera ciclos completos hasta estado cuasi estacionario, devuelve métricas agregadas (torque medio, potencia, IMEP/BMEP por cilindro y global, VE).
    - `run_rpm_sweep(rpm_grid)`: ejecuta `run_steady_state` en una malla de rpm y agrega curvas rpm–par–potencia–VE.
    - `step_cycle(delta_theta=None)`: avanza un paso de ángulo sobre todos los cilindros; coordina llamadas a `ValvetrainAssembly` para áreas de válvula, a `ThermoSolver.step` por cilindro y a `GasNetwork.step` cuando está acoplado.
    - `compute_instantaneous_torque(crank_angle)`: suma contribuciones indicadas de cada cilindro y pérdidas mecánicas.
    - `export_results()`: retorna estructura con trazas de presión, par, VE y estados de la red.

### Modos de operación

- Modo rápido: `EngineSimulator` desactiva `GasNetwork`, usa modelos empíricos de llenado en `ThermoSolver` y áreas de válvula para estimar caudales; útil para barridos masivos.
- Modo detallado: acopla `ThermoSolver` con `GasNetwork` en cada paso; usa rpm fija o variable según `rpm_schedule`; permite extraer señales transitorias para `core.acoustics`.

### Flujo de datos

- Entrada: objetos de `io.config` (motor, levas, red, combustión, opciones de simulación), rpm objetivo o malla de rpm.
- Interacción: por ángulo, consulta geometría a `Engine`, alzadas/áreas a `ValvetrainAssembly`, actualiza estados 0D con `ThermoSolver`, intercambia condiciones de contorno con `GasNetwork` si procede.
- Salida: métricas de rendimiento para `io.results` y `core.performance`, trazas de presión/caudal para `core.acoustics`, datos de VE y caudales para validación.

## core.acoustics

### Componentes principales

- **AcousticPostProcessor**
  - Atributos: `sampling_rate`, `rpm_reference`, `windowing_options`, `normalization_mode`, `filter_options` (HP/LP/BP), `spectrum_config` (tamaño de FFT, promedio), historial de señales recibidas.
  - Métodos públicos:
    - `angle_to_time(pressure_trace, rpm)`: convierte trazas vs ángulo a vs tiempo.
    - `resample_to_audio(signal, target_fs)`: remuestrea y filtra para generar señal audible.
    - `normalize(signal)`: ajusta amplitud evitando clipping (opcionalmente con headroom configurable).
    - `generate_exhaust_sound(simulation_result)`: pipeline completo que toma presión en cola de escape (de `core.gasdynamics` o modelo simplificado), convierte a audio y devuelve serie temporal.
    - `compute_spectrum(signal)`: calcula FFT y devuelve magnitud/frecuencia con metadatos.
    - `export_audio(signal, path)`: delega en `io.audio` para escribir `.wav` con metadatos de rpm y condiciones.
    - `export_spectrum(spectrum, path)`: serializa espectros en CSV/JSON vía `io.results`.

### Flujo de datos

- Entradas: historial de presión en la cola de escape de `GasNetwork` o señales generadas por modelos rápidos; rpm actual desde `core.cycle` para la conversión ángulo-tiempo.
- Salidas: serie de audio y espectros para análisis o reproducción; datos pasan a `io.audio`/`io.results` y gráficos opcionales.

## io.config

### Responsabilidad

- Leer configuraciones JSON/YAML, validar estructura y construir objetos de dominio (`Engine`, `ValvetrainAssembly`, `GasNetworkConfig`, `CombustionModel`, opciones de simulación).

### Clases y funciones

- **ConfigSchema**
  - Atributos: definiciones de campos esperados para secciones `engine`, `valvetrain`, `gasdynamics`, `thermo/combustion`, `operation` (rpm, modos), `simulation_options` (resolución angular, modo rápido/detallado), `acoustics`.
  - Métodos públicos: `validate(raw_config)`, `default_values()`, `schema_version()`.

- **ConfigLoader**
  - Métodos públicos: `load_config(path)`, `validate_config(raw_config)`, `build_engine(config)`, `build_valvetrain(config)`, `build_gas_network(config)`, `build_thermo_models(config)`, `build_acoustics(config)`, `build_simulation_options(config)`.
  - Salida: estructura compuesta (por ejemplo `SimulationSetup`) con instancias listas para `core.cycle`.

- **SimulationSetup**
  - Atributos: instancias configuradas de `Engine`, `ValvetrainAssembly`, `GasNetwork` (o su configuración para inicializar), `ThermoSolver`/`CombustionModel`, opciones de operación (rpm, grid), flags de modos, rutas de salida.
  - Métodos públicos: `to_dict()`, `from_dict()`, `summary()`.

### Flujo de datos

- Entrada: archivo JSON/YAML del usuario.
- Proceso: `ConfigLoader.load_config` parsea y valida con `ConfigSchema`; construye objetos o diccionarios de parámetros para inicializar módulos `core.*`.
- Salida: `SimulationSetup` se pasa a `core.cycle.EngineSimulator` y a cualquier orquestador CLI/API.

## io.results

### Funciones y estructuras

- **ResultWriter**
  - Métodos públicos: `save_rpm_curves(results, path, formats=['csv','json'])`, `save_pressure_history(signal, path)`, `save_mass_flow_history(signal, path)`, `save_spectrum(spectrum, path)`, `save_simulation_metadata(setup, path)`.
  - Opcionales: `plot_rpm_curves(results, path=None)`, `plot_pressure_trace(signal, path=None)`, `plot_spectrum(spectrum, path=None)` usando Matplotlib.

- **ResultBundle**
  - Atributos: `rpm_curves` (par, potencia, VE), `cylinder_traces` (presión vs ángulo), `network_traces` (presión/caudal en nodos), `acoustic_signals`, `spectra`, `metadata` (configuración, versión de modelo, fecha).
  - Métodos públicos: `merge(other_bundle)`, `filter_by_rpm(rpm_range)`, `to_dict()`.

### Flujo de datos

- Entrada: resultados agregados de `core.cycle` (curvas y trazas), señales de `core.gasdynamics` y audio/espectros de `core.acoustics`.
- Salida: archivos CSV/JSON y gráficos opcionales; provee insumos para `core.validation` y para reporting por CLI/API.

## app.cli

### Comandos conceptuales y parámetros clave

- **sim-0d**
  - Parámetros: `--config path` (JSON/YAML), `--rpm value` o `--rpm-map path`, `--output dir`, `--angle-step deg`, `--fast-fill-model name`.
  - Resultados: archivos CSV/JSON con par/potencia/VE/IMEP/BMEP, metadatos de simulación; opcionalmente gráficas y trazas de presión por cilindro.

- **sim-1d**
  - Parámetros: `--config path`, `--rpm value|map`, `--output dir`, `--cfl-safety factor`, `--angle-step deg`, `--max-cycles n`.
  - Resultados: mismos que `sim-0d` pero incluyendo trazas de red 1D (presión/caudal en nodos), historial de convergencia y datos para acústica.

- **rpm-sweep**
  - Parámetros: `--config path`, `--rpm-grid start:end:step` o lista, `--mode 0d|1d`, `--output dir`.
  - Resultados: curvas rpm–par–potencia–VE en CSV/JSON; gráficos opcionales.

- **sound**
  - Parámetros: `--config path`, `--rpm value`, `--mode 0d|1d`, `--audio-fs hz`, `--output dir`.
  - Resultados: archivo `.wav` y espectro en CSV/JSON generados desde la señal de escape.

- **explore**
  - Parámetros: `--config path`, `--exploration-config path`, `--mode 0d|1d`, `--rpm value|grid`, `--output dir`, `--max-samples n`.
  - Resultados: tabla de combinaciones exploradas con métricas clave; mejores configuraciones exportables.

- **optimize**
  - Parámetros: `--config path`, `--optimization-config path`, `--rpm-range min max`, `--mode 0d|1d`, `--output dir`, `--method grid|random|hill`.
  - Resultados: configuración óptima y top N, curvas comparativas vs configuración base, tabla con T_mean, P_mean y objetivo J.

### Relaciones

- Cada comando invoca `io.config` para cargar/validar el setup, llama a `core.cycle` (modo 0D o 1D) y utiliza `io.results`/`io.audio` para persistir salidas.
- `explore` y `optimize` orquestan `core.exploration` y `core.optimization`, respectivamente, y reutilizan los resultados para construir reportes.

## app.api

### Funciones de alto nivel

- `run_simulation(config_obj, mode="0d"|"1d", rpm=value|schedule) → SimulationResult`
  - Entrada: estructura de configuración validada (`SimulationSetup`), selección de modo y rpm o mapa de rpm.
  - Salida: objeto `SimulationResult` con par/potencia/VE/IMEP/BMEP, trazas por cilindro y de red según el modo.

- `run_rpm_sweep(config_obj, rpm_grid, mode="0d"|"1d") → SweepResult`
  - Entrada: setup más malla de rpm.
  - Salida: curvas rpm–par–potencia–VE y metadatos de cada punto.

- `generate_exhaust_sound(sim_result, audio_settings) → AudioBundle`
  - Entrada: `SimulationResult` que incluya presión en cola de escape; ajustes de muestreo/normalización.
  - Salida: señal de audio remuestreada, espectro y metadatos listos para exportar.

- `run_exploration(config_obj, exploration_settings) → ExplorationResult`
  - Entrada: setup base más definición de parámetros a barrer y muestreo.
  - Salida: tabla de combinaciones con métricas agregadas y mejores configuraciones.

- `run_optimization(config_obj, optimization_settings) → OptimizationResult`
  - Entrada: setup base más rangos/cotas de parámetros y pesos de objetivo.
  - Salida: configuración óptima p*, top N y comparación con base.

### Relaciones

- Las funciones instancian `core.cycle.EngineSimulator` con objetos de `io.config`, delegan en `core.exploration` o `core.optimization` cuando aplica, y usan `core.acoustics` para sonido.
- `SimulationResult`, `SweepResult`, `ExplorationResult` y `OptimizationResult` son contenedores serializables que `io.results` puede escribir.

## core.exploration

### Configuración de exploración

- **ExplorationParameter**: nombre del parámetro (ej. `intake.primary_length`), mínimo, máximo, pasos o distribución (grid, latin-hypercube, aleatorio), tipo (geométrico, lineal), flag de fijación.
- **ExplorationSettings**: lista de `ExplorationParameter`, tamaño máximo de muestras, modo de simulación (0D/1D), rpm única o malla, criterios de ordenación (por T_mean, P_mean, VE), filtros postproceso.

### Motor de exploración

- **Explorer**
  - Atributos: referencia al setup base, generador de muestras (grid builder o sampler), `EngineSimulator` o wrapper de simulación, buffers de resultados.
  - Métodos públicos: `generate_samples(settings)`, `run_all(settings)`, `collect_metrics(rpm_range=None)`, `top_n(n, metric)`, `export_table()`.

### Flujo de datos

- Entrada: setup base desde `io.config`, definición de parámetros y rangos desde archivo de exploración.
- Proceso: `generate_samples` crea combinaciones; cada combinación genera un setup modificado (ajustando geometría, levas o calibración) y ejecuta `core.cycle` en modo elegido; métricas (T_mean, P_mean, VE) se calculan sobre rpm especificada o rango.
- Salida: tabla ordenable/filtrable, exportable vía `io.results`; top N se usa como semillas para `core.optimization`.

## core.optimization

### Configuración de optimización

- **OptimizationParameter**: nombre, cota_inferior, cota_superior, resolución/paso, fijo (bool), tipo de ajuste (geométrico/lineal), sensibilidad esperada.
- **OptimizationSettings**: rango rpm (`rpm_min`, `rpm_max`), pesos `w_T`, `w_P`, tamaño de población/muestras, método (`grid`, `random`, `hill`), límites de iteraciones, configuración inicial opcional.

### Motor de optimización

- **Optimizer**
  - Atributos: setup base, método seleccionado, generador de vecinos (para `hill`), `EngineSimulator` o wrapper, historial de evaluaciones.
  - Métodos públicos: `evaluate(p_vector) → (T_mean, P_mean, J)`, `run(settings) → OptimizationResult`, `neighbor(p_current)` (para hill-climbing), `sample_random()` (para random search), `grid_iter()` (para grid search), `best()`.

### Flujo de datos

- Entrada: configuración base y espacios de parámetros; métricas objetivo definidas por `T_mean` y `P_mean` en el rango [rpm_min, rpm_max].
- Proceso: cada candidato `p` modifica el setup (geometría, levas, calibración), ejecuta `core.cycle.run_rpm_sweep` en la malla del rango y calcula `J = w_T*T_mean + w_P*P_mean`; métodos de búsqueda controlan la exploración del espacio.
- Salida: configuración óptima p*, top N y comparación con base; resultados preparados para `io.results` y para alimentar pipelines de validación.

## core.validation

### Tipos de tests

- **Unitarios numéricos**: conservación de masa/energía en `core.gasdynamics` para conductos simples; validación de `ThermoSolver` en procesos adiabáticos/isotermos; cálculo de volumen y cinemática en `core.engine`.
- **Regresión**: comparación de salidas completas (curvas rpm–par–potencia, trazas de presión) contra archivos de referencia; umbrales de tolerancia configurables.
- **Coherencia física**: checks automáticos de tendencias (p.ej., aumentar VE en 5% debe aumentar par en banda objetivo dentro de rango esperado; adelantos de levas modifican VE en la dirección correcta).

### Integración y flujo

- Integración con `pytest` u otro runner para ejecutar suites; fixtures que cargan configuraciones base desde `io.config` y resultados de referencia desde `io.results`.
- Casos base: cilindro único atmosférico sin onda (validación 0D), conducto recto con onda reflejada (validación 1D), ciclo completo con levas básicas.
- Resultados de validación se almacenan vía `io.results` para trazabilidad; los checks se usan en CI para detectar regresiones al modificar modelos numéricos.
