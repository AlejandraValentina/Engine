# Arquitectura detallada del simulador 0D/1D de motores ICE

Este documento describe la arquitectura completa del simulador 0D/1D de motores de combustión interna de altas rpm. Incluye la estructura de paquetes, clases, atributos y métodos públicos esperados para `core.engine`, `core.thermo`, `core.valvetrain`, `core.gasdynamics`, `core.cycle`, `core.acoustics`, `core.performance`, `core.exploration`, `core.optimization`, `core.validation`, así como los módulos de entrada/salida `io.config`, `io.results`, `io.audio` y las capas de interfaz `app.cli` y `app.api`. Sirve como fuente de verdad para la implementación sin introducir módulos o tipos no definidos.

## core.engine

### Clases principales

- **Engine**
  - Atributos: `n_cylinders`, `layout` (L/V/flat), `v_angle`, `firing_order` (lista de índices de cilindro), `rpm_schedule`, `banks` (lista de `Bank`), `shared_intake_manifold`, `shared_exhaust_manifold`, parámetros globales de operación (modo rápido/detallado, referencia de sincronización).
  - Formato de `rpm_schedule`: admite `float` para rpm fija o lista de objetos `{rpm: float, duration_deg: float}` indicando rpm objetivo y la duración en grados de cigüeñal antes de cambiar al siguiente punto.
  - Métodos públicos: `from_config(config_dict)`, `build_standard(layout, displacement, n_cylinders, v_angle=None, stroke_to_bore=None)`, `get_cylinder(cid)`, `get_bank(bid)`, `volume_at(crank_angle, cid)`, `volume_derivative(crank_angle, cid)`, `piston_kinematics(crank_angle, cid)` (posición/velocidad/aceleración), `firing_angle(cid)` (ángulo absoluto de referencia), `synchronize_with(valvetrain)` (valida offset de levas) y helpers para iterar cilindros en orden de encendido.

- **Bank**
  - Atributos: `bank_id`, `bank_angle` (respecto al plano de referencia), `cylinders` (lista de `Cylinder`), referencias opcionales a árboles de levas o colectores específicos.
  - Métodos públicos: `add_cylinder(cylinder)`, `get_cylinders()`, `relative_firing_sequence()`.

- **Cylinder**
  - Atributos: `cylinder_id`, `bore`, `stroke`, `rod_length`, `compression_ratio`, `deck_clearance`, `wrist_pin_offset`, `crank_offset` (fase de referencia), `bank_id`, parámetros geométricos derivados (área de pistón, desplazamiento unitario). `clearance_volume` no se almacena: se deriva siempre a partir de `compression_ratio` y cilindrada para evitar inconsistencias.
  - Métodos públicos: `instant_volume(crank_angle)`, `instant_volume_derivative(crank_angle)`, `piston_state(crank_angle)` (posición/velocidad/aceleración), `top_dead_center_angle()`, `bottom_dead_center_angle()`.

### Relaciones y uso por otros módulos

- `core.cycle` consulta a `Engine`/`Cylinder` para volumen instantáneo y cinemática al avanzar el estado termodinámico por ángulo de cigüeñal.
- `core.valvetrain` se sincroniza mediante `engine.firing_angle` y los offsets de banco para alinear eventos de apertura/cierre con cada cilindro.
- `core.gasdynamics` consume datos de disposición (bancos, colectores, orden de encendido) para asignar condiciones de contorno 1D y la topología de conductos.
- `io.config` construye instancias vía `Engine.from_config` usando parámetros de JSON/YAML; `core.exploration` y `core.optimization` pueden generar variantes llamando a `build_standard` con diferentes dimensiones.

## core.thermo

### Clases y estructuras

- **CylinderState**
  - Atributos: `pressure`, `temperature`, `mass`, `burn_fraction`, `fuel_mass`, `equivalence_ratio`, `gamma_effective`, `cp`, `cv`, `exhaust_residual_fraction`, indicadores de modo (`fast_fill_model` vs `coupled_1d`).
  - Política de propiedades: `gamma_effective`, `cp` y `cv` se recalculan mediante `update_properties(thermo_table)` siempre que cambie `pressure`, `temperature` o la composición; no se consideran constantes almacenadas salvo para registrar el último valor usado.
  - Métodos públicos: `clone()`, `update_properties(thermo_table)`, `reset_burn()`, acceso a propiedades derivadas (densidad, energía interna específica).

- **CombustionModel** (por ejemplo, Wiebe parametrizable)
  - Atributos: `theta_start`, `duration`, `m_shape`, `n_shape`, `mass_fraction_burned_curve` (opcional pretabulada), `heat_release_lhv`, correcciones por rpm/carga.
  - Métodos públicos: `mass_fraction_burned(crank_angle)`, `heat_release_rate(crank_angle)`, `configure(params_dict)`, `shift(theta_offset)`, `scale_duration(factor)`.

- **ThermoSolver**
  - Atributos: referencias a `Cylinder`, `CylinderState`, `CombustionModel`, parámetros del modo de cálculo (`fast_mode`, `detailed_mode`, resolución de ángulo), modelos de pérdidas mecánicas (para BMEP), banderas de acoplamiento 1D.
  - Métodos públicos: `step(crank_angle, delta_theta, mass_in, mass_out, enthalpy_in, enthalpy_out)` (actualiza estado 0D), `compute_pressure_trace(crank_window)`, `indicated_work()`, `imep()`, `bmep(mechanical_loss_model)`, `set_boundary_conditions(...)`, `enable_fast_fill(empirical_model)`.
  - Convenciones de `step`: `mass_in` y `mass_out` representan la masa total intercambiada durante el paso de ángulo `delta_theta` (integral de caudal másico sobre el paso); `enthalpy_in` y `enthalpy_out` son las entalpías asociadas a esas masas (masa × entalpía específica promedio en el paso).

- **EmpiricalFillModel** (interfaz mínima para modo rápido)
  - Responsabilidad: dado rpm y parámetros de estado, devolver masa fresca por ciclo o VE estimada.
  - API esperada: `estimate_mass_flow(rpm, cylinder, valvetrain_state, ambient_conditions) → {mass_in_cycle, residual_fraction}`.

### Relaciones y flujo de información

- `ThermoSolver` requiere geometría de `core.engine.Cylinder` para volumen/derivada en cada paso y usa la alzada/área efectiva de válvulas de `core.valvetrain` (vía `core.cycle`) para estimar masas entrantes/salientes.
- En modo rápido, `ThermoSolver` consume modelos empíricos de llenado (`EmpiricalFillModel`) para estimar `mass_in/mass_out` sin resolver 1D.
- En modo detallado, las condiciones de contorno y caudales provienen de `core.gasdynamics`, mientras que los resultados de presión/temperatura se devuelven a `core.gasdynamics` y `core.acoustics`.
- Configuración: parámetros de `CombustionModel` y estados iniciales se cargan desde JSON/YAML a través de `io.config` y se asignan a cada cilindro.

## core.valvetrain

### Clases principales

- **CamProfile**
  - Atributos: `angle_lift_table` (par tabla ángulo de cigüeñal → alzada), `duration_nominal`, `max_lift`, `lobe_separation_angle`, `centerline_timing` (IVC/EVC), metadatos de origen (CSV/JSON), parámetros de suavizado.
  - Métodos públicos: `lift_at(crank_angle)`, `derivative_at(crank_angle)`, `curtain_area(valve_diameter, crank_angle)`, `apply_advance(delta_deg)`, `apply_duration_scale(factor)`, `to_dict()`/`from_dict()` para import/export, `resample(resolution_deg)`.

- **Valve**
  - Atributos: `valve_id`, `type` (intake/exhaust), `diameter`, `seat_angle`, `cam_profile` (referencia a `CamProfile`), `stem_length`, `clearance`, `flow_coefficient_map` (tabla `lift → C_d` con interpolación lineal o spline), restricciones de movimiento (VVL/VVT).
  - Métodos públicos: `lift(crank_angle)`, `effective_area(crank_angle, discharge_coeff_model=None)`, `apply_global_phase(delta_deg)`, `modify_lift(factor)`.

- **ValvetrainAssembly**
  - Atributos: `banks` (mapeo banco → árbol de admisión/escape), `cylinder_valves` (mapeo cilindro → lista de `Valve`), `cam_advance_intake`, `cam_advance_exhaust`, parámetros de VVT/VVL por modo, referencias a sincronización con `Engine` (offsets por cilindro).
  - Métodos públicos: `assign_valve_to_cylinder(valve, cylinder_id)`, `intake_lift(cylinder_id, crank_angle)`, `exhaust_lift(cylinder_id, crank_angle)`, `intake_effective_area(cylinder_id, crank_angle)`, `exhaust_effective_area(cylinder_id, crank_angle)`, `apply_global_intake_advance(delta_deg)`, `apply_global_exhaust_advance(delta_deg)`, `export_profiles(format)`, `import_profiles(data)`, `generate_basic_profile(duration, max_lift, lsa, advance)`.

### Relaciones y uso por otros módulos

- `core.cycle` consulta `ValvetrainAssembly` para obtener alzadas y áreas efectivas en cada ángulo al calcular `mass_in/mass_out` y establecer condiciones de contorno hacia `core.gasdynamics` o modelos empíricos.
- `core.gasdynamics` usa las áreas efectivas y coeficientes de descarga como entrada a condiciones de contorno 1D (válvulas como conectores entre cilindro y conductos).
- `core.engine` sincroniza ángulos de referencia de cilindros con `ValvetrainAssembly` para considerar offsets de banco y orden de encendido en la evaluación de perfiles.
- `core.exploration` y `core.optimization` ajustan globalmente `cam_advance_intake/ cam_advance_exhaust`, duración y alzada vía métodos de `CamProfile`/`Valve` para estudios paramétricos.

## core.gasdynamics

### Entidades geométricas y de red

- **Pipe**: longitud, diámetro interno, rugosidad, número de segmentos, material/temperatura de pared simplificada.
- **Segment/Cell**: variables de estado (`pressure`, `temperature`, `density`, `velocity`, composición simple), posición axial, referencias a celdas vecinas.
- **Junction**: unión T/Y con continuidad de masa/energía, coeficientes de pérdida local.
- **Plenum**: volumen, presión y temperatura medias, intercambio de calor simplificado.
- **GasNetwork**: grafo de nodos (plenos, uniones, cilindros) y aristas (conductos); mantiene conectividad con cilindros y salida a atmósfera.
- **GasNetworkConfig**: representación construida desde `io.config` que incluye lista de conductos (con sus extremos), plenos, uniones y condiciones de contorno asociadas a cilindros y a la atmósfera.

### Dinámica y esquema numérico

- Esquema 1D explícito o semi-implícito basado en características simplificadas; control de paso por condición CFL (`dt ≤ CFL * dx / a`).
- `GasNetwork.step(time_step, engine_state, valve_states)`: avanza todas las celdas; actualiza variables por segmento, resuelve uniones/plenos y aplica condiciones de contorno.
- Condiciones de contorno: en válvulas (caudal según área efectiva y salto de presión), en plenos (balance de masa/energía), en salida (presión fija con coeficiente de reflexión ajustable).
- Consultas de salida: `pressure_history_at(node_id)`, `massflow_history_at(node_id)`, `export_signals()` para acústica.

### Relaciones

- Recibe geometría y acoplamiento cilindro–válvula desde `core.engine` y `core.valvetrain` vía `GasNetworkConfig`.
- Interactúa con `core.thermo.ThermoSolver` intercambiando masa/entalpía en cada paso de ángulo/tiempo.
- Proporciona señales de presión/caudal a `core.acoustics` y métricas a `io.results`.

## core.cycle

### Clase principal: EngineSimulator (o CycleRunner)

- Atributos: referencias a `Engine`, `ValvetrainAssembly`, `ThermoSolver`/`CombustionModel` por cilindro, `GasNetwork` (opcional), configuración de paso angular `delta_theta`, banderas de modo (`fast_mode` vs `detailed_mode`).
- Métodos públicos: `run_steady_state(rpm)`, `run_rpm_sweep(rpm_grid)`, `step_once()` (avanza un paso de ángulo), `get_instant_torque()`, `collect_metrics()`.
- Modos: modo rápido desacoplado de `core.gasdynamics` usando `EmpiricalFillModel`; modo detallado acoplado con `GasNetwork`.

### Salidas principales

- Par instantáneo por cilindro y total, IMEP/BMEP por cilindro y global, VE por cilindro y global.
- Trazas de presión vs ángulo, caudales en válvulas, señales de red 1D para acústica.

### Relaciones

- Orquesta el avance sincronizado de `ThermoSolver` y `GasNetwork` usando geometría de `Engine` y perfiles de `ValvetrainAssembly`.
- Alimenta a `core.performance` para integración de par/potencia y a `core.acoustics` para síntesis de sonido.

## core.performance

### Responsabilidad

- Integrar salidas de `core.cycle` (IMEP/BMEP instantáneos y par indicado) con rpm para obtener curvas de par y potencia, así como VE y BSFC si se dispone de consumo.

### Entidades y métodos

- **PerformanceAggregator**
  - Métodos públicos: `torque_from_imep(imep, displacement, mechanical_loss_model)`, `power_from_torque(torque, rpm)`, `aggregate_cycle_results(cycle_data) → {torque_curve, power_curve, ve_curve, imep_per_cyl, bmep_per_cyl}`.
  - Inputs: resultados discretos por ángulo/ciclo de `EngineSimulator`.
  - Outputs: curvas listadas y listas por cilindro que se entregan a `io.results` o contenedores de resultados.
- `mechanical_loss_model`: función u objeto con método `loss_torque(rpm, imep, engine) → torque_pérdidas_promedio` usado para convertir IMEP en BMEP/torque efectivo.

### Relaciones

- Consumido por `core.cycle` y `app.api` para generar `SimulationResult` y `SweepResult` listos para serialización.
- Reutilizado por `core.exploration` y `core.optimization` para evaluar métricas T_mean y P_mean.

## core.acoustics

### Funciones principales

- Convertir señales de presión vs ángulo a dominio temporal según rpm, remuestrear a frecuencia de audio (p.ej., 44.1 kHz), normalizar amplitud y evitar clipping.
- Calcular espectros vía FFT y generar datos listos para graficar.

### APIs

- `generate_exhaust_sound(simulation_result, audio_settings)`: produce señal de audio y metadatos a partir de la presión en cola de escape.
- `compute_spectrum(pressure_signal, sampling_rate)`: devuelve magnitud vs frecuencia.
- Salidas preparadas para `io.audio` y `io.results`.

### Relaciones

- Consume señales de `core.gasdynamics` o de modelos simplificados entregados por `core.cycle`.
- Devuelve series temporales y espectros para exportación o análisis.

## io.config

### Estructura esperada de configuración (JSON/YAML)

- Secciones: `engine`, `valvetrain`, `gasdynamics`, `combustion/thermo`, `operation` (rpm o `rpm_schedule`, condiciones ambientales), `simulation` (paso angular, modo rápido/detallado), `acoustics` (opciones de audio), `exploration`, `optimization`.

### Funciones públicas

- `load_config(path) → raw_config`.
- `validate_config(raw_config) → validated_config` (chequea tipos, rangos, presencia de secciones requeridas).
- `build_engine(validated_config) → Engine`, `build_valvetrain(validated_config) → ValvetrainAssembly`, `build_thermo_models(validated_config) → {CombustionModel, EmpiricalFillModel}`, `build_gas_network(validated_config) → GasNetworkConfig`, `build_simulation_setup(validated_config) → SimulationSetup` (contiene referencias a todos los objetos y opciones de simulación).

### SimulationSetup

- Estructura compuesta que agrupa todo lo necesario para lanzar una simulación.
- Atributos: `engine` (`Engine`), `valvetrain` (`ValvetrainAssembly`), `gas_network_config` (`GasNetworkConfig` o `None` en modo 0D), `combustion_models` (mapa `cylinder_id → CombustionModel`), `fill_model` (`EmpiricalFillModel` o `None` si no se usa modo rápido), `operation` (rpm fija o `rpm_schedule`, condiciones ambientales, rango de rpm para sweeps), `simulation_options` (p. ej. `delta_theta`, flags de `fast_mode`/`detailed_mode`, CFL, número máximo de ciclos), `acoustics_options` (frecuencia de muestreo, normalización), `output_options` (rutas, qué artefactos guardar).
- Métodos públicos: `to_dict()`, `from_dict()`, `summary()`.

### Relaciones

- Fuente única de creación de objetos de dominio para `app.cli` y `app.api`.
- Provee `GasNetworkConfig` que se transforma en instancias de `GasNetwork` en `core.gasdynamics`.

## io.results

### Funciones principales

- `save_rpm_curves(results, path)`: guarda curvas rpm–par–potencia–VE en CSV/JSON.
- `save_pressure_history(results, path)`: guarda señales de presión/caudal de interés.
- `save_spectrum(spectrum, path)`: almacena espectros de audio en CSV/JSON.
- `save_run_metadata(metadata, path)`: registra parámetros clave de la simulación.
- Opcional: helpers de graficado con Matplotlib (`plot_rpm_curves`, `plot_pressure_traces`).

### Relaciones

- Consume contenedores `SimulationResult`, `SweepResult`, `AudioBundle`, `ExplorationResult`, `OptimizationResult` para serialización.
- Usado por `app.cli` y suites de `core.validation` para generar reportes y artefactos.

## io.audio

### Responsabilidad

- Convertir series temporales de presión/sonido a archivos `.wav` con normalización y metadatos básicos.

### APIs

- `write_wav(signal, sampling_rate, path, metadata=None)`: exporta audio.
- `normalize_signal(signal, target_level)`: ajusta amplitud evitando clipping.
- `attach_metadata(signal, metadata_dict)`: almacena información de origen (rpm, modo, fecha, configuración) junto al archivo.

### Relaciones

- Consumido por `core.acoustics` y `app.cli` (`sound` command) para persistir audio.
- Puede ser invocado desde `io.results` como helper al guardar `AudioBundle`.

## app.cli

### Comandos conceptuales

- **sim-0d**
  - Parámetros: `--config path`, `--rpm value|schedule`, `--output dir`.
  - Resultado: `SimulationResult` en modo rápido (0D) y archivos CSV/JSON vía `io.results`.

- **sim-1d**
  - Parámetros: `--config path`, `--rpm value|schedule`, `--output dir`.
  - Resultado: `SimulationResult` en modo detallado (0D+1D) con señales de red.

- **rpm-sweep**
  - Parámetros: `--config path`, `--rpm-grid start:end:step` o lista, `--mode 0d|1d`, `--output dir`.
  - Resultado: `SweepResult` con curvas rpm–par–potencia–VE; gráficos opcionales.

- **sound**
  - Parámetros: `--config path`, `--rpm value`, `--mode 0d|1d`, `--audio-fs hz`, `--output dir`.
  - Resultado: archivo `.wav` y espectro en CSV/JSON generados desde la señal de escape.

- **explore**
  - Parámetros: `--config path`, `--exploration-config path`, `--mode 0d|1d`, `--rpm value|grid`, `--output dir`, `--max-samples n`.
  - Resultado: `ExplorationResult` con tabla de combinaciones y métricas; exportable vía `io.results`.

- **optimize**
  - Parámetros: `--config path`, `--optimization-config path`, `--rpm-range min max`, `--mode 0d|1d`, `--output dir`, `--method grid|random|hill`.
  - Resultado: `OptimizationResult` con configuración óptima y top N, curvas comparativas vs configuración base.

### Relaciones

- Cada comando invoca `io.config` para cargar/validar el setup, llama a `core.cycle` (modo 0D o 1D) y utiliza `io.results`/`io.audio` para persistir salidas.
- `explore` y `optimize` orquestan `core.exploration` y `core.optimization`, respectivamente, y reutilizan los resultados para construir reportes.

## app.api

### Funciones de alto nivel

- `run_simulation(config_obj, mode="0d"|"1d", rpm=value|schedule) → SimulationResult`
  - Entrada: estructura de configuración validada (`SimulationSetup`), selección de modo y rpm o mapa de rpm.
  - Salida: `SimulationResult` con par/potencia/VE/IMEP/BMEP, trazas por cilindro y de red según el modo.

- `run_rpm_sweep(config_obj, rpm_grid, mode="0d"|"1d") → SweepResult`
  - Entrada: setup más malla de rpm.
  - Salida: curvas rpm–par–potencia–VE y metadatos de cada punto.

- `generate_exhaust_sound(sim_result, audio_settings) → AudioBundle`
  - Entrada: `SimulationResult` que incluya presión en cola de escape; ajustes de muestreo/normalización.
  - Salida: señal de audio remuestreada, espectro y metadatos listos para exportar.
  - `audio_settings`: dict/struct mínimo con `sampling_rate`, `normalization_mode` y opciones básicas de filtro (p. ej. pasa-altas/pasa-bajas opcionales).

- `run_exploration(config_obj, exploration_settings) → ExplorationResult`
  - Entrada: setup base más definición de parámetros a barrer y muestreo.
  - Salida: tabla de combinaciones con métricas agregadas y mejores configuraciones.

- `run_optimization(config_obj, optimization_settings) → OptimizationResult`
  - Entrada: setup base más rangos/cotas de parámetros y pesos de objetivo.
  - Salida: configuración óptima p*, top N y comparación con base.

### Relaciones

- Las funciones instancian `core.cycle.EngineSimulator` con objetos de `io.config`, delegan en `core.exploration` o `core.optimization` cuando aplica, y usan `core.acoustics`/`core.performance` para sonido y métricas.
- `SimulationResult`, `SweepResult`, `ExplorationResult`, `OptimizationResult` y `AudioBundle` son contenedores serializables que `io.results` y `io.audio` pueden escribir.

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

## Tipos de resultados de alto nivel

### ResultBundle (base genérica)

- Contiene campos comunes: `metadata` (config, modo, rpm/rpm_grid), `torque_curve`, `power_curve`, `ve_curve`, referencias a trazas crudas (presiones, caudales), y utilidades de serialización.

### SimulationResult

- Especialización para una rpm o `rpm_schedule`; incluye `torque`, `power`, `imep_per_cyl`, `bmep_per_cyl`, `pressure_traces`, `network_signals`, `ve_per_cyl`, y puede envolver un `ResultBundle` como contenedor interno o exponer la misma interfaz.

### SweepResult

- Curvas discretas en malla de rpm: `torque_curve`, `power_curve`, `ve_curve`, `imep_map`, `bmep_map`, `points_metadata`; puede componerse de múltiples `SimulationResult` y exponer la interfaz de `ResultBundle`.

### AudioBundle

- Señal temporal de audio, `sampling_rate`, espectro (`freq`, `magnitude`), metadatos (rpm, modo, origen de señal). Pensado para ser serializable por `io.audio`/`io.results`.

### ExplorationResult

- Tabla de combinaciones evaluadas con columnas de parámetros y métricas agregadas (`T_mean`, `P_mean`, `VE_mean`, objetivo J si aplica); incluye `top_n` ordenado y referencia al setup base. Serializa vía `io.results` y mantiene compatibilidad con `ResultBundle` para compartir metadatos.

### OptimizationResult

- Configuración óptima `p*`, métricas (`T_mean`, `P_mean`, `J`), comparativa con la configuración base (mejora porcentual), lista de las N mejores configuraciones, y opcionalmente el historial de búsqueda. Puede envolver internamente un `ExplorationResult` si el método incluye fase exploratoria.
