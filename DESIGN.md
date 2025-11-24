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
