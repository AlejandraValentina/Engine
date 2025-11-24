"""
Simple Tkinter GUI to run 0D RPM sweeps using the core engine simulator.
"""

from __future__ import annotations

import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from icengine_sim.core.cycle import EngineSimulator
from icengine_sim.core.engine import Engine
from icengine_sim.core.performance import PerformanceAggregator
from icengine_sim.core.thermo import CombustionModel, CylinderState, EmpiricalFillModel, ThermoSolver
from icengine_sim.core.valvetrain import CamProfile, Valve, ValvetrainAssembly


AMBIENT_PRESSURE = 101_325.0
AMBIENT_TEMPERATURE = 300.0


class SimpleFillModel(EmpiricalFillModel):
    """Empirical fill model assuming volumetric efficiency near unity."""

    def __init__(self, ve_target: float = 0.95) -> None:
        self.ve_target = ve_target

    def estimate_mass_flow(self, rpm: float, cylinder, valvetrain_state, ambient_conditions):
        del rpm, valvetrain_state
        rho_amb = ambient_conditions["pressure"] / (287.0 * ambient_conditions["temperature"])
        mass_in_cycle = self.ve_target * rho_amb * cylinder.displacement()
        return {"mass_in_cycle": mass_in_cycle, "residual_fraction": 0.0}


def build_basic_valvetrain(n_cylinders: int, max_lift: float = 0.009, duration: float = 240.0) -> ValvetrainAssembly:
    """Creates a simple symmetric valvetrain with identical intake/exhaust profiles."""

    assembly = ValvetrainAssembly()
    intake_profile = CamProfile(
        angle_lift_table=np.maximum(
            0.0,
            max_lift
            * np.sin(
                np.clip((np.linspace(0, 720, 361) - 360 + duration / 2) / duration * np.pi, 0, np.pi)
            ),
        ),
        duration_nominal=duration,
        max_lift=max_lift,
        lobe_separation_angle=110.0,
        centerline_timing=360.0,
    )
    exhaust_profile = CamProfile(
        angle_lift_table=np.maximum(
            0.0,
            max_lift
            * np.sin(
                np.clip((np.linspace(0, 720, 361) - 180 + duration / 2) / duration * np.pi, 0, np.pi)
            ),
        ),
        duration_nominal=duration,
        max_lift=max_lift,
        lobe_separation_angle=110.0,
        centerline_timing=180.0,
    )
    for cid in range(1, n_cylinders + 1):
        intake_valve = Valve(
            valve_id=cid * 2 - 1,
            type="intake",
            diameter=0.03,
            seat_angle=45.0,
            cam_profile=intake_profile,
        )
        exhaust_valve = Valve(
            valve_id=cid * 2,
            type="exhaust",
            diameter=0.027,
            seat_angle=45.0,
            cam_profile=exhaust_profile,
        )
        assembly.assign_valve_to_cylinder(intake_valve, cid)
        assembly.assign_valve_to_cylinder(exhaust_valve, cid)
    return assembly


def build_basic_engine(
    n_cylinders: int, displacement_l: float, layout: str, v_angle: float | None, compression_ratio: float
) -> Engine:
    """Creates an Engine instance with derived bore/stroke for the given displacement."""

    displacement_m3 = displacement_l / 1000.0
    engine = Engine.build_standard(layout=layout, displacement=displacement_m3, n_cylinders=n_cylinders, v_angle=v_angle)
    for bank in engine.banks:
        for cyl in bank.cylinders:
            cyl.compression_ratio = compression_ratio
    return engine


def build_thermo_solvers(engine: Engine, fill_model: EmpiricalFillModel, comb_model: CombustionModel, delta_theta: float) -> dict:
    """Initializes ThermoSolver objects for each cylinder."""

    solvers = {}
    for cyl in engine.iterate_cylinders_in_firing_order():
        volume_init = cyl.instant_volume(180.0)
        mass_init = AMBIENT_PRESSURE * volume_init / (287.0 * AMBIENT_TEMPERATURE)
        state = CylinderState(
            pressure=AMBIENT_PRESSURE,
            temperature=AMBIENT_TEMPERATURE,
            mass=mass_init,
            fuel_mass=mass_init * 0.015,
            volume=volume_init,
        )
        solver = ThermoSolver(
            cylinder=cyl,
            state=state,
            combustion_model=comb_model,
            fast_mode=True,
            detailed_mode=False,
            delta_theta=delta_theta,
        )
        solver.enable_fast_fill(fill_model)
        solvers[cyl.cylinder_id] = solver
    return solvers


@dataclass
class GuiState:
    rpm: np.ndarray
    torque: np.ndarray
    power_hp: np.ndarray


def run_rpm_sweep(settings: dict) -> GuiState:
    """Executes an RPM sweep using the core 0D simulator."""

    engine = build_basic_engine(
        n_cylinders=settings["n_cyl"],
        displacement_l=settings["disp_l"],
        layout=settings["layout"],
        v_angle=settings["v_angle"],
        compression_ratio=settings["cr"],
    )
    valvetrain = build_basic_valvetrain(engine.n_cylinders)
    comb_model = CombustionModel(theta_start=360.0, duration=40.0, m_shape=5.0, n_shape=2.0)
    fill_model = SimpleFillModel(ve_target=0.95)
    thermo_solvers = build_thermo_solvers(engine, fill_model, comb_model, delta_theta=1.0)
    simulator = EngineSimulator(
        engine=engine,
        valvetrain=valvetrain,
        thermo_solvers=thermo_solvers,
        delta_theta=1.0,
        fast_mode=True,
        detailed_mode=False,
        aggregator=PerformanceAggregator(),
    )
    rpm_values = np.arange(settings["rpm_start"], settings["rpm_end"] + 1e-6, settings["rpm_step"])
    torques = []
    powers_hp = []
    for rpm in rpm_values:
        result = simulator.run_steady_state(float(rpm))
        torques.append(result.torque)
        powers_hp.append(result.power / 745.7)
    return GuiState(rpm=rpm_values, torque=np.array(torques), power_hp=np.array(powers_hp))


def main() -> None:
    """Launches the Tkinter GUI for the 0D engine simulator."""

    root = tk.Tk()
    root.title("ICE 0D Engine Simulator")

    input_frame = ttk.Frame(root, padding=10)
    input_frame.pack(side=tk.TOP, fill=tk.X)

    # Engine inputs
    entries = {}
    defaults = {
        "n_cyl": 12,
        "disp_l": 1.2,
        "layout": "V",
        "v_angle": 65.0,
        "cr": 13.0,
        "rpm_start": 6000,
        "rpm_end": 18000,
        "rpm_step": 1000,
    }

    def add_entry(row: int, label: str, key: str):
        ttk.Label(input_frame, text=label).grid(row=row, column=0, sticky=tk.W)
        if key == "layout":
            combo = ttk.Combobox(input_frame, values=["V", "L"], state="readonly")
            combo.set(defaults[key])
            combo.grid(row=row, column=1, padx=5, pady=2, sticky=tk.W)
            entries[key] = combo
        else:
            entry = ttk.Entry(input_frame)
            entry.insert(0, str(defaults[key]))
            entry.grid(row=row, column=1, padx=5, pady=2, sticky=tk.W)
            entries[key] = entry

    add_entry(0, "Number of cylinders", "n_cyl")
    add_entry(1, "Displacement [L]", "disp_l")
    add_entry(2, "Layout", "layout")
    add_entry(3, "V angle [deg]", "v_angle")
    add_entry(4, "Compression ratio", "cr")
    add_entry(5, "RPM start", "rpm_start")
    add_entry(6, "RPM end", "rpm_end")
    add_entry(7, "RPM step", "rpm_step")

    button_frame = ttk.Frame(root, padding=10)
    button_frame.pack(side=tk.TOP, fill=tk.X)

    status_var = tk.StringVar(value="Ready")
    status_label = ttk.Label(root, textvariable=status_var, padding=5)
    status_label.pack(side=tk.BOTTOM, fill=tk.X)

    fig, ax_torque = plt.subplots(figsize=(6, 4))
    ax_power = ax_torque.twinx()
    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    result_var = tk.StringVar(value="")
    result_label = ttk.Label(root, textvariable=result_var, padding=5)
    result_label.pack(side=tk.TOP, fill=tk.X)

    def parse_inputs() -> dict:
        try:
            n_cyl = int(entries["n_cyl"].get())
            disp_l = float(entries["disp_l"].get())
            layout = entries["layout"].get()
            v_angle = float(entries["v_angle"].get()) if layout == "V" else None
            cr = float(entries["cr"].get())
            rpm_start = float(entries["rpm_start"].get())
            rpm_end = float(entries["rpm_end"].get())
            rpm_step = float(entries["rpm_step"].get())
        except ValueError as exc:
            messagebox.showerror("Input error", f"Invalid numeric value: {exc}")
            return {}

        if rpm_step <= 0 or rpm_end <= rpm_start:
            messagebox.showerror("Input error", "RPM step must be positive and end > start.")
            return {}
        return {
            "n_cyl": n_cyl,
            "disp_l": disp_l,
            "layout": layout,
            "v_angle": v_angle,
            "cr": cr,
            "rpm_start": rpm_start,
            "rpm_end": rpm_end,
            "rpm_step": rpm_step,
        }

    def run_sweep() -> None:
        settings = parse_inputs()
        if not settings:
            return
        status_var.set("Running sweep...")
        root.update_idletasks()
        try:
            state = run_rpm_sweep(settings)
        except Exception as exc:  # pragma: no cover - GUI error handling
            messagebox.showerror("Simulation error", str(exc))
            status_var.set("Error")
            return

        ax_torque.clear()
        ax_power.clear()
        ax_torque.plot(state.rpm, state.torque, label="Torque [Nm]", color="tab:blue")
        ax_power.plot(state.rpm, state.power_hp, label="Power [HP]", color="tab:red")
        ax_torque.set_xlabel("RPM")
        ax_torque.set_ylabel("Torque [Nm]", color="tab:blue")
        ax_power.set_ylabel("Power [HP]", color="tab:red")
        ax_torque.grid(True, which="both", linestyle="--", alpha=0.5)
        fig.legend(loc="upper right")
        canvas.draw()

        max_torque_idx = int(np.argmax(state.torque)) if len(state.torque) > 0 else 0
        max_power_idx = int(np.argmax(state.power_hp)) if len(state.power_hp) > 0 else 0
        max_torque_info = (
            state.rpm[max_torque_idx],
            state.torque[max_torque_idx],
        )
        max_power_info = (
            state.rpm[max_power_idx],
            state.power_hp[max_power_idx],
        )
        result_var.set(
            f"Max torque: {max_torque_info[1]:.1f} Nm @ {max_torque_info[0]:.0f} rpm | "
            f"Max power: {max_power_info[1]:.1f} HP @ {max_power_info[0]:.0f} rpm"
        )
        status_var.set("Done")

    ttk.Button(button_frame, text="Run RPM Sweep", command=run_sweep).pack(side=tk.LEFT, padx=5)

    root.mainloop()


if __name__ == "__main__":
    main()
