"""
Engine cycle coordination for 0D simulations.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence
import numpy as np

from .engine import Engine
from .thermo import ThermoSolver, EmpiricalFillModel
from .valvetrain import ValvetrainAssembly
from .performance import PerformanceAggregator, SimulationResult, SweepResult


@dataclass
class EngineSimulator:
    """Coordinates ThermoSolver instances to run engine cycles in 0D mode."""

    engine: Engine
    valvetrain: ValvetrainAssembly
    thermo_solvers: Dict[int, ThermoSolver]
    delta_theta: float = 1.0
    fast_mode: bool = True
    detailed_mode: bool = False
    current_angle: float = 0.0
    aggregator: PerformanceAggregator = field(default_factory=PerformanceAggregator)

    def step_once(self) -> None:
        """Advances a single crank-angle step across all cylinders."""
        raise NotImplementedError("Per-step advance requires persistent state tracking; use run_steady_state instead.")

    def _prepare_mass_distribution(
        self, rpm: float, cyl_id: int, angle_grid: np.ndarray
    ) -> Dict[str, np.ndarray]:
        solver = self.thermo_solvers[cyl_id]
        solver.rpm = rpm
        intake_mask = np.array([self.valvetrain.intake_effective_area(cyl_id, a) > 0 for a in angle_grid])
        exhaust_mask = np.array([self.valvetrain.exhaust_effective_area(cyl_id, a) > 0 for a in angle_grid])
        mass_in_cycle = 0.0
        if solver.empirical_model:
            estimate = solver.empirical_model.estimate_mass_flow(rpm, solver.cylinder, None, {"pressure": 101325, "temperature": 300})
            mass_in_cycle = estimate.get("mass_in_cycle", 0.0)
        intake_steps = max(1, int(np.sum(intake_mask)))
        exhaust_steps = max(1, int(np.sum(exhaust_mask)))
        mass_in_profile = np.zeros_like(angle_grid)
        mass_out_profile = np.zeros_like(angle_grid)
        mass_in_profile[intake_mask] = mass_in_cycle / intake_steps
        mass_out_profile[exhaust_mask] = solver.state.mass * 0.02 / exhaust_steps
        return {
            "mass_in": mass_in_profile,
            "mass_out": mass_out_profile,
            "intake_mask": intake_mask,
            "exhaust_mask": exhaust_mask,
            "mass_in_cycle": mass_in_cycle,
        }

    def run_steady_state(self, rpm: float) -> SimulationResult:
        """Simulates one complete 720-degree cycle at fixed rpm in 0D mode."""
        angle_grid = np.arange(0, 720, self.delta_theta)
        pressure_traces: Dict[int, List[float]] = {}
        ve_per_cyl: List[float] = []
        imep_per_cyl: List[float] = []
        displacement_per_cyl: List[float] = []

        for cyl in self.engine.iterate_cylinders_in_firing_order():
            profiles = self._prepare_mass_distribution(rpm, cyl.cylinder_id, angle_grid)
            solver = self.thermo_solvers[cyl.cylinder_id]
            solver.history_pressure.clear()
            solver.history_volume.clear()
            mass_in_total = 0.0
            for theta in angle_grid:
                cyl_angle = (theta + self.engine.firing_angle(cyl.cylinder_id)) % 720.0
                idx = int(theta / self.delta_theta)
                mass_in = profiles["mass_in"][idx]
                mass_out = profiles["mass_out"][idx]
                enthalpy_in = mass_in * solver.state.cp * 300.0
                enthalpy_out = mass_out * solver.state.cp * solver.state.temperature
                solver.step(cyl_angle, self.delta_theta, mass_in=mass_in, mass_out=mass_out, enthalpy_in=enthalpy_in, enthalpy_out=enthalpy_out)
                mass_in_total += mass_in
            pressure_traces[cyl.cylinder_id] = list(solver.history_pressure)
            displacement_per_cyl.append(cyl.displacement())
            imep_per_cyl.append(solver.imep())
            rho_amb = 101325 / (287.0 * 300.0)
            ve_per_cyl.append(mass_in_total / (rho_amb * cyl.displacement()) if cyl.displacement() > 0 else 0.0)

        perf = self.aggregator.aggregate_cycle_results(imep_per_cyl, displacement_per_cyl, rpm, ve_per_cyl)
        result = SimulationResult(
            metadata={"rpm": rpm, "mode": "0d"},
            torque_curve=perf["torque_curve"],
            power_curve=perf["power_curve"],
            ve_curve=perf["ve_curve"],
            torque=perf["torque"],
            power=perf["power"],
            imep_per_cyl=perf["imep_per_cyl"],
            ve_per_cyl=perf["ve_per_cyl"],
            pressure_traces=pressure_traces,
            network_signals={},
        )
        return result

    def run_rpm_sweep(self, rpm_grid: Sequence[float]) -> SweepResult:
        torques = []
        powers = []
        ve_points = []
        points_metadata: List[Dict] = []
        imep_map: Dict[str, List[float]] = {"imep": []}
        for rpm in rpm_grid:
            sim_res = self.run_steady_state(rpm)
            torques.append(sim_res.torque)
            powers.append(sim_res.power)
            ve_points.append(np.mean(sim_res.ve_per_cyl or [0.0]))
            imep_map["imep"].append(float(np.mean(sim_res.imep_per_cyl or [0.0])))
            points_metadata.append({"rpm": rpm})
        return SweepResult(
            metadata={"rpm_grid": list(rpm_grid), "mode": "0d"},
            torque_curve=np.array(torques),
            power_curve=np.array(powers),
            ve_curve=np.array(ve_points),
            imep_map=imep_map,
            points_metadata=points_metadata,
        )

    def get_instant_torque(self) -> float:
        """Instantaneous torque is not computed in fast 0D mode."""
        raise NotImplementedError("Instantaneous torque requires cylinder pressure phasing.")

    def collect_metrics(self) -> Dict:
        """Collects available metrics from the last run (not persisted yet)."""
        return {}


__all__ = ["EngineSimulator"]
