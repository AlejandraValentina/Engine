"""
Thermodynamic state and combustion models for 0D cylinder simulation.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional
import math
import numpy as np


GAS_CONSTANT_AIR = 287.0  # J/(kg*K)
DEFAULT_CP = 1005.0  # J/(kg*K)
DEFAULT_GAMMA = 1.4


@dataclass
class CylinderState:
    """Thermodynamic state inside a cylinder."""

    pressure: float  # Pa
    temperature: float  # K
    mass: float  # kg
    burn_fraction: float = 0.0
    fuel_mass: float = 0.0
    equivalence_ratio: float = 1.0
    gamma_effective: float = DEFAULT_GAMMA
    cp: float = DEFAULT_CP
    cv: float = DEFAULT_CP / DEFAULT_GAMMA
    exhaust_residual_fraction: float = 0.0
    mode: str = "fast_fill_model"
    volume: float = 0.0

    def clone(self) -> "CylinderState":
        return CylinderState(**self.__dict__)

    def update_properties(self, thermo_table: Optional[Dict[str, float]] = None) -> None:
        """Updates gamma, cp, and cv based on current state or provided table."""
        if thermo_table:
            self.cp = thermo_table.get("cp", self.cp)
            self.cv = thermo_table.get("cv", self.cv)
            self.gamma_effective = thermo_table.get("gamma", self.gamma_effective)
        else:
            self.cp = DEFAULT_CP
            self.cv = self.cp - GAS_CONSTANT_AIR
            self.gamma_effective = self.cp / self.cv

    def reset_burn(self) -> None:
        self.burn_fraction = 0.0

    @property
    def density(self) -> float:
        return self.mass / max(1e-12, self.specific_volume)

    @property
    def specific_volume(self) -> float:
        return self.volume / max(self.mass, 1e-12)

    @property
    def internal_energy(self) -> float:
        return self.mass * self.cv * self.temperature


@dataclass
class CombustionModel:
    """Wiebe-based combustion phasing model."""

    theta_start: float
    duration: float
    m_shape: float
    n_shape: float
    heat_release_lhv: float = 43e6  # J/kg fuel
    mass_fraction_burned_curve: Optional[np.ndarray] = None

    def mass_fraction_burned(self, crank_angle: float) -> float:
        if self.mass_fraction_burned_curve is not None:
            angles = np.linspace(self.theta_start, self.theta_start + self.duration, len(self.mass_fraction_burned_curve))
            return float(np.interp(crank_angle, angles, self.mass_fraction_burned_curve, left=0.0, right=1.0))
        x = max(0.0, min(1.0, (crank_angle - self.theta_start) / max(self.duration, 1e-6)))
        return 1.0 - math.exp(-self.m_shape * (x ** (self.n_shape + 1))) if x > 0 else 0.0

    def heat_release_rate(self, crank_angle: float) -> float:
        dtheta = 0.1
        f1 = self.mass_fraction_burned(crank_angle)
        f2 = self.mass_fraction_burned(crank_angle + dtheta)
        return (f2 - f1) * self.heat_release_lhv / dtheta

    def configure(self, params_dict: Dict) -> None:
        self.theta_start = params_dict.get("theta_start", self.theta_start)
        self.duration = params_dict.get("duration", self.duration)
        self.m_shape = params_dict.get("m_shape", self.m_shape)
        self.n_shape = params_dict.get("n_shape", self.n_shape)

    def shift(self, theta_offset: float) -> None:
        self.theta_start += theta_offset

    def scale_duration(self, factor: float) -> None:
        self.duration *= factor


class EmpiricalFillModel:
    """Interface for empirical filling models used in fast mode."""

    def estimate_mass_flow(self, rpm: float, cylinder, valvetrain_state, ambient_conditions: Dict) -> Dict[str, float]:
        raise NotImplementedError("EmpiricalFillModel.estimate_mass_flow must be implemented in subclasses")


@dataclass
class ThermoSolver:
    """0D thermodynamic solver for a single cylinder."""

    cylinder: any
    state: CylinderState
    combustion_model: CombustionModel
    fast_mode: bool = True
    detailed_mode: bool = False
    delta_theta: float = 1.0
    mechanical_loss_model: Optional[object] = None
    empirical_model: Optional[EmpiricalFillModel] = None
    rpm: float = 1000.0
    history_pressure: list = field(default_factory=list)
    history_volume: list = field(default_factory=list)

    def set_boundary_conditions(self, **kwargs) -> None:
        del kwargs

    def enable_fast_fill(self, empirical_model: EmpiricalFillModel) -> None:
        self.empirical_model = empirical_model

    def step(
        self,
        crank_angle: float,
        delta_theta: float,
        mass_in: float = 0.0,
        mass_out: float = 0.0,
        enthalpy_in: float = 0.0,
        enthalpy_out: float = 0.0,
    ) -> CylinderState:
        """Advances the thermodynamic state by delta_theta degrees."""
        volume_start = self.cylinder.instant_volume(crank_angle)
        volume_end = self.cylinder.instant_volume(crank_angle + delta_theta)
        dV = volume_end - volume_start

        self.state.update_properties()
        cp = self.state.cp
        cv = self.state.cv
        R = cp - cv

        dt = delta_theta / 360.0 * 60.0 / max(self.rpm, 1e-6)

        f_old = self.state.burn_fraction
        f_new = min(1.0, self.combustion_model.mass_fraction_burned(crank_angle + 0.5 * delta_theta))
        delta_f = max(0.0, f_new - f_old)
        q_in = delta_f * self.state.fuel_mass * self.combustion_model.heat_release_lhv

        m_new = max(1e-9, self.state.mass + mass_in - mass_out)
        u_old = self.state.internal_energy
        p_old = self.state.pressure

        u_new = u_old + q_in + enthalpy_in - enthalpy_out - p_old * dV
        T_new = u_new / (m_new * cv)
        p_new = m_new * R * T_new / max(volume_end, 1e-12)

        self.state.mass = m_new
        self.state.temperature = T_new
        self.state.pressure = p_new
        self.state.burn_fraction = f_new
        self.state.volume = volume_end

        self.history_pressure.append(p_new)
        self.history_volume.append(volume_end)
        return self.state

    def compute_pressure_trace(self, crank_window) -> np.ndarray:
        del crank_window
        return np.array(self.history_pressure)

    def indicated_work(self) -> float:
        if len(self.history_pressure) < 2:
            return 0.0
        p = np.array(self.history_pressure)
        v = np.array(self.history_volume)
        return float(np.trapz(p, v))

    def imep(self) -> float:
        work = self.indicated_work()
        swept = self.cylinder.displacement()
        return work / swept if swept > 0 else 0.0

    def bmep(self, mechanical_loss_model: Optional[object] = None) -> float:
        imep_value = self.imep()
        loss_model = mechanical_loss_model or self.mechanical_loss_model
        if loss_model is None:
            return imep_value
        torque_loss = loss_model.loss_torque(self.rpm, imep_value, self.cylinder)
        swept = self.cylinder.displacement()
        return imep_value - torque_loss / swept
