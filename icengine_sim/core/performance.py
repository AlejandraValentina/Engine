"""
Performance aggregation utilities for torque and power metrics.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import numpy as np


@dataclass
class ResultBundle:
    """Generic container for simulation outputs."""

    metadata: Dict
    torque_curve: Optional[np.ndarray] = None
    power_curve: Optional[np.ndarray] = None
    ve_curve: Optional[np.ndarray] = None
    raw_traces: Dict = field(default_factory=dict)

    def to_dict(self) -> Dict:
        return {
            "metadata": self.metadata,
            "torque_curve": None if self.torque_curve is None else self.torque_curve.tolist(),
            "power_curve": None if self.power_curve is None else self.power_curve.tolist(),
            "ve_curve": None if self.ve_curve is None else self.ve_curve.tolist(),
            "raw_traces": self.raw_traces,
        }


@dataclass
class SimulationResult(ResultBundle):
    torque: float = 0.0
    power: float = 0.0
    imep_per_cyl: Optional[List[float]] = None
    bmep_per_cyl: Optional[List[float]] = None
    pressure_traces: Dict[int, List[float]] = field(default_factory=dict)
    network_signals: Dict = field(default_factory=dict)
    ve_per_cyl: Optional[List[float]] = None

    def to_dict(self) -> Dict:
        base = super().to_dict()
        base.update(
            {
                "torque": self.torque,
                "power": self.power,
                "imep_per_cyl": self.imep_per_cyl,
                "bmep_per_cyl": self.bmep_per_cyl,
                "pressure_traces": self.pressure_traces,
                "network_signals": self.network_signals,
                "ve_per_cyl": self.ve_per_cyl,
            }
        )
        return base


@dataclass
class SweepResult(ResultBundle):
    imep_map: Optional[Dict[str, List[float]]] = None
    bmep_map: Optional[Dict[str, List[float]]] = None
    points_metadata: Optional[List[Dict]] = None

    def to_dict(self) -> Dict:
        base = super().to_dict()
        base.update(
            {
                "imep_map": self.imep_map,
                "bmep_map": self.bmep_map,
                "points_metadata": self.points_metadata,
            }
        )
        return base


@dataclass
class AudioBundle(ResultBundle):
    sampling_rate: Optional[float] = None
    spectrum: Optional[Dict[str, List[float]]] = None

    def to_dict(self) -> Dict:
        base = super().to_dict()
        base.update({"sampling_rate": self.sampling_rate, "spectrum": self.spectrum})
        return base


@dataclass
class ExplorationResult(ResultBundle):
    table: Optional[List[Dict]] = None
    top_n: Optional[List[Dict]] = None

    def to_dict(self) -> Dict:
        base = super().to_dict()
        base.update({"table": self.table, "top_n": self.top_n})
        return base


@dataclass
class OptimizationResult(ResultBundle):
    optimum: Optional[Dict] = None
    candidates: Optional[List[Dict]] = None

    def to_dict(self) -> Dict:
        base = super().to_dict()
        base.update({"optimum": self.optimum, "candidates": self.candidates})
        return base


class PerformanceAggregator:
    """Converts IMEP/BMEP and cycle data into torque and power curves."""

    def torque_from_imep(self, imep: float, displacement: float, mechanical_loss_model=None) -> float:
        bmep = imep
        if mechanical_loss_model is not None:
            bmep -= mechanical_loss_model.loss_torque(0, imep, None) / max(displacement, 1e-12)
        return bmep * displacement / (4.0 * np.pi)

    def power_from_torque(self, torque: float, rpm: float) -> float:
        return torque * (2.0 * np.pi * rpm / 60.0)

    def aggregate_cycle_results(
        self,
        imep_per_cyl: List[float],
        displacement_per_cyl: List[float],
        rpm: float,
        ve_per_cyl: Optional[List[float]] = None,
    ) -> Dict:
        total_displacement = float(np.sum(displacement_per_cyl))
        mean_imep = float(np.mean(imep_per_cyl))
        torque = self.torque_from_imep(mean_imep, total_displacement)
        power = self.power_from_torque(torque, rpm)
        ve_curve = np.array(ve_per_cyl) if ve_per_cyl is not None else None
        return {
            "torque": torque,
            "power": power,
            "imep_per_cyl": imep_per_cyl,
            "ve_per_cyl": ve_per_cyl,
            "torque_curve": np.array([torque]),
            "power_curve": np.array([power]),
            "ve_curve": ve_curve,
        }


__all__ = [
    "PerformanceAggregator",
    "ResultBundle",
    "SimulationResult",
    "SweepResult",
    "AudioBundle",
    "ExplorationResult",
    "OptimizationResult",
]
