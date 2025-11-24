"""
Valvetrain and cam profile models.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence
import numpy as np
import math


@dataclass
class CamProfile:
    """Tabulated cam profile with utilities for lift and area queries."""

    angle_lift_table: Sequence[float]
    duration_nominal: float
    max_lift: float
    lobe_separation_angle: float
    centerline_timing: float
    metadata: Dict = field(default_factory=dict)

    def lift_at(self, crank_angle: float) -> float:
        angles = np.linspace(0, 720, len(self.angle_lift_table))
        return float(np.interp(crank_angle % 720.0, angles, self.angle_lift_table))

    def derivative_at(self, crank_angle: float) -> float:
        dtheta = 0.5
        l1 = self.lift_at(crank_angle)
        l2 = self.lift_at(crank_angle + dtheta)
        return (l2 - l1) / dtheta

    def curtain_area(self, valve_diameter: float, crank_angle: float) -> float:
        lift = self.lift_at(crank_angle)
        return math.pi * valve_diameter * lift

    def apply_advance(self, delta_deg: float) -> None:
        angles = np.linspace(0, 720, len(self.angle_lift_table))
        shifted_angles = (angles + delta_deg) % 720.0
        self.angle_lift_table = np.interp(angles, shifted_angles, self.angle_lift_table)
        self.centerline_timing = (self.centerline_timing + delta_deg) % 720.0

    def apply_duration_scale(self, factor: float) -> None:
        new_len = max(2, int(len(self.angle_lift_table) * factor))
        angles = np.linspace(0, 720, len(self.angle_lift_table))
        new_angles = np.linspace(0, 720, new_len)
        self.angle_lift_table = np.interp(new_angles, angles, self.angle_lift_table)
        self.duration_nominal *= factor

    def to_dict(self) -> Dict:
        return {
            "angle_lift_table": list(self.angle_lift_table),
            "duration_nominal": self.duration_nominal,
            "max_lift": self.max_lift,
            "lobe_separation_angle": self.lobe_separation_angle,
            "centerline_timing": self.centerline_timing,
            "metadata": self.metadata,
        }

    @classmethod
    def from_dict(cls, data: Dict) -> "CamProfile":
        return cls(
            angle_lift_table=data["angle_lift_table"],
            duration_nominal=data["duration_nominal"],
            max_lift=data["max_lift"],
            lobe_separation_angle=data.get("lobe_separation_angle", 0.0),
            centerline_timing=data.get("centerline_timing", 0.0),
            metadata=data.get("metadata", {}),
        )

    def resample(self, resolution_deg: float) -> None:
        angles = np.arange(0, 720 + resolution_deg, resolution_deg)
        self.angle_lift_table = np.interp(angles, np.linspace(0, 720, len(self.angle_lift_table)), self.angle_lift_table)


@dataclass
class Valve:
    """Valve geometry and discharge behavior linked to a cam profile."""

    valve_id: int
    type: str
    diameter: float
    seat_angle: float
    cam_profile: CamProfile
    stem_length: float = 0.0
    clearance: float = 0.0
    flow_coefficient_map: Optional[Dict[float, float]] = None

    def lift(self, crank_angle: float) -> float:
        return self.cam_profile.lift_at(crank_angle)

    def discharge_coefficient(self, lift: float) -> float:
        if not self.flow_coefficient_map:
            return 1.0
        lifts = np.array(sorted(self.flow_coefficient_map.keys()))
        cds = np.array([self.flow_coefficient_map[l] for l in lifts])
        return float(np.interp(lift, lifts, cds, left=cds[0], right=cds[-1]))

    def effective_area(self, crank_angle: float, discharge_coeff_model=None) -> float:
        lift_value = self.lift(crank_angle)
        cd = discharge_coeff_model(lift_value) if discharge_coeff_model else self.discharge_coefficient(lift_value)
        return cd * self.cam_profile.curtain_area(self.diameter, crank_angle)

    def apply_global_phase(self, delta_deg: float) -> None:
        self.cam_profile.apply_advance(delta_deg)

    def modify_lift(self, factor: float) -> None:
        self.cam_profile.angle_lift_table = [min(self.cam_profile.max_lift * factor, l * factor) for l in self.cam_profile.angle_lift_table]
        self.cam_profile.max_lift *= factor


@dataclass
class ValvetrainAssembly:
    """Associates valves to cylinders and manages global phasing."""

    banks: Dict[int, Dict[str, CamProfile]] = field(default_factory=dict)
    cylinder_valves: Dict[int, List[Valve]] = field(default_factory=dict)
    cam_advance_intake: float = 0.0
    cam_advance_exhaust: float = 0.0

    def assign_valve_to_cylinder(self, valve: Valve, cylinder_id: int) -> None:
        self.cylinder_valves.setdefault(cylinder_id, []).append(valve)

    def _get_valves(self, cylinder_id: int, valve_type: str) -> List[Valve]:
        return [v for v in self.cylinder_valves.get(cylinder_id, []) if v.type == valve_type]

    def intake_lift(self, cylinder_id: int, crank_angle: float) -> float:
        valves = self._get_valves(cylinder_id, "intake")
        return sum(v.lift(crank_angle + self.cam_advance_intake) for v in valves)

    def exhaust_lift(self, cylinder_id: int, crank_angle: float) -> float:
        valves = self._get_valves(cylinder_id, "exhaust")
        return sum(v.lift(crank_angle + self.cam_advance_exhaust) for v in valves)

    def intake_effective_area(self, cylinder_id: int, crank_angle: float) -> float:
        valves = self._get_valves(cylinder_id, "intake")
        return sum(v.effective_area(crank_angle + self.cam_advance_intake) for v in valves)

    def exhaust_effective_area(self, cylinder_id: int, crank_angle: float) -> float:
        valves = self._get_valves(cylinder_id, "exhaust")
        return sum(v.effective_area(crank_angle + self.cam_advance_exhaust) for v in valves)

    def apply_global_intake_advance(self, delta_deg: float) -> None:
        self.cam_advance_intake += delta_deg
        for valves in self.cylinder_valves.values():
            for valve in valves:
                if valve.type == "intake":
                    valve.apply_global_phase(delta_deg)

    def apply_global_exhaust_advance(self, delta_deg: float) -> None:
        self.cam_advance_exhaust += delta_deg
        for valves in self.cylinder_valves.values():
            for valve in valves:
                if valve.type == "exhaust":
                    valve.apply_global_phase(delta_deg)

    def export_profiles(self, format: str = "dict"):
        if format != "dict":
            raise NotImplementedError("Only dict export implemented")
        return {vid: [v.cam_profile.to_dict() for v in valves] for vid, valves in self.cylinder_valves.items()}

    def import_profiles(self, data) -> None:
        for cid, profiles in data.items():
            for profile_dict in profiles:
                profile = CamProfile.from_dict(profile_dict)
                valve = Valve(
                    valve_id=len(self.cylinder_valves.get(cid, [])) + 1,
                    type="intake",
                    diameter=profile.metadata.get("diameter", 0.03),
                    seat_angle=45.0,
                    cam_profile=profile,
                )
                self.assign_valve_to_cylinder(valve, cid)

    def generate_basic_profile(self, duration: float, max_lift: float, lsa: float, advance: float) -> CamProfile:
        angles = np.linspace(0, 720, 361)
        lift = max_lift * np.sin(np.clip((angles - advance) / duration * math.pi, 0, math.pi))
        return CamProfile(
            angle_lift_table=lift,
            duration_nominal=duration,
            max_lift=max_lift,
            lobe_separation_angle=lsa,
            centerline_timing=advance,
        )


__all__ = ["CamProfile", "Valve", "ValvetrainAssembly"]
