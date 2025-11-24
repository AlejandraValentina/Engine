"""
Engine geometry and configuration models.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Union
import math


RADIANS_PER_DEGREE = math.pi / 180.0


@dataclass
class Cylinder:
    """Represents a single cylinder geometry and kinematics."""

    cylinder_id: int
    bore: float
    stroke: float
    rod_length: float
    compression_ratio: float
    deck_clearance: float = 0.0
    wrist_pin_offset: float = 0.0
    crank_offset: float = 0.0
    bank_id: Optional[int] = None

    def displacement(self) -> float:
        """Returns swept volume of the cylinder [m^3]."""
        return math.pi * (self.bore ** 2) * self.stroke / 4.0

    def clearance_volume(self) -> float:
        """Computes clearance volume derived from compression ratio and displacement."""
        swept = self.displacement()
        return swept / (self.compression_ratio - 1.0)

    def piston_state(self, crank_angle: float) -> Dict[str, float]:
        """Returns piston position, velocity, and acceleration for a crank angle in degrees."""
        theta = (crank_angle + self.crank_offset) * RADIANS_PER_DEGREE
        r = self.stroke / 2.0
        l = self.rod_length
        x = r * (1 - math.cos(theta)) + (l - math.sqrt(l ** 2 - (r * math.sin(theta)) ** 2))
        # Velocity and acceleration via derivatives
        omega = 1.0  # normalized angular speed; scaling handled externally
        sin_t = math.sin(theta)
        cos_t = math.cos(theta)
        term = (r * sin_t) / math.sqrt(max(l ** 2 - (r * sin_t) ** 2, 1e-12))
        dx_dtheta = r * sin_t + term * r * cos_t
        d2x_dtheta2 = r * cos_t + r * (-sin_t * term + (r * cos_t) ** 2 / (l ** 2 - (r * sin_t) ** 2))
        return {
            "position": x + self.deck_clearance,
            "velocity": dx_dtheta * omega,
            "acceleration": d2x_dtheta2 * omega ** 2,
        }

    def instant_volume(self, crank_angle: float) -> float:
        """Instant cylinder volume [m^3] at a given crank angle in degrees."""
        state = self.piston_state(crank_angle)
        swept = self.displacement()
        clearance = self.clearance_volume()
        return clearance + swept * (state["position"] / (self.stroke + self.deck_clearance))

    def instant_volume_derivative(self, crank_angle: float) -> float:
        """Derivative dV/dtheta [m^3/deg] at a crank angle in degrees."""
        theta1 = crank_angle
        theta2 = crank_angle + 0.01
        v1 = self.instant_volume(theta1)
        v2 = self.instant_volume(theta2)
        return (v2 - v1) / (theta2 - theta1)

    def top_dead_center_angle(self) -> float:
        return 0.0

    def bottom_dead_center_angle(self) -> float:
        return 180.0


@dataclass
class Bank:
    """Represents an engine bank with its own cylinders and angle."""

    bank_id: int
    bank_angle: float
    cylinders: List[Cylinder] = field(default_factory=list)

    def add_cylinder(self, cylinder: Cylinder) -> None:
        self.cylinders.append(cylinder)

    def get_cylinders(self) -> List[Cylinder]:
        return list(self.cylinders)

    def relative_firing_sequence(self) -> List[int]:
        return [c.cylinder_id for c in self.cylinders]


@dataclass
class Engine:
    """High-level engine configuration and geometry."""

    n_cylinders: int
    layout: str
    v_angle: Optional[float]
    firing_order: List[int]
    rpm_schedule: Union[float, List[Dict[str, float]]]
    banks: List[Bank]
    shared_intake_manifold: bool = True
    shared_exhaust_manifold: bool = True
    global_mode: str = "fast"

    @classmethod
    def from_config(cls, config_dict: Dict) -> "Engine":
        banks = []
        for bank_cfg in config_dict.get("banks", []):
            cylinders = [
                Cylinder(
                    cylinder_id=cy_cfg["id"],
                    bore=cy_cfg["bore"],
                    stroke=cy_cfg["stroke"],
                    rod_length=cy_cfg["rod_length"],
                    compression_ratio=cy_cfg["compression_ratio"],
                    deck_clearance=cy_cfg.get("deck_clearance", 0.0),
                    wrist_pin_offset=cy_cfg.get("wrist_pin_offset", 0.0),
                    crank_offset=cy_cfg.get("crank_offset", 0.0),
                    bank_id=bank_cfg.get("bank_id"),
                )
                for cy_cfg in bank_cfg.get("cylinders", [])
            ]
            bank = Bank(
                bank_id=bank_cfg.get("bank_id", len(banks)),
                bank_angle=bank_cfg.get("bank_angle", 0.0),
                cylinders=cylinders,
            )
            banks.append(bank)
        return cls(
            n_cylinders=config_dict.get("n_cylinders", sum(len(b.cylinders) for b in banks)),
            layout=config_dict.get("layout", "L"),
            v_angle=config_dict.get("v_angle"),
            firing_order=config_dict.get("firing_order", list(range(1, sum(len(b.cylinders) for b in banks) + 1))),
            rpm_schedule=config_dict.get("rpm_schedule", 0.0),
            banks=banks,
            shared_intake_manifold=config_dict.get("shared_intake_manifold", True),
            shared_exhaust_manifold=config_dict.get("shared_exhaust_manifold", True),
            global_mode=config_dict.get("global_mode", "fast"),
        )

    @classmethod
    def build_standard(
        cls,
        layout: str,
        displacement: float,
        n_cylinders: int,
        v_angle: Optional[float] = None,
        stroke_to_bore: Optional[float] = None,
    ) -> "Engine":
        bore = math.sqrt((4 * displacement) / (math.pi * n_cylinders * (stroke_to_bore or 1.0)))
        stroke = (stroke_to_bore or 1.0) * bore
        cylinders = [
            Cylinder(
                cylinder_id=i + 1,
                bore=bore,
                stroke=stroke,
                rod_length=stroke * 1.7,
                compression_ratio=13.0,
                bank_id=0,
            )
            for i in range(n_cylinders)
        ]
        bank = Bank(bank_id=0, bank_angle=v_angle or 0.0, cylinders=cylinders)
        firing_order = list(range(1, n_cylinders + 1))
        return cls(
            n_cylinders=n_cylinders,
            layout=layout,
            v_angle=v_angle,
            firing_order=firing_order,
            rpm_schedule=0.0,
            banks=[bank],
        )

    def get_cylinder(self, cid: int) -> Cylinder:
        for bank in self.banks:
            for cylinder in bank.cylinders:
                if cylinder.cylinder_id == cid:
                    return cylinder
        raise KeyError(f"Cylinder {cid} not found")

    def get_bank(self, bid: int) -> Bank:
        for bank in self.banks:
            if bank.bank_id == bid:
                return bank
        raise KeyError(f"Bank {bid} not found")

    def volume_at(self, crank_angle: float, cid: int) -> float:
        return self.get_cylinder(cid).instant_volume(crank_angle)

    def volume_derivative(self, crank_angle: float, cid: int) -> float:
        return self.get_cylinder(cid).instant_volume_derivative(crank_angle)

    def piston_kinematics(self, crank_angle: float, cid: int) -> Dict[str, float]:
        return self.get_cylinder(cid).piston_state(crank_angle)

    def firing_angle(self, cid: int) -> float:
        if cid not in self.firing_order:
            raise KeyError(f"Cylinder {cid} not in firing order")
        index = self.firing_order.index(cid)
        return index * (720.0 / len(self.firing_order))

    def synchronize_with(self, valvetrain: "ValvetrainAssembly") -> None:
        # Placeholder for synchronization checks
        del valvetrain
        return

    def iterate_cylinders_in_firing_order(self) -> Sequence[Cylinder]:
        order = []
        for cid in self.firing_order:
            order.append(self.get_cylinder(cid))
        return order


__all__ = ["Engine", "Bank", "Cylinder"]
