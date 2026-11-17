"""ROS-free safety supervisor for the SO-101 bridge and sim twin (C2).

Sits between velocity→position integrator output and the actuator write.
Watchdog, limit/step clamp, enable lease, and latched category-2 ``stop``
are enforced here. Fault entry and torque enable/disable live in
:class:`~fret.hardware.bridge_core.BridgeCore`; this module never names
``stop`` an emergency stop.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, cast

import numpy as np
import numpy.typing as npt


@dataclass(frozen=True, slots=True)
class SafetyConfig:
    """Thresholds loaded from ``config/hardware/so101.yml`` ``safety:``."""

    watchdog_s: float = 0.1
    enable_lease_s: float = 0.5
    limit_margin_rad: float = 0.03
    max_joint_velocity: float = 2.0
    dt_s: float = 0.02
    fault_failed_cycles: int = 5
    ease_in_speed_rad_s: float = 0.3
    out_of_limit_refuse_margin_rad: float = 0.15
    torque_limit: int = 1000
    protection_current: int = 500
    max_temperature: int = 70


@dataclass
class SafetyDiagnostics:
    """Observable supervisor reasons (greppable from tests and ROS)."""

    watchdog: bool = False
    lease_expired: bool = False
    stop_latched: bool = False
    controller_fault: bool = False
    clamp_events: int = 0
    reasons: list[str] = field(default_factory=list)

    def reset_cycle_flags(self) -> None:
        self.watchdog = False
        self.lease_expired = False
        self.reasons.clear()


class SafetySupervisor:
    """Filter position goals before SYNC WRITE / MuJoCo ctrl write.

    ``active`` is True for the real supervisor (FR-SYS-19). The PR3
    :class:`~fret.hardware.bridge_core.IdentitySafetySupervisor` reports
    ``active=False``.
    """

    def __init__(
        self,
        joint_limits_rad: npt.NDArray[np.float64],
        *,
        config: SafetyConfig | None = None,
        calibrated_limits_rad: npt.NDArray[np.float64] | None = None,
    ) -> None:
        limits = np.asarray(joint_limits_rad, dtype=np.float64)
        if limits.ndim != 2 or limits.shape[1] != 2:
            raise ValueError(
                f"joint_limits_rad must be (DOF, 2), got {limits.shape}"
            )
        self._profile_limits = limits
        if calibrated_limits_rad is None:
            self._cal_limits = limits.copy()
        else:
            cal = np.asarray(calibrated_limits_rad, dtype=np.float64)
            if cal.shape != limits.shape:
                raise ValueError(
                    "calibrated_limits_rad shape must match joint_limits_rad"
                )
            self._cal_limits = cal
        self._config = config or SafetyConfig()
        self._diag = SafetyDiagnostics()
        self._last_command_s: float | None = None
        self._lease_deadline_s: float | None = None
        self._stop_latched = False
        self._controller_fault = False
        self._last_written: npt.NDArray[np.float64] | None = None
        self._enabled = False
        margin = self._config.limit_margin_rad
        lo = np.maximum(self._profile_limits[:, 0], self._cal_limits[:, 0])
        hi = np.minimum(self._profile_limits[:, 1], self._cal_limits[:, 1])
        self._eff_lo = lo + margin
        self._eff_hi = hi - margin
        # Degenerate after margin: fall back to mid-point clamp.
        for i in range(self._eff_lo.shape[0]):
            if self._eff_lo[i] > self._eff_hi[i]:
                mid = 0.5 * (lo[i] + hi[i])
                self._eff_lo[i] = mid
                self._eff_hi[i] = mid

    @property
    def active(self) -> bool:
        return True

    @property
    def config(self) -> SafetyConfig:
        return self._config

    @property
    def diagnostics(self) -> SafetyDiagnostics:
        return self._diag

    @property
    def stop_latched(self) -> bool:
        return self._stop_latched

    @property
    def controller_fault(self) -> bool:
        return self._controller_fault

    @property
    def enabled(self) -> bool:
        return self._enabled

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = bool(enabled)
        if not enabled:
            self._lease_deadline_s = None

    def note_command(self, now_s: float) -> None:
        """Record arrival of a valid command (resets watchdog)."""
        self._last_command_s = float(now_s)

    def renew_lease(self, now_s: float) -> None:
        """Assert the enable lease (level/lease, not an edge)."""
        self._lease_deadline_s = float(now_s) + self._config.enable_lease_s

    def latch_stop(self) -> None:
        """Category-2 controlled stop to hold; latched until :meth:`clear_stop`."""
        self._stop_latched = True
        self._diag.stop_latched = True

    def clear_stop(self) -> None:
        """Clear a latched stop. Does not enable motion."""
        self._stop_latched = False
        self._diag.stop_latched = False

    def set_controller_fault(self, active: bool) -> None:
        self._controller_fault = bool(active)
        self._diag.controller_fault = self._controller_fault

    def soft_start_goals(
        self, measured_rad: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Return goals equal to measured pose (soft start, FR-HW-44)."""
        measured = np.asarray(measured_rad, dtype=np.float64)
        goals = measured.copy()
        self._last_written = goals.copy()
        return goals

    def check_enable_pose(
        self, measured_rad: npt.NDArray[np.float64]
    ) -> str | None:
        """Return a refusal reason if measured pose is too far outside limits."""
        measured = np.asarray(measured_rad, dtype=np.float64)
        refuse = self._config.out_of_limit_refuse_margin_rad
        for i in range(measured.shape[0]):
            if measured[i] < self._eff_lo[i] - refuse:
                return (
                    f"joint {i} at {measured[i]:.4f} rad below limit "
                    f"{self._eff_lo[i]:.4f} by more than {refuse:.4f}"
                )
            if measured[i] > self._eff_hi[i] + refuse:
                return (
                    f"joint {i} at {measured[i]:.4f} rad above limit "
                    f"{self._eff_hi[i]:.4f} by more than {refuse:.4f}"
                )
        return None

    def filter_goals(
        self,
        goals_rad: npt.NDArray[np.float64],
        measured_rad: npt.NDArray[np.float64],
        now_s: float,
    ) -> npt.NDArray[np.float64]:
        """Clamp / hold goals before the actuator write.

        Hold (goal = measured) when: stop latched, controller fault,
        watchdog expired while enabled, or enable lease expired while
        enabled. Otherwise clamp to effective limits and limit per-cycle
        step by ``max_joint_velocity × dt``.
        """
        goals = np.asarray(goals_rad, dtype=np.float64).copy()
        measured = np.asarray(measured_rad, dtype=np.float64)
        if goals.shape != measured.shape:
            raise ValueError(
                f"goals shape {goals.shape} != measured {measured.shape}"
            )
        self._diag.reset_cycle_flags()
        self._diag.stop_latched = self._stop_latched
        self._diag.controller_fault = self._controller_fault

        hold = False
        if self._stop_latched:
            hold = True
            self._diag.reasons.append("stop")
        if self._controller_fault:
            hold = True
            self._diag.reasons.append("controller_fault")

        if self._enabled:
            if (
                self._last_command_s is None
                or (now_s - self._last_command_s) > self._config.watchdog_s
            ):
                hold = True
                self._diag.watchdog = True
                self._diag.reasons.append("watchdog")
            if (
                self._lease_deadline_s is None
                or now_s > self._lease_deadline_s
            ):
                hold = True
                self._diag.lease_expired = True
                self._diag.reasons.append("lease")

        if hold:
            out = measured.copy()
            self._last_written = out.copy()
            return out

        # Effective limit clamp.
        before = goals.copy()
        np.clip(goals, self._eff_lo, self._eff_hi, out=goals)
        if not np.allclose(before, goals):
            self._diag.clamp_events += 1
            self._diag.reasons.append("limit_clamp")

        # Ease joints that started slightly outside the effective band.
        ease = self._config.ease_in_speed_rad_s * self._config.dt_s
        for i in range(goals.shape[0]):
            if measured[i] < self._eff_lo[i]:
                goals[i] = min(self._eff_lo[i], measured[i] + max(ease, 0.0))
            elif measured[i] > self._eff_hi[i]:
                goals[i] = max(self._eff_hi[i], measured[i] - max(ease, 0.0))

        # Per-cycle step limit relative to last written (or measured).
        ref = (
            self._last_written if self._last_written is not None else measured
        )
        max_step = self._config.max_joint_velocity * self._config.dt_s
        delta = goals - ref
        stepped = False
        for i in range(goals.shape[0]):
            if abs(delta[i]) > max_step:
                goals[i] = ref[i] + np.sign(delta[i]) * max_step
                stepped = True
        if stepped:
            self._diag.clamp_events += 1
            self._diag.reasons.append("step_clamp")

        self._last_written = goals.copy()
        return goals


def default_safety_config_from_mapping(
    data: dict[str, object] | None,
) -> SafetyConfig:
    """Build :class:`SafetyConfig` from a YAML ``safety:`` mapping."""
    raw: dict[str, Any] = dict(data) if data is not None else {}

    def _float(key: str, default: float) -> float:
        return float(cast(Any, raw.get(key, default)))

    def _int(key: str, default: int) -> int:
        return int(cast(Any, raw.get(key, default)))

    return SafetyConfig(
        watchdog_s=_float("watchdog_s", 0.1),
        enable_lease_s=_float("enable_lease_s", 0.5),
        limit_margin_rad=_float("limit_margin_rad", 0.03),
        max_joint_velocity=_float("max_joint_velocity", 2.0),
        dt_s=_float("dt_s", 0.02),
        fault_failed_cycles=_int("fault_failed_cycles", 5),
        ease_in_speed_rad_s=_float("ease_in_speed_rad_s", 0.3),
        out_of_limit_refuse_margin_rad=_float(
            "out_of_limit_refuse_margin_rad", 0.15
        ),
        torque_limit=_int("torque_limit", 1000),
        protection_current=_int("protection_current", 500),
        max_temperature=_int("max_temperature", 70),
    )
