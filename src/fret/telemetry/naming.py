"""Series-id grammar: ``agent.quantity_frame.component`` (FR-SIM-12)."""

from __future__ import annotations

import re
from dataclasses import dataclass

_AGENT_RE = re.compile(r"^[a-z][a-z0-9_]*$")
_QUANTITY_RE = re.compile(r"^[a-z][a-z0-9_]*$")
_COMPONENT_RE = re.compile(r"^[a-z][a-z0-9_]*$")
_SERIES_RE = re.compile(
    r"^(?P<agent>[a-z][a-z0-9_]*)\."
    r"(?P<quantity_frame>[a-z][a-z0-9_]*)\."
    r"(?P<component>[a-z][a-z0-9_]*)$"
)

FRAMES: frozenset[str] = frozenset(
    {
        "enu",
        "ned",
        "body",
        "base",
        "map",
        "joint",
        "ctrl",
    }
)

_VECTOR_COMPONENTS: frozenset[str] = frozenset({"x", "y", "z"})
_QUAT_COMPONENTS: frozenset[str] = frozenset({"qw", "qx", "qy", "qz"})
_SCALAR_COMPONENTS: frozenset[str] = frozenset({"val", "yaw", "rad"})


@dataclass(frozen=True)
class SeriesId:
    """Parsed telemetry series identifier."""

    agent: str
    quantity: str
    frame: str
    component: str

    @property
    def quantity_frame(self) -> str:
        """Return ``quantity_frame`` segment."""
        return f"{self.quantity}_{self.frame}"

    def __str__(self) -> str:
        return f"{self.agent}.{self.quantity_frame}.{self.component}"


def _split_quantity_frame(quantity_frame: str) -> tuple[str, str]:
    """Split ``quantity_frame`` into ``(quantity, frame)``.

    Frames may contain underscores when using the ``sensor_<name>`` form, so
    matching is suffix-based against the frame catalog rather than a blind
    ``rsplit``.
    """
    if "_" not in quantity_frame:
        raise ValueError(
            f"quantity_frame must be '{{quantity}}_{{frame}}', got "
            f"{quantity_frame!r}"
        )
    sensor_marker = "_sensor_"
    if sensor_marker in quantity_frame:
        idx = quantity_frame.find(sensor_marker)
        if idx > 0:
            quantity = quantity_frame[:idx]
            frame = quantity_frame[idx + 1 :]  # sensor_<name>
            if quantity and frame.startswith("sensor_"):
                return quantity, frame
    for frame in sorted(FRAMES, key=len, reverse=True):
        suffix = f"_{frame}"
        if quantity_frame.endswith(suffix):
            quantity = quantity_frame[: -len(suffix)]
            if quantity:
                return quantity, frame
    raise ValueError(
        f"quantity_frame must end with a known frame token, got "
        f"{quantity_frame!r}"
    )


def build_series_id(
    agent: str,
    quantity: str,
    frame: str,
    component: str,
) -> str:
    """Build and validate a series id string."""
    series = SeriesId(
        agent=agent,
        quantity=quantity,
        frame=frame,
        component=component,
    )
    text = str(series)
    validate_series_id(text)
    return text


def parse_series_id(series_id: str) -> SeriesId:
    """Parse a series id into segments.

    Raises:
        ValueError: If the id violates the grammar or frame catalog.
    """
    validate_series_id(series_id)
    match = _SERIES_RE.fullmatch(series_id)
    assert match is not None  # validate_series_id already checked
    quantity, frame = _split_quantity_frame(match.group("quantity_frame"))
    return SeriesId(
        agent=match.group("agent"),
        quantity=quantity,
        frame=frame,
        component=match.group("component"),
    )


def validate_series_id(series_id: str) -> None:
    """Raise ``ValueError`` when ``series_id`` is illegal."""
    if not series_id:
        raise ValueError("series id must be non-empty")
    if any(ch in series_id for ch in ',/" \t\n'):
        raise ValueError(
            f"series id must not contain commas, quotes, spaces, or '/': "
            f"{series_id!r}"
        )
    if series_id != series_id.lower():
        raise ValueError(f"series id must be lowercase: {series_id!r}")
    match = _SERIES_RE.fullmatch(series_id)
    if match is None:
        raise ValueError(
            "series id must match "
            "'{agent}.{quantity}_{frame}.{component}'; "
            f"got {series_id!r}"
        )
    agent = match.group("agent")
    quantity_frame = match.group("quantity_frame")
    component = match.group("component")
    if not _AGENT_RE.fullmatch(agent):
        raise ValueError(f"illegal agent segment: {agent!r}")
    if not _COMPONENT_RE.fullmatch(component):
        raise ValueError(f"illegal component segment: {component!r}")
    quantity, frame = _split_quantity_frame(quantity_frame)
    if not _QUANTITY_RE.fullmatch(quantity):
        raise ValueError(f"illegal quantity segment: {quantity!r}")
    if frame not in FRAMES and not frame.startswith("sensor_"):
        raise ValueError(
            f"unknown frame {frame!r}; expected one of "
            f"{sorted(FRAMES)} or sensor_<name>"
        )
    if frame.startswith("sensor_"):
        suffix = frame[len("sensor_") :]
        if not suffix or not _AGENT_RE.fullmatch(suffix):
            raise ValueError(f"illegal sensor frame: {frame!r}")

    # Component catalog: allow joint names under frame=joint, else known set.
    if frame == "joint":
        return
    allowed = _VECTOR_COMPONENTS | _QUAT_COMPONENTS | _SCALAR_COMPONENTS
    if component not in allowed:
        raise ValueError(
            f"component {component!r} not allowed for frame {frame!r}; "
            f"expected one of {sorted(allowed)} "
            f"(or any joint name when frame='joint')"
        )


def validate_agent_name(name: str) -> None:
    """Raise ``ValueError`` when an agent / simulator name is illegal."""
    if not _AGENT_RE.fullmatch(name):
        raise ValueError(
            f"agent name must match [a-z][a-z0-9_]*; got {name!r}"
        )
