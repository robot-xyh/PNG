from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import numpy as np


THRUST_LUT_SCHEMA_VERSION = 1
THRUST_LUT_MODEL_TYPE = "voltage_throttle_specific_force_lut"


@dataclass(frozen=True)
class ThrustLookupResult:
    voltage_v: float
    required_specific_force_m_s2: float
    throttle_us: float
    minimum_force_m_s2: float
    maximum_force_m_s2: float
    limited: bool


class VoltageThrottleThrustModel:
    """Monotonic bilinear voltage/throttle to specific-force calibration."""

    def __init__(
        self,
        *,
        calibration_id: str,
        voltage_v: np.ndarray,
        throttle_us: np.ndarray,
        specific_force_m_s2: np.ndarray,
        validation: Mapping[str, object],
        fit: Mapping[str, object] | None = None,
        dynamics: Mapping[str, object] | None = None,
        source_path: str = "",
        source_sha256: str = "",
    ) -> None:
        self.calibration_id = str(calibration_id).strip()
        self.voltage_v = np.asarray(voltage_v, dtype=float)
        self.throttle_us = np.asarray(throttle_us, dtype=float)
        self.specific_force_m_s2 = np.asarray(specific_force_m_s2, dtype=float)
        self.validation = dict(validation)
        self.fit = {} if fit is None else dict(fit)
        self.dynamics = {} if dynamics is None else dict(dynamics)
        self.source_path = str(source_path)
        self.source_sha256 = str(source_sha256)
        self._validate()

    @classmethod
    def from_mapping(
        cls,
        values: Mapping[str, object],
        *,
        source_path: str = "",
        source_sha256: str = "",
    ) -> "VoltageThrottleThrustModel":
        if int(values.get("schema_version", 0)) != THRUST_LUT_SCHEMA_VERSION:
            raise ValueError("unsupported thrust LUT schema_version")
        if str(values.get("model_type", "")) != THRUST_LUT_MODEL_TYPE:
            raise ValueError("unsupported thrust LUT model_type")
        return cls(
            calibration_id=str(values.get("calibration_id", "")),
            voltage_v=np.asarray(values.get("voltage_v", []), dtype=float),
            throttle_us=np.asarray(values.get("throttle_us", []), dtype=float),
            specific_force_m_s2=np.asarray(
                values.get("specific_force_m_s2", []), dtype=float
            ),
            validation=dict(values.get("validation", {})),
            fit=dict(values.get("fit", {})),
            dynamics=dict(values.get("dynamics", {})),
            source_path=source_path,
            source_sha256=source_sha256,
        )

    @classmethod
    def from_file(
        cls,
        path: str | Path,
        *,
        expected_sha256: str,
        expected_calibration_id: str = "",
    ) -> "VoltageThrottleThrustModel":
        resolved = Path(path).expanduser().resolve()
        payload = resolved.read_bytes()
        actual_sha256 = hashlib.sha256(payload).hexdigest()
        if actual_sha256 != str(expected_sha256).strip().lower():
            raise ValueError("thrust LUT SHA256 mismatch")
        values = json.loads(payload)
        if not isinstance(values, dict):
            raise ValueError("thrust LUT root must be an object")
        model = cls.from_mapping(
            values,
            source_path=str(resolved),
            source_sha256=actual_sha256,
        )
        if expected_calibration_id and model.calibration_id != expected_calibration_id:
            raise ValueError("thrust LUT calibration_id mismatch")
        return model

    @property
    def minimum_voltage_v(self) -> float:
        return float(self.voltage_v[0])

    @property
    def maximum_voltage_v(self) -> float:
        return float(self.voltage_v[-1])

    def covers_voltage(self, voltage_v: float | None) -> bool:
        if voltage_v is None:
            return False
        value = float(voltage_v)
        return bool(
            math.isfinite(value)
            and self.minimum_voltage_v <= value <= self.maximum_voltage_v
        )

    def specific_force(self, voltage_v: float, throttle_us: float) -> float:
        voltage = self._bounded_voltage(voltage_v)
        throttle = float(throttle_us)
        if not math.isfinite(throttle):
            raise ValueError("throttle must be finite")
        if throttle < self.throttle_us[0] or throttle > self.throttle_us[-1]:
            raise ValueError("throttle is outside thrust LUT coverage")
        force_by_throttle = self._force_row_at_voltage(voltage)
        return float(np.interp(throttle, self.throttle_us, force_by_throttle))

    def throttle_for_specific_force(
        self,
        voltage_v: float,
        required_specific_force_m_s2: float,
    ) -> ThrustLookupResult:
        voltage = self._bounded_voltage(voltage_v)
        required = float(required_specific_force_m_s2)
        if not math.isfinite(required) or required <= 0.0:
            raise ValueError("required specific force must be finite and positive")
        force_by_throttle = self._force_row_at_voltage(voltage)
        minimum = float(force_by_throttle[0])
        maximum = float(force_by_throttle[-1])
        bounded = float(np.clip(required, minimum, maximum))
        throttle = float(np.interp(bounded, force_by_throttle, self.throttle_us))
        return ThrustLookupResult(
            voltage_v=voltage,
            required_specific_force_m_s2=required,
            throttle_us=throttle,
            minimum_force_m_s2=minimum,
            maximum_force_m_s2=maximum,
            limited=not math.isclose(required, bounded, rel_tol=0.0, abs_tol=1.0e-9),
        )

    def metadata(self) -> dict[str, object]:
        return {
            "model_type": THRUST_LUT_MODEL_TYPE,
            "calibration_id": self.calibration_id,
            "path": self.source_path,
            "sha256": self.source_sha256,
            "voltage_coverage_v": [self.minimum_voltage_v, self.maximum_voltage_v],
            "throttle_coverage_us": [
                float(self.throttle_us[0]),
                float(self.throttle_us[-1]),
            ],
            "validation": dict(self.validation),
            "fit": dict(self.fit),
            "dynamics": dict(self.dynamics),
        }

    @property
    def first_order_time_constant_s(self) -> float | None:
        try:
            value = float(self.dynamics["first_order_time_constant_s"])
        except (KeyError, TypeError, ValueError):
            return None
        return value if math.isfinite(value) and value > 0.0 else None

    def _validate(self) -> None:
        if not self.calibration_id:
            raise ValueError("thrust LUT calibration_id is required")
        if self.voltage_v.ndim != 1 or len(self.voltage_v) < 2:
            raise ValueError("thrust LUT requires at least two voltage knots")
        if self.throttle_us.ndim != 1 or len(self.throttle_us) < 3:
            raise ValueError("thrust LUT requires at least three throttle knots")
        expected_shape = (len(self.voltage_v), len(self.throttle_us))
        if self.specific_force_m_s2.shape != expected_shape:
            raise ValueError("thrust LUT force matrix shape does not match its axes")
        arrays = (self.voltage_v, self.throttle_us, self.specific_force_m_s2)
        if any(not np.all(np.isfinite(value)) for value in arrays):
            raise ValueError("thrust LUT values must be finite")
        if np.any(np.diff(self.voltage_v) <= 0.0):
            raise ValueError("thrust LUT voltage knots must be strictly increasing")
        if np.any(np.diff(self.throttle_us) <= 0.0):
            raise ValueError("thrust LUT throttle knots must be strictly increasing")
        if np.any(self.specific_force_m_s2 <= 0.0):
            raise ValueError("thrust LUT force values must be positive")
        if np.any(np.diff(self.specific_force_m_s2, axis=1) <= 0.0):
            raise ValueError("thrust LUT force must increase with throttle")
        if np.any(np.diff(self.specific_force_m_s2, axis=0) < 0.0):
            raise ValueError("thrust LUT force must not decrease with voltage")
        median_error = _finite_metric(self.validation, "median_relative_error")
        p95_error = _finite_metric(self.validation, "p95_relative_error")
        if self.validation.get("passed") is not True:
            raise ValueError("thrust LUT validation is not passed")
        if median_error > 0.10 or p95_error > 0.20:
            raise ValueError("thrust LUT validation error exceeds release limits")

    def _bounded_voltage(self, voltage_v: float) -> float:
        value = float(voltage_v)
        if not self.covers_voltage(value):
            raise ValueError("battery voltage is outside thrust LUT coverage")
        return value

    def _force_row_at_voltage(self, voltage_v: float) -> np.ndarray:
        return np.asarray(
            [
                np.interp(voltage_v, self.voltage_v, self.specific_force_m_s2[:, index])
                for index in range(len(self.throttle_us))
            ],
            dtype=float,
        )


def _finite_metric(values: Mapping[str, object], key: str) -> float:
    try:
        value = float(values[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"thrust LUT validation requires {key}") from exc
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"thrust LUT validation {key} must be finite and non-negative")
    return value
