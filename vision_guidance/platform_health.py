from __future__ import annotations

import os
import shutil
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class PlatformHealthSnapshot:
    timestamp_s: float | None = None
    load_1m: float | None = None
    process_rss_mb: float | None = None
    mem_available_mb: float | None = None
    disk_free_gb: float | None = None
    thermal_max_c: float | None = None
    soc_temp_c: float | None = None
    npu_temp_c: float | None = None
    cpu_freq_min_mhz: float | None = None
    cpu_freq_max_mhz: float | None = None
    npu_freq_mhz: float | None = None
    error: str = ""


class PlatformHealthSampler:
    def __init__(self, *, sample_hz: float, log_directory: str | Path):
        if sample_hz <= 0.0:
            raise ValueError("platform health sample_hz must be positive")
        self.sample_hz = float(sample_hz)
        self.log_directory = Path(log_directory)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._snapshot = PlatformHealthSnapshot()
        self._thermal_sources = self._discover_thermal_sources()
        self._cpu_freq_sources = sorted(Path("/sys/devices/system/cpu/cpufreq").glob("policy*/scaling_cur_freq"))
        self._npu_freq_source = self._discover_npu_frequency_source()

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="platform-health", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, 2.0 / self.sample_hz))
        self._thread = None

    def snapshot(self) -> PlatformHealthSnapshot:
        with self._lock:
            return self._snapshot

    def metadata(self) -> dict[str, Any]:
        return {
            "sample_hz": self.sample_hz,
            "thermal_sources": {name: str(path) for name, path in self._thermal_sources},
            "cpu_frequency_sources": [str(path) for path in self._cpu_freq_sources],
            "npu_frequency_source": "" if self._npu_freq_source is None else str(self._npu_freq_source),
            "initial_snapshot": asdict(self.snapshot()),
        }

    def _run(self) -> None:
        period_s = 1.0 / self.sample_hz
        while not self._stop.is_set():
            started_s = time.monotonic()
            sample = self._sample(started_s)
            with self._lock:
                self._snapshot = sample
            self._stop.wait(max(0.0, period_s - (time.monotonic() - started_s)))

    def _sample(self, timestamp_s: float) -> PlatformHealthSnapshot:
        errors: list[str] = []
        load_1m = None
        process_rss_mb = None
        mem_available_mb = None
        disk_free_gb = None
        try:
            load_1m = float(os.getloadavg()[0])
        except (AttributeError, OSError) as exc:
            errors.append(f"load:{exc}")
        try:
            process_rss_mb = self._read_process_rss_mb()
        except (OSError, ValueError) as exc:
            errors.append(f"rss:{exc}")
        try:
            mem_available_mb = self._read_mem_available_mb()
        except (OSError, ValueError) as exc:
            errors.append(f"memory:{exc}")
        try:
            disk_free_gb = float(shutil.disk_usage(self.log_directory).free) / (1024.0**3)
        except OSError as exc:
            errors.append(f"disk:{exc}")

        thermal_values: list[tuple[str, float]] = []
        for name, path in self._thermal_sources:
            try:
                thermal_values.append((name, float(path.read_text(encoding="ascii").strip()) / 1000.0))
            except (OSError, ValueError):
                continue
        thermal_max_c = max((value for _name, value in thermal_values), default=None)
        soc_temp_c = max(
            (value for name, value in thermal_values if "soc" in name or "cpu" in name),
            default=None,
        )
        npu_temp_c = max((value for name, value in thermal_values if "npu" in name), default=None)

        cpu_frequencies: list[float] = []
        for path in self._cpu_freq_sources:
            try:
                cpu_frequencies.append(float(path.read_text(encoding="ascii").strip()) / 1000.0)
            except (OSError, ValueError):
                continue
        npu_freq_mhz = None
        if self._npu_freq_source is not None:
            try:
                npu_freq_mhz = float(self._npu_freq_source.read_text(encoding="ascii").strip()) / 1.0e6
            except (OSError, ValueError):
                pass
        return PlatformHealthSnapshot(
            timestamp_s=timestamp_s,
            load_1m=load_1m,
            process_rss_mb=process_rss_mb,
            mem_available_mb=mem_available_mb,
            disk_free_gb=disk_free_gb,
            thermal_max_c=thermal_max_c,
            soc_temp_c=soc_temp_c,
            npu_temp_c=npu_temp_c,
            cpu_freq_min_mhz=min(cpu_frequencies, default=None),
            cpu_freq_max_mhz=max(cpu_frequencies, default=None),
            npu_freq_mhz=npu_freq_mhz,
            error="; ".join(errors),
        )

    @staticmethod
    def _read_process_rss_mb() -> float:
        for line in Path("/proc/self/status").read_text(encoding="ascii").splitlines():
            if line.startswith("VmRSS:"):
                return float(line.split()[1]) / 1024.0
        raise ValueError("VmRSS missing")

    @staticmethod
    def _read_mem_available_mb() -> float:
        for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
            if line.startswith("MemAvailable:"):
                return float(line.split()[1]) / 1024.0
        raise ValueError("MemAvailable missing")

    @staticmethod
    def _discover_thermal_sources() -> list[tuple[str, Path]]:
        sources: list[tuple[str, Path]] = []
        for zone in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
            try:
                name = (zone / "type").read_text(encoding="ascii").strip().lower()
            except OSError:
                name = zone.name.lower()
            temperature = zone / "temp"
            if temperature.is_file():
                sources.append((name, temperature))
        return sources

    @staticmethod
    def _discover_npu_frequency_source() -> Path | None:
        for path in sorted(Path("/sys/class/devfreq").glob("*/cur_freq")):
            identity = f"{path.parent.name} {path.parent.resolve()}".lower()
            if "npu" in identity or "rknpu" in identity:
                return path
        return None
