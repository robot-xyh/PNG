from __future__ import annotations

import fcntl
import hashlib
import json
import math
import os
import queue
import shlex
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Mapping, Sequence


@dataclass(frozen=True)
class EvidenceFrameConfig:
    enabled: bool = False
    max_fps: float = 5.0
    jpeg_quality: int = 80

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "EvidenceFrameConfig":
        config = cls(
            enabled=bool(values.get("enabled", False)),
            max_fps=float(values.get("max_fps", 5.0)),
            jpeg_quality=int(values.get("jpeg_quality", 80)),
        )
        if not 0.1 <= config.max_fps <= 10.0:
            raise ValueError("evidence frame max_fps must be within 0.1-10")
        if not 40 <= config.jpeg_quality <= 95:
            raise ValueError("evidence frame jpeg_quality must be within 40-95")
        return config


class ExclusiveResourceLock:
    """Advisory process lock with inspectable owner metadata."""

    def __init__(self, resource: str, *, lock_directory: str | Path = "/tmp") -> None:
        self.resource = str(resource)
        token = hashlib.sha256(self.resource.encode("utf-8")).hexdigest()[:16]
        self.path = Path(lock_directory) / f"png-betaflight-{token}.lock"
        self._stream = None

    def acquire(self) -> "ExclusiveResourceLock":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        stream = self.path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            stream.seek(0)
            owner = stream.read().strip() or "owner metadata unavailable"
            stream.close()
            raise RuntimeError(
                f"resource is already locked: {self.resource}; {owner}"
            ) from exc
        stream.seek(0)
        stream.truncate()
        stream.write(
            json.dumps(
                {
                    "pid": os.getpid(),
                    "resource": self.resource,
                    "command": " ".join(shlex.quote(value) for value in os.sys.argv),
                    "created_unix_s": time.time(),
                },
                sort_keys=True,
            )
            + "\n"
        )
        stream.flush()
        os.fsync(stream.fileno())
        self._stream = stream
        return self

    def close(self) -> None:
        stream = self._stream
        self._stream = None
        if stream is None:
            return
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
        finally:
            stream.close()

    def __enter__(self) -> "ExclusiveResourceLock":
        return self.acquire()

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()


class AsyncJpegEvidenceRecorder:
    """Encode and persist latest-only visual evidence outside the control loop."""

    def __init__(
        self,
        directory: Path,
        index_path: Path,
        config: EvidenceFrameConfig,
        *,
        clock=time.monotonic,
    ) -> None:
        self.directory = directory
        self.index_path = index_path
        self.config = config
        self.clock = clock
        self._queue: queue.Queue[tuple[str, Any, float, dict[str, Any]]] = queue.Queue(
            maxsize=1
        )
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._next_offer_s = 0.0
        self._sequence = 0
        self._offered = 0
        self._written = 0
        self._dropped = 0
        self._errors = 0
        self._last_error = ""
        self._index_stream = None

    def start(self) -> None:
        if not self.config.enabled or self._thread is not None:
            return
        self.directory.mkdir(parents=True, exist_ok=True)
        self.index_path.parent.mkdir(parents=True, exist_ok=True)
        self._index_stream = self.index_path.open("w", encoding="utf-8")
        self._thread = threading.Thread(
            target=self._worker,
            name="betaflight-evidence-jpeg",
            daemon=True,
        )
        self._thread.start()

    def wants_preview(self) -> bool:
        return bool(self.config.enabled and not self._stop.is_set())

    def offer_preview(
        self,
        frame_bgr: Any,
        overlay: Mapping[str, Any] | None = None,
    ) -> None:
        if frame_bgr is None or not self._reserve_offer():
            return
        try:
            frame = frame_bgr.copy()
        except Exception as exc:
            self._record_error(exc)
            return
        self._replace(("frame", frame, self.clock(), dict(overlay or {})))

    def offer_encoded_preview(
        self,
        jpeg: bytes | bytearray | memoryview,
        metadata: Mapping[str, Any] | None = None,
    ) -> None:
        if not jpeg or not self._reserve_offer():
            return
        self._replace(("jpeg", bytes(jpeg), self.clock(), dict(metadata or {})))

    def stats(self) -> dict[str, Any]:
        return {
            "evidence_frame_enabled": int(self.config.enabled),
            "evidence_frame_offer_count": self._offered,
            "evidence_frame_write_count": self._written,
            "evidence_frame_drop_count": self._dropped,
            "evidence_frame_error_count": self._errors,
            "evidence_frame_last_error": self._last_error,
            "evidence_frame_index": str(self.index_path) if self.config.enabled else "",
        }

    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=5.0)
            if thread.is_alive():
                self._record_error(RuntimeError("evidence recorder did not stop within 5 s"))
                return
        stream = self._index_stream
        self._index_stream = None
        if stream is not None:
            stream.flush()
            os.fsync(stream.fileno())
            stream.close()

    def _reserve_offer(self) -> bool:
        if not self.config.enabled or self._stop.is_set():
            return False
        now = self.clock()
        if now < self._next_offer_s:
            return False
        self._next_offer_s = now + 1.0 / self.config.max_fps
        self._offered += 1
        return True

    def _replace(self, item: tuple[str, Any, float, dict[str, Any]]) -> None:
        try:
            self._queue.put_nowait(item)
            return
        except queue.Full:
            pass
        try:
            self._queue.get_nowait()
            self._dropped += 1
        except queue.Empty:
            pass
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self._dropped += 1

    def _worker(self) -> None:
        cv2 = None
        while not self._stop.is_set() or not self._queue.empty():
            try:
                kind, payload, timestamp_s, metadata = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if kind == "frame":
                    if cv2 is None:
                        import cv2 as imported_cv2

                        cv2 = imported_cv2
                    ok, encoded = cv2.imencode(
                        ".jpg",
                        payload,
                        [int(cv2.IMWRITE_JPEG_QUALITY), self.config.jpeg_quality],
                    )
                    if not ok:
                        raise RuntimeError("cv2.imencode returned false")
                    jpeg = encoded.tobytes()
                else:
                    jpeg = payload
                self._write(jpeg, timestamp_s=timestamp_s, metadata=metadata)
            except Exception as exc:
                self._record_error(exc)

    def _write(
        self,
        jpeg: bytes,
        *,
        timestamp_s: float,
        metadata: Mapping[str, Any],
    ) -> None:
        self._sequence += 1
        name = f"frame_{self._sequence:06d}_{timestamp_s:.6f}.jpg"
        path = self.directory / name
        temporary = path.with_name(f".{path.name}.tmp")
        with temporary.open("wb") as stream:
            stream.write(jpeg)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        record = {
            "schema_version": 1,
            "sequence": self._sequence,
            "monotonic_s": timestamp_s,
            "path": str(path),
            "sha256": hashlib.sha256(jpeg).hexdigest(),
            "bytes": len(jpeg),
            "metadata": dict(metadata),
        }
        assert self._index_stream is not None
        self._index_stream.write(json.dumps(record, sort_keys=True) + "\n")
        self._index_stream.flush()
        os.fsync(self._index_stream.fileno())
        self._written += 1

    def _record_error(self, exc: BaseException) -> None:
        self._errors += 1
        self._last_error = f"{type(exc).__name__}:{exc}"[:500]


class PreviewEvidenceMux:
    """Fan out preview data to web telemetry and durable evidence recording."""

    def __init__(self, web_sink: Any, recorder: AsyncJpegEvidenceRecorder) -> None:
        self.web_sink = web_sink
        self.recorder = recorder
        web_preview = getattr(getattr(web_sink, "config", None), "preview", None)
        web_rate = float(getattr(web_preview, "max_fps", 1.0))
        web_quality = int(getattr(web_preview, "jpeg_quality", 70))
        self.config = SimpleNamespace(
            preview=SimpleNamespace(
                enabled=bool(
                    recorder.config.enabled
                    or getattr(web_preview, "enabled", False)
                ),
                max_fps=max(web_rate, recorder.config.max_fps),
                jpeg_quality=(
                    recorder.config.jpeg_quality
                    if recorder.config.enabled
                    else web_quality
                ),
            )
        )

    def wants_preview(self) -> bool:
        return bool(
            self.recorder.wants_preview()
            or callable(getattr(self.web_sink, "wants_preview", None))
            and self.web_sink.wants_preview()
        )

    def offer_preview(
        self,
        frame_bgr: Any,
        overlay: Mapping[str, Any] | None = None,
    ) -> None:
        self.recorder.offer_preview(frame_bgr, overlay)
        self.web_sink.offer_preview(frame_bgr, overlay)

    def offer_encoded_preview(
        self,
        jpeg: bytes | bytearray | memoryview,
        metadata: Mapping[str, Any] | None = None,
    ) -> None:
        self.recorder.offer_encoded_preview(jpeg, metadata)
        if self.web_sink.wants_preview():
            self.web_sink.offer_encoded_preview(jpeg)


class OperatorMarkerInbox:
    """Append-only event inbox shared with a small operator CLI."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._offset = 0
        self._partial = ""
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch(exist_ok=True)
        self._offset = path.stat().st_size

    def poll(self) -> list[dict[str, Any]]:
        with self.path.open("r", encoding="utf-8") as stream:
            stream.seek(self._offset)
            chunk = stream.read()
            self._offset = stream.tell()
        if not chunk:
            return []
        lines = (self._partial + chunk).splitlines(keepends=True)
        self._partial = ""
        if lines and not lines[-1].endswith(("\n", "\r")):
            self._partial = lines.pop()
        events = []
        for line in lines:
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                events.append({"event": "invalid_operator_marker", "raw": line.strip()})
                continue
            if isinstance(value, dict):
                events.append(value)
        return events


def append_operator_marker(
    path: Path,
    *,
    event: str,
    note: str = "",
    tags: Sequence[str] = (),
) -> dict[str, Any]:
    if not str(event).strip():
        raise ValueError("event must not be empty")
    path.parent.mkdir(parents=True, exist_ok=True)
    record = {
        "schema_version": 1,
        "event": str(event).strip(),
        "note": str(note),
        "tags": [str(value) for value in tags],
        "pid": os.getpid(),
        "monotonic_s": time.monotonic(),
        "unix_s": time.time(),
    }
    with path.open("a", encoding="utf-8") as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        stream.write(json.dumps(record, sort_keys=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
        fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
    return record


def file_metadata(path: Path) -> dict[str, Any]:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    digest = hashlib.sha256()
    with resolved.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {
        "path": str(resolved),
        "sha256": digest.hexdigest(),
        "bytes": resolved.stat().st_size,
    }


def validate_blackbox_mode_binding(
    path: Path,
    *,
    fc_identity: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Load the firmware-specific contract used to interpret Blackbox flags."""

    path = path.expanduser().resolve()
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("schema_version") != 1:
        raise RuntimeError("unsupported Blackbox mode binding schema")
    binding_id = str(value.get("binding_id", "")).strip()
    expected_identity = value.get("fc_identity")
    decoder = value.get("decoder")
    mode_source = str(value.get("authoritative_mode_source", "")).strip()
    required_host_fields = value.get("required_host_fields")
    if not binding_id or not isinstance(expected_identity, dict):
        raise RuntimeError("Blackbox mode binding identity is incomplete")
    if not isinstance(decoder, dict):
        raise RuntimeError("Blackbox mode binding decoder contract is incomplete")
    if decoder.get("flight_mode_labels_trusted") is not False:
        raise RuntimeError("Blackbox decoder flight-mode labels must be marked untrusted")
    if decoder.get("rotation_units") != "raw":
        raise RuntimeError("Blackbox mode binding requires raw rotation units")
    if mode_source != "host_msp_status_box_ids":
        raise RuntimeError("Blackbox mode decisions must use host MSP status and BOXIDS")
    if (
        not isinstance(required_host_fields, list)
        or not required_host_fields
        or any(not isinstance(field, str) or not field for field in required_host_fields)
    ):
        raise RuntimeError("Blackbox mode binding host field contract is incomplete")
    if fc_identity is not None:
        mismatches = [
            key
            for key, expected in expected_identity.items()
            if fc_identity.get(key) != expected
        ]
        if mismatches:
            raise RuntimeError(
                "Blackbox mode binding firmware mismatch: " + ", ".join(mismatches)
            )
    return {
        "binding_id": binding_id,
        "binding_file": file_metadata(path),
        "fc_identity": dict(expected_identity),
        "decoder": dict(decoder),
        "authoritative_mode_source": mode_source,
        "required_host_fields": list(required_host_fields),
        "decoder_labels_used_for_mode_decisions": False,
    }


def write_run_manifest(
    path: Path,
    *,
    artifacts: Sequence[Path],
    completion: Mapping[str, Any],
    external_artifacts_pending: Sequence[str] = ("console", "blackbox", "target_log"),
) -> dict[str, Any]:
    indexed = []
    missing = []
    for artifact in artifacts:
        if artifact.is_file():
            indexed.append(file_metadata(artifact))
        else:
            missing.append(str(artifact))
    manifest = {
        "schema_version": 1,
        "created_unix_s": time.time(),
        "finalized": False,
        "completion": dict(completion),
        "artifacts": indexed,
        "missing_runtime_artifacts": missing,
        "external_artifacts_pending": list(external_artifacts_pending),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    _fsync_directory(path.parent)
    return manifest


def verify_evidence_frame_index(index_path: Path) -> dict[str, Any]:
    """Verify every frame referenced by an evidence JSONL index."""

    index_path = index_path.expanduser().resolve()
    records: list[dict[str, Any]] = []
    with index_path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise RuntimeError(
                    f"invalid evidence frame index line {line_number}: {index_path}"
                ) from exc
            if not isinstance(record, dict):
                raise RuntimeError(
                    f"evidence frame index line {line_number} is not an object"
                )
            frame_path = Path(str(record.get("path", ""))).expanduser()
            metadata = file_metadata(frame_path)
            if metadata["sha256"] != record.get("sha256"):
                raise RuntimeError(f"evidence frame changed: {frame_path}")
            if metadata["bytes"] != record.get("bytes"):
                raise RuntimeError(f"evidence frame size changed: {frame_path}")
            records.append(record)
    sequences = [record.get("sequence") for record in records]
    if sequences != list(range(1, len(records) + 1)):
        raise RuntimeError("evidence frame index sequence is not contiguous")
    timestamps = [float(record.get("monotonic_s")) for record in records]
    if any(not math.isfinite(value) for value in timestamps):
        raise RuntimeError("evidence frame index contains a non-finite timestamp")
    if any(right <= left for left, right in zip(timestamps, timestamps[1:])):
        raise RuntimeError("evidence frame timestamps are not strictly increasing")
    return {
        "index": file_metadata(index_path),
        "frame_count": len(records),
        "first_monotonic_s": timestamps[0] if timestamps else None,
        "last_monotonic_s": timestamps[-1] if timestamps else None,
    }


def _fsync_directory(path: Path) -> None:
    try:
        directory_fd = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)
