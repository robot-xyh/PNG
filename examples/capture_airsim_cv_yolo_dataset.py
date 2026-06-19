from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "datasets" / "airsim_cv_intruder_yolo"


@dataclass(frozen=True)
class CapturePose:
    position: np.ndarray
    yaw_rad: float
    pitch_rad: float
    roll_rad: float = 0.0


@dataclass(frozen=True)
class YoloBox:
    class_id: int
    x_center: float
    y_center: float
    width: float
    height: float
    raw_xyxy: tuple[float, float, float, float]
    name: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture AirSim Blocks ComputerVision images and write YOLOv8 labels "
            "from AirSim simGetDetections()."
        )
    )
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--split", default="train")
    parser.add_argument("--count", type=int, default=500)
    parser.add_argument("--seed", type=int, default=20260619)
    parser.add_argument("--camera", default="0")
    parser.add_argument("--image-type", default="Scene")
    parser.add_argument("--target-pattern", default="Intruder*")
    parser.add_argument("--class-id", type=int, default=0)
    parser.add_argument("--detection-radius-cm", type=float, default=50000.0)
    parser.add_argument(
        "--intruder-actor",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Spawn or move a scene Actor as the target before capturing images.",
    )
    parser.add_argument("--intruder-actor-name", default="IntruderActor")
    parser.add_argument("--intruder-actor-asset", default="1M_Cube_Chamfer")
    parser.add_argument("--intruder-actor-scale", type=float, default=2.0)
    parser.add_argument("--intruder-actor-x", type=float, default=0.0)
    parser.add_argument("--intruder-actor-y", type=float, default=0.0)
    parser.add_argument("--intruder-actor-z", type=float, default=-50.0)
    parser.add_argument("--intruder-actor-yaw-deg", type=float, default=90.0)
    parser.add_argument("--intruder-actor-physics", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--intruder-actor-blueprint", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--intruder-actor-respawn", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--width", type=int, default=640, help="Expected width; actual AirSim image size is used for labels.")
    parser.add_argument("--height", type=int, default=480, help="Expected height; actual AirSim image size is used for labels.")
    parser.add_argument("--range-min-m", type=float, default=20.0)
    parser.add_argument("--range-max-m", type=float, default=160.0)
    parser.add_argument("--z-offset-min-m", type=float, default=-40.0)
    parser.add_argument("--z-offset-max-m", type=float, default=40.0)
    parser.add_argument("--pitch-jitter-deg", type=float, default=4.0)
    parser.add_argument("--yaw-jitter-deg", type=float, default=6.0)
    parser.add_argument("--roll-jitter-deg", type=float, default=0.0)
    parser.add_argument("--settle-s", type=float, default=0.03)
    parser.add_argument("--max-attempts", type=int, default=5000)
    parser.add_argument("--save-empty", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--strict-target-pose",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Exit if target pose cannot be found instead of falling back to scan poses.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate arguments and output paths without connecting to AirSim or writing images.",
    )
    return parser.parse_args()


def _require_positive(value: float, name: str) -> None:
    if value <= 0.0:
        raise SystemExit(f"{name} must be positive")


def _validate_args(args: argparse.Namespace) -> None:
    if args.count <= 0:
        raise SystemExit("--count must be positive")
    if args.max_attempts < args.count:
        raise SystemExit("--max-attempts must be >= --count")
    _require_positive(args.detection_radius_cm, "--detection-radius-cm")
    _require_positive(args.range_min_m, "--range-min-m")
    _require_positive(args.range_max_m, "--range-max-m")
    if args.range_max_m < args.range_min_m:
        raise SystemExit("--range-max-m must be >= --range-min-m")
    if args.width <= 0 or args.height <= 0:
        raise SystemExit("--width and --height must be positive")
    if args.intruder_actor and args.intruder_actor_scale <= 0.0:
        raise SystemExit("--intruder-actor-scale must be positive")


def _image_type(airsim_module, name: str):
    try:
        return getattr(airsim_module.ImageType, name)
    except AttributeError as exc:
        raise SystemExit(f"Unsupported AirSim ImageType: {name}") from exc


def _vector_to_np(vector) -> np.ndarray:
    return np.array([float(vector.x_val), float(vector.y_val), float(vector.z_val)], dtype=float)


def _finite_pose_position(pose) -> Optional[np.ndarray]:
    position = _vector_to_np(pose.position)
    if np.all(np.isfinite(position)):
        return position
    return None


def _actor_name(args: argparse.Namespace) -> str:
    return str(args.intruder_actor_name or "IntruderActor")


def _actor_position(args: argparse.Namespace) -> np.ndarray:
    return np.array(
        [
            float(args.intruder_actor_x),
            float(args.intruder_actor_y),
            float(args.intruder_actor_z),
        ],
        dtype=float,
    )


def _actor_pose(airsim_module, position: np.ndarray, yaw_deg: float):
    return airsim_module.Pose(
        airsim_module.Vector3r(float(position[0]), float(position[1]), float(position[2])),
        airsim_module.to_quaternion(0.0, 0.0, math.radians(float(yaw_deg))),
    )


def _spawn_or_move_intruder_actor(client, airsim_module, args: argparse.Namespace) -> tuple[str, np.ndarray]:
    object_name = _actor_name(args)
    position = _actor_position(args)
    pose = _actor_pose(airsim_module, position, float(args.intruder_actor_yaw_deg))
    if args.intruder_actor_respawn:
        try:
            client.simDestroyObject(object_name)
        except Exception:
            pass
    else:
        try:
            moved = client.simSetObjectPose(object_name, pose, teleport=True)
            if moved is None or bool(moved):
                return object_name, position
        except Exception:
            pass
        try:
            scene_objects = client.simListSceneObjects(f"{object_name}*")
        except Exception:
            scene_objects = []
        if object_name in scene_objects:
            client.simSetObjectPose(object_name, pose, teleport=True)
            return object_name, position

    spawned = False
    try:
        scale = airsim_module.Vector3r(
            float(args.intruder_actor_scale),
            float(args.intruder_actor_scale),
            float(args.intruder_actor_scale),
        )
        spawned = bool(
            client.simSpawnObject(
                object_name,
                args.intruder_actor_asset,
                pose,
                scale,
                bool(args.intruder_actor_physics),
                bool(args.intruder_actor_blueprint),
            )
        )
    except Exception as exc:
        print(f"intruder_actor_spawn_warning={exc}; trying simSetObjectPose on existing object")
    if not spawned:
        try:
            client.simSetObjectPose(object_name, pose, teleport=True)
        except Exception as exc:
            raise SystemExit(
                f"Failed to spawn or move intruder actor '{object_name}' with asset "
                f"'{args.intruder_actor_asset}': {exc}"
            ) from exc
    return object_name, position


def _target_patterns(args: argparse.Namespace) -> list[str]:
    patterns = [str(args.target_pattern or "")]
    if args.intruder_actor:
        patterns.extend([_actor_name(args), f"{_actor_name(args)}*", str(args.intruder_actor_asset or "")])
    seen: set[str] = set()
    unique: list[str] = []
    for pattern in patterns:
        item = pattern.strip()
        if item and item not in seen:
            unique.append(item)
            seen.add(item)
    return unique


def _find_target(client, patterns: list[str]) -> tuple[str, Optional[np.ndarray]]:
    if not patterns:
        return "", None
    all_names: list[str] = []
    for pattern in patterns:
        try:
            names = list(client.simListSceneObjects(pattern))
        except Exception as exc:
            print(f"target_list_warning pattern={pattern!r}: {exc}")
            names = []
        all_names.extend(name for name in names if name)
    names = sorted(set(all_names))
    for name in names:
        try:
            position = _finite_pose_position(client.simGetObjectPose(name))
        except Exception as exc:
            print(f"target_pose_warning name={name!r}: {exc}")
            continue
        if position is not None:
            return name, position
    return (names[0], None) if names else ("", None)


def _look_at_pose(camera_position: np.ndarray, target_position: np.ndarray) -> CapturePose:
    relative = np.asarray(target_position, dtype=float) - np.asarray(camera_position, dtype=float)
    horizontal = math.hypot(float(relative[0]), float(relative[1]))
    yaw_rad = math.atan2(float(relative[1]), float(relative[0]))
    # AirSim to_quaternion uses pitch positive down in NED conventions.
    pitch_rad = math.atan2(float(relative[2]), max(1.0e-9, horizontal))
    return CapturePose(np.asarray(camera_position, dtype=float), yaw_rad, pitch_rad, 0.0)


def _sample_pose_around_target(target_position: np.ndarray, rng: random.Random, args: argparse.Namespace) -> CapturePose:
    radius = rng.uniform(float(args.range_min_m), float(args.range_max_m))
    bearing = rng.uniform(-math.pi, math.pi)
    z_offset = rng.uniform(float(args.z_offset_min_m), float(args.z_offset_max_m))
    camera_position = np.array(
        [
            float(target_position[0]) - radius * math.cos(bearing),
            float(target_position[1]) - radius * math.sin(bearing),
            float(target_position[2]) + z_offset,
        ],
        dtype=float,
    )
    pose = _look_at_pose(camera_position, target_position)
    yaw_jitter = math.radians(float(args.yaw_jitter_deg)) * rng.uniform(-1.0, 1.0)
    pitch_jitter = math.radians(float(args.pitch_jitter_deg)) * rng.uniform(-1.0, 1.0)
    roll_jitter = math.radians(float(args.roll_jitter_deg)) * rng.uniform(-1.0, 1.0)
    return CapturePose(
        pose.position,
        pose.yaw_rad + yaw_jitter,
        pose.pitch_rad + pitch_jitter,
        roll_jitter,
    )


def _scan_poses(rng: random.Random, args: argparse.Namespace) -> Iterable[CapturePose]:
    for _ in range(max(1, int(args.max_attempts))):
        x = rng.uniform(-float(args.range_max_m), float(args.range_max_m))
        y = rng.uniform(-float(args.range_max_m), float(args.range_max_m))
        z = rng.uniform(-80.0, -5.0)
        yaw_rad = rng.uniform(-math.pi, math.pi)
        pitch_rad = rng.uniform(math.radians(-35.0), math.radians(35.0))
        roll_rad = math.radians(float(args.roll_jitter_deg)) * rng.uniform(-1.0, 1.0)
        yield CapturePose(np.array([x, y, z], dtype=float), yaw_rad, pitch_rad, roll_rad)


def _airsim_pose(airsim_module, capture_pose: CapturePose):
    return airsim_module.Pose(
        airsim_module.Vector3r(
            float(capture_pose.position[0]),
            float(capture_pose.position[1]),
            float(capture_pose.position[2]),
        ),
        airsim_module.to_quaternion(
            float(capture_pose.pitch_rad),
            float(capture_pose.roll_rad),
            float(capture_pose.yaw_rad),
        ),
    )


def _configure_detection(client, airsim_module, args: argparse.Namespace) -> None:
    image_type = _image_type(airsim_module, args.image_type)
    clear = getattr(client, "simClearDetectionMeshNames", None)
    if callable(clear):
        clear(args.camera, image_type)
    client.simSetDetectionFilterRadius(args.camera, image_type, float(args.detection_radius_cm))
    for pattern in _target_patterns(args):
        client.simAddDetectionFilterMeshName(args.camera, image_type, pattern)


def _capture_png(client, airsim_module, args: argparse.Namespace) -> bytes:
    request = airsim_module.ImageRequest(
        args.camera,
        _image_type(airsim_module, args.image_type),
        False,
        True,
    )
    responses = client.simGetImages([request])
    if not responses:
        raise RuntimeError("simGetImages returned no responses")
    response = responses[0]
    data = bytes(getattr(response, "image_data_uint8", b"") or b"")
    if not data:
        raise RuntimeError("AirSim image response is empty")
    return data


def _decode_size(cv2, png_bytes: bytes) -> tuple[int, int]:
    image = cv2.imdecode(np.frombuffer(png_bytes, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError("OpenCV failed to decode AirSim PNG image")
    height, width = image.shape[:2]
    return int(width), int(height)


def _detections(client, airsim_module, args: argparse.Namespace):
    return list(client.simGetDetections(args.camera, _image_type(airsim_module, args.image_type)))


def _clip(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def _detection_to_yolo(detection, image_width: int, image_height: int, class_id: int) -> Optional[YoloBox]:
    box = detection.box2D
    x1 = _clip(float(box.min.x_val), 0.0, float(image_width))
    y1 = _clip(float(box.min.y_val), 0.0, float(image_height))
    x2 = _clip(float(box.max.x_val), 0.0, float(image_width))
    y2 = _clip(float(box.max.y_val), 0.0, float(image_height))
    if x2 <= x1 or y2 <= y1:
        return None
    width = x2 - x1
    height = y2 - y1
    return YoloBox(
        class_id=int(class_id),
        x_center=((x1 + x2) * 0.5) / float(image_width),
        y_center=((y1 + y2) * 0.5) / float(image_height),
        width=width / float(image_width),
        height=height / float(image_height),
        raw_xyxy=(x1, y1, x2, y2),
        name=str(getattr(detection, "name", "") or ""),
    )


def _write_label(path: Path, boxes: list[YoloBox]) -> None:
    lines = [
        f"{box.class_id} {box.x_center:.8f} {box.y_center:.8f} {box.width:.8f} {box.height:.8f}"
        for box in boxes
    ]
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def _manifest_row(
    *,
    image_path: Path,
    label_path: Path,
    target_name: str,
    capture_pose: CapturePose,
    image_width: int,
    image_height: int,
    detections_count: int,
    boxes: list[YoloBox],
) -> dict:
    return {
        "image_path": str(image_path),
        "label_path": str(label_path),
        "target_name": target_name,
        "timestamp_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "image_width": image_width,
        "image_height": image_height,
        "detections_count": detections_count,
        "boxes": [
            {
                "name": box.name,
                "class_id": box.class_id,
                "xyxy": list(box.raw_xyxy),
                "xywhn": [box.x_center, box.y_center, box.width, box.height],
            }
            for box in boxes
        ],
        "camera_pose_ned": {
            "x": float(capture_pose.position[0]),
            "y": float(capture_pose.position[1]),
            "z": float(capture_pose.position[2]),
            "pitch_deg": math.degrees(float(capture_pose.pitch_rad)),
            "roll_deg": math.degrees(float(capture_pose.roll_rad)),
            "yaw_deg": math.degrees(float(capture_pose.yaw_rad)),
        },
    }


def _output_paths(args: argparse.Namespace) -> tuple[Path, Path, Path, Path]:
    base = Path(args.output_dir).expanduser()
    image_dir = base / "images" / args.split
    label_dir = base / "labels" / args.split
    manifest_path = base / "manifest.jsonl"
    classes_path = base / "classes.txt"
    return image_dir, label_dir, manifest_path, classes_path


def _prepare_output(args: argparse.Namespace) -> tuple[Path, Path, Path, Path]:
    image_dir, label_dir, manifest_path, classes_path = _output_paths(args)
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    if not classes_path.exists():
        classes_path.write_text("intruder\n", encoding="utf-8")
    return image_dir, label_dir, manifest_path, classes_path


def _pose_iterator(target_position: Optional[np.ndarray], rng: random.Random, args: argparse.Namespace) -> Iterable[CapturePose]:
    if target_position is None:
        yield from _scan_poses(rng, args)
        return
    for _ in range(max(1, int(args.max_attempts))):
        yield _sample_pose_around_target(target_position, rng, args)


def main() -> None:
    args = parse_args()
    _validate_args(args)
    image_dir, label_dir, manifest_path, classes_path = _output_paths(args)
    if args.dry_run:
        print("dry_run=True")
        print(f"images_dir={image_dir}")
        print(f"labels_dir={label_dir}")
        print(f"manifest={manifest_path}")
        print(f"classes={classes_path}")
        if args.intruder_actor:
            print(
                "intruder_actor="
                f"name={_actor_name(args)}, asset={args.intruder_actor_asset}, "
                f"scale={args.intruder_actor_scale}, "
                f"position={np.array2string(_actor_position(args), precision=3)}"
            )
            print(f"target_patterns={','.join(_target_patterns(args))}")
        return

    try:
        import airsim
    except ImportError as exc:
        raise SystemExit("Install the AirSim Python package before running this script.") from exc
    try:
        import cv2
    except ImportError as exc:
        raise SystemExit("Install OpenCV before running this script: python3 -m pip install opencv-python") from exc

    client = airsim.VehicleClient()
    try:
        client.confirmConnection()
    except Exception as exc:
        raise SystemExit("Failed to connect to AirSim RPC. Start Blocks in ComputerVision mode first.") from exc

    image_dir, label_dir, manifest_path, classes_path = _prepare_output(args)
    actor_target_name = ""
    actor_target_position: Optional[np.ndarray] = None
    if args.intruder_actor:
        actor_target_name, actor_target_position = _spawn_or_move_intruder_actor(client, airsim, args)
        print(
            "intruder_actor_ready "
            f"name={actor_target_name!r} asset={args.intruder_actor_asset!r} "
            f"scale={args.intruder_actor_scale:.3f} "
            f"position={np.array2string(actor_target_position, precision=3)}"
        )
    _configure_detection(client, airsim, args)
    target_name, target_position = _find_target(client, _target_patterns(args))
    if target_position is None and actor_target_position is not None:
        target_name = actor_target_name
        target_position = actor_target_position
    if target_position is None and args.strict_target_pose:
        raise SystemExit(f"Target pose not found for patterns {_target_patterns(args)!r}")
    if target_position is None:
        print(f"target_pose_unavailable patterns={_target_patterns(args)!r}; using scan poses.")
    else:
        print(f"target={target_name!r} position={np.array2string(target_position, precision=3)}")
    print(f"output_images={image_dir}")
    print(f"output_labels={label_dir}")

    rng = random.Random(int(args.seed))
    saved = 0
    attempts = 0
    with manifest_path.open("a", encoding="utf-8") as manifest:
        for capture_pose in _pose_iterator(target_position, rng, args):
            if saved >= args.count or attempts >= args.max_attempts:
                break
            attempts += 1
            client.simSetVehiclePose(_airsim_pose(airsim, capture_pose), True)
            if args.settle_s > 0.0:
                time.sleep(float(args.settle_s))
            try:
                png_bytes = _capture_png(client, airsim, args)
                image_width, image_height = _decode_size(cv2, png_bytes)
                detections = _detections(client, airsim, args)
            except Exception as exc:
                print(f"capture_warning attempt={attempts}: {exc}")
                continue
            boxes = [
                box
                for detection in detections
                if (box := _detection_to_yolo(detection, image_width, image_height, args.class_id)) is not None
            ]
            if not boxes and not args.save_empty:
                continue
            stem = f"{saved:06d}"
            image_path = image_dir / f"{stem}.png"
            label_path = label_dir / f"{stem}.txt"
            image_path.write_bytes(png_bytes)
            _write_label(label_path, boxes)
            row = _manifest_row(
                image_path=image_path,
                label_path=label_path,
                target_name=target_name,
                capture_pose=capture_pose,
                image_width=image_width,
                image_height=image_height,
                detections_count=len(detections),
                boxes=boxes,
            )
            manifest.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
            saved += 1
            if saved % 25 == 0 or saved == args.count:
                print(f"saved={saved}/{args.count} attempts={attempts} last_boxes={len(boxes)}")

    print(f"done saved={saved} attempts={attempts} manifest={manifest_path} classes={classes_path}")
    if saved < args.count:
        print("warning: saved fewer images than requested; increase --max-attempts or adjust pose ranges.")


if __name__ == "__main__":
    main()
