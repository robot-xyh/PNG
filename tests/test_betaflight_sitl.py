import struct
import importlib.util
import hashlib
import io
import json
import socket
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

import numpy as np

from vision_guidance.betaflight_sitl import (
    FDM_PACKET_STRUCT,
    RC_PACKET_STRUCT,
    SERVO_PACKET_STRUCT,
    SERVO_RAW_PACKET_STRUCT,
    BetaflightFdmPacket,
    GazeboPoseSample,
    GazeboProjectedDetectionSource,
    SitlPilotRcConfig,
    SitlPilotRcScheduler,
    pack_fdm_packet,
    pack_rc_packet,
    gazebo_pose_to_body_frd_euler_deg,
    project_target_box,
    quaternion_rotation_matrix_wxyz,
    sitl_truth_stats,
    unpack_servo_packet,
    unpack_servo_raw_packet,
    validate_loopback_sitl_config,
)


def _load_materializer():
    path = Path(__file__).resolve().parents[1] / "tools" / "materialize_betaflight_sitl_config.py"
    spec = importlib.util.spec_from_file_location("materialize_betaflight_sitl_config_for_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


materializer = _load_materializer()


def _load_tool(filename, module_name):
    path = Path(__file__).resolve().parents[1] / "tools" / filename
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


from tools import audit_betaflight_gazebo_sil as audit_tool
from tools import configure_betaflight_sitl as configure_tool
from tools import run_betaflight_gazebo_sil as orchestrator
from vision_guidance.types import CameraIntrinsics


class BetaflightSitlTest(unittest.TestCase):
    def test_materialized_config_preserves_controller_and_removes_flight_authority(self):
        source_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "betaflight.rk3588.velocity_png.flight_supervised.json"
        )
        source_bytes = source_path.read_bytes()
        source = json.loads(source_bytes)
        generated = materializer.materialize_sitl_config(
            source,
            policy="noncollision",
            source_path=source_path,
            source_sha256=hashlib.sha256(source_bytes).hexdigest(),
            simulated_vbat_v=23.6,
        )

        self.assertNotIn("flight_profile", generated)
        self.assertFalse(generated["control_authorization"]["enabled"])
        self.assertEqual(generated["serial"]["port"], "socket://127.0.0.1:5761")
        self.assertEqual(
            generated["guidance"]["velocity_establishing_png"],
            source["guidance"]["velocity_establishing_png"],
        )
        self.assertEqual(
            generated["msp_runtime"]["raw_imu_gyro"]["axis_sign"], [1, -1, 1]
        )
        self.assertEqual(generated["msp_runtime"]["raw_imu_poll_hz"], 20)
        self.assertIsNone(generated["sitl_profile"]["pilot_rc"]["motion_test_after_s"])
        self.assertEqual(generated["sitl_profile"]["pilot_rc"]["takeover_after_s"], 7.35)
        self.assertEqual(generated["sitl_profile"]["pilot_rc"]["takeover_duration_s"], 0.9)
        self.assertEqual(validate_loopback_sitl_config(generated)["simulated_vbat_v"], 23.6)

    def test_materialized_contact_config_uses_policy_specific_takeover_timing(self):
        source_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "betaflight.rk3588.velocity_png.flight_contact_supervised.json"
        )
        source_bytes = source_path.read_bytes()

        generated = materializer.materialize_sitl_config(
            json.loads(source_bytes),
            policy="contact",
            source_path=source_path,
            source_sha256=hashlib.sha256(source_bytes).hexdigest(),
            simulated_vbat_v=23.6,
        )

        self.assertEqual(generated["sitl_profile"]["pilot_rc"]["takeover_after_s"], 7.70)
        self.assertEqual(generated["sitl_profile"]["pilot_rc"]["takeover_duration_s"], 0.9)
        self.assertEqual(generated["sitl_profile"]["pilot_rc"]["motion_test_after_s"], 6.5)
        self.assertEqual(generated["sitl_profile"]["pilot_rc"]["motion_test_delta_us"], 50)
        self.assertEqual(generated["sitl_profile"]["pilot_rc"]["motion_test_axis"], "roll")

    def test_rendered_target_approach_preserves_terminal_detection_standoff(self):
        model_path = (
            Path(__file__).resolve().parents[1]
            / "sitl"
            / "gazebo"
            / "models"
            / "target_uav"
            / "model.sdf"
        )
        root = ET.parse(model_path).getroot()
        plugin = root.find(".//plugin[@name='png::sitl::DeterministicTargetMotion']")

        self.assertIsNotNone(plugin)
        assert plugin is not None
        speed = float(plugin.findtext("verticalApproachSpeedMps", "nan"))
        maximum_approach = float(plugin.findtext("maximumVerticalApproachM", "nan"))
        initial_camera_range = 7.0 - 0.16 - 0.06
        final_standoff = initial_camera_range - maximum_approach

        self.assertEqual(speed, 10.0)
        self.assertGreaterEqual(final_standoff, 0.8)
        self.assertLessEqual(final_standoff, 1.0)

    def test_target_model_approach_is_materialized_per_policy(self):
        source_path = (
            Path(__file__).resolve().parents[1]
            / "sitl"
            / "gazebo"
            / "models"
            / "target_uav"
            / "model.sdf"
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            noncollision_projected = orchestrator.materialize_target_model(
                source_path,
                root / "noncollision_projected",
                "noncollision",
                "projected",
            )
            noncollision_rendered = orchestrator.materialize_target_model(
                source_path,
                root / "noncollision_rendered",
                "noncollision",
                "rendered",
            )
            contact_projected = orchestrator.materialize_target_model(
                source_path, root / "contact_projected", "contact", "projected"
            )
            contact_rendered = orchestrator.materialize_target_model(
                source_path, root / "contact_rendered", "contact", "rendered"
            )

            def approach(path):
                plugin = ET.parse(path).getroot().find(
                    ".//plugin[@name='png::sitl::DeterministicTargetMotion']"
                )
                assert plugin is not None
                return (
                    float(plugin.findtext("verticalApproachStartS", "nan")),
                    float(plugin.findtext("verticalApproachSpeedMps", "nan")),
                    float(plugin.findtext("maximumVerticalApproachM", "nan")),
                )

            self.assertEqual(approach(noncollision_projected), (7.5, 10.0, 5.95))
            self.assertEqual(approach(noncollision_rendered), (7.1, 2.5, 6.2))
            self.assertEqual(approach(contact_projected), (7.5, 10.0, 5.95))
            self.assertEqual(approach(contact_rendered), (7.5, 10.0, 5.95))

    def test_fdm_packet_matches_official_2025_12_2_layout(self):
        packet = BetaflightFdmPacket(
            timestamp_s=1.25,
            angular_velocity_body_rad_s=(1.0, 2.0, 3.0),
            linear_acceleration_body_m_s2=(4.0, 5.0, 6.0),
            orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
            velocity_enu_m_s=(7.0, 8.0, 9.0),
            longitude_latitude_altitude=(113.86, 22.80, 10.0),
            pressure_pa=101325.0,
        )

        encoded = pack_fdm_packet(packet)

        self.assertEqual(len(encoded), FDM_PACKET_STRUCT.size)
        self.assertEqual(FDM_PACKET_STRUCT.size, 18 * 8)
        self.assertEqual(FDM_PACKET_STRUCT.unpack(encoded)[0], 1.25)
        self.assertEqual(FDM_PACKET_STRUCT.unpack(encoded)[-1], 101325.0)

    def test_rc_packet_pads_to_sixteen_channels(self):
        encoded = pack_rc_packet(2.0, (1500, 1500, 1200, 1500, 1000, 1000, 2000, 1000))
        unpacked = RC_PACKET_STRUCT.unpack(encoded)

        self.assertEqual(unpacked[0], 2.0)
        self.assertEqual(unpacked[1:9], (1500, 1500, 1200, 1500, 1000, 1000, 2000, 1000))
        self.assertEqual(unpacked[9:], (1000,) * 8)
        with self.assertRaisesRegex(ValueError, "sane"):
            pack_rc_packet(0.0, (500,))

    def test_motor_packets_match_sitl_abi(self):
        normalized = unpack_servo_packet(SERVO_PACKET_STRUCT.pack(0.1, 0.2, 0.3, 0.4))
        raw = unpack_servo_raw_packet(
            SERVO_RAW_PACKET_STRUCT.pack(4, 1000, 1100, 1200, 1300, *([0.0] * 12))
        )

        np.testing.assert_allclose(normalized, (0.1, 0.2, 0.3, 0.4), rtol=1.0e-7)
        self.assertEqual(raw, (1000.0, 1100.0, 1200.0, 1300.0))

    def test_quaternion_rotation_is_normalized(self):
        rotation = quaternion_rotation_matrix_wxyz((2.0, 0.0, 0.0, 0.0))

        np.testing.assert_allclose(rotation, np.eye(3), atol=1.0e-12)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-12)

    def test_gazebo_pose_converts_to_frd_ned_euler(self):
        yaw_enu_90 = (
            math.cos(math.pi / 4.0),
            0.0,
            0.0,
            math.sin(math.pi / 4.0),
        )

        roll, pitch, yaw = gazebo_pose_to_body_frd_euler_deg(yaw_enu_90)

        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)
        self.assertAlmostEqual(yaw, 0.0, places=6)

    def test_upward_camera_projects_target_above_image_center(self):
        interceptor = self._pose((0.0, 0.0, 1.0))
        target = self._pose((0.0, 0.0, 11.0))
        intrinsics = CameraIntrinsics(530.8443, 532.2955, 321.0279, 247.2573, 640, 512)
        R_BC = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)

        bbox, depth = project_target_box(
            interceptor,
            target,
            intrinsics=intrinsics,
            R_BC=R_BC,
            target_size_m=(0.5, 0.5, 0.2),
        )

        self.assertIsNotNone(bbox)
        self.assertGreater(depth, 9.0)
        center_x = 0.5 * (bbox[0] + bbox[2])
        center_y = 0.5 * (bbox[1] + bbox[3])
        self.assertAlmostEqual(center_x, intrinsics.cx, delta=1.0)
        self.assertAlmostEqual(center_y, intrinsics.cy, delta=1.0)

    def test_upward_camera_rejects_target_below(self):
        interceptor = self._pose((0.0, 0.0, 2.0))
        target = self._pose((0.0, 0.0, 0.0))
        intrinsics = CameraIntrinsics(500, 500, 320, 256, 640, 512)
        R_BC = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)

        bbox, _ = project_target_box(
            interceptor,
            target,
            intrinsics=intrinsics,
            R_BC=R_BC,
            target_size_m=(0.5, 0.5, 0.2),
        )

        self.assertIsNone(bbox)

    def test_sitl_config_is_strictly_loopback_and_not_flight_scoped(self):
        config = {
            "sitl_profile": {
                "scope": "betaflight_sitl_loopback_v1",
                "loopback_only": True,
                "simulated_telemetry_provenance": "gazebo_truth",
                "simulated_vbat_v": 23.6,
                "simulated_voltage_provenance": "sitl_config_only",
                "projected_detection_latency_s": 0.04,
                "pilot_rc": {},
            },
            "serial": {"port": "socket://127.0.0.1:5761"},
        }

        self.assertEqual(validate_loopback_sitl_config(config)["scope"], "betaflight_sitl_loopback_v1")
        config["serial"]["port"] = "/dev/ttyS1"
        with self.assertRaisesRegex(RuntimeError, "socket"):
            validate_loopback_sitl_config(config)
        config["serial"]["port"] = "socket://127.0.0.1:5761"
        config["flight_profile"] = {"scope": "flight_contact_short_supervised_v2"}
        with self.assertRaisesRegex(RuntimeError, "real-flight"):
            validate_loopback_sitl_config(config)

    def test_sitl_pilot_rc_sequence_matches_real_aux_modes(self):
        scheduler = SitlPilotRcScheduler(
            SitlPilotRcConfig(
                arm_after_s=3.0,
                takeover_after_s=8.0,
                takeover_duration_s=0.7,
                disarm_after_s=11.0,
            )
        )
        try:
            self.assertEqual(scheduler.channels_at(0.0)[4:], (2000, 1000, 1000, 1000))
            self.assertEqual(scheduler.channels_at(3.5)[:5], (1500, 1500, 1000, 1500, 1000))
            self.assertEqual(scheduler.channels_at(4.0)[:5], (1500, 1500, 1275, 1500, 1000))
            self.assertEqual(scheduler.channels_at(8.2)[4:], (1000, 1000, 2000, 1000))
            self.assertEqual(scheduler.channels_at(9.0)[4:], (1000, 1000, 1000, 1000))
            self.assertEqual(scheduler.channels_at(12.0)[4:], (2000, 1000, 1000, 1000))
        finally:
            scheduler.close()

    def test_sitl_pilot_rejects_takeover_longer_than_flight_limit(self):
        with self.assertRaisesRegex(ValueError, "0.5-0.9"):
            SitlPilotRcConfig.from_mapping({"takeover_duration_s": 1.0})

    def test_sitl_pilot_motion_pulse_is_symmetric(self):
        scheduler = SitlPilotRcScheduler(
            SitlPilotRcConfig(
                motion_test_after_s=5.0,
                takeover_after_s=8.0,
            )
        )
        try:
            self.assertEqual(scheduler.channels_at(5.1)[:2], (1600, 1600))
            self.assertEqual(scheduler.channels_at(5.5)[:2], (1400, 1400))
            self.assertEqual(scheduler.channels_at(5.9)[:2], (1500, 1500))
        finally:
            scheduler.close()

    def test_sitl_pilot_motion_can_use_one_axis(self):
        scheduler = SitlPilotRcScheduler(
            SitlPilotRcConfig(
                motion_test_after_s=5.0,
                motion_test_axis="roll",
                takeover_after_s=8.0,
            )
        )
        try:
            self.assertEqual(scheduler.channels_at(5.1)[:2], (1600, 1500))
            self.assertEqual(scheduler.channels_at(5.5)[:2], (1400, 1500))
        finally:
            scheduler.close()

    def test_projected_detector_exposes_enu_and_expected_ned_truth(self):
        detector = object.__new__(GazeboProjectedDetectionSource)
        detector.interceptor_model = "interceptor"
        detector.target_model = "target"
        detector.intrinsics = CameraIntrinsics(500, 500, 320, 256, 640, 512)
        detector.R_BC = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
        detector.target_size_m = (0.5, 0.5, 0.2)
        detector.detection_latency_s = 0.04
        detector._last_pose_stamp = None
        interceptor = self._pose((1.0, 2.0, 1.0), velocity=(3.0, 4.0, 5.0))
        target = self._pose((1.0, 2.0, 8.0), velocity=(0.5, -0.5, 0.0))
        detector.stream = mock.Mock()
        detector.stream.topic = "/model/interceptor/pose,/model/target/pose"
        detector.stream.latest.side_effect = [interceptor, target]

        detection, stats = detector.detect(
            timestamp=1.1, frame_id=3, active_track_id=None
        )

        self.assertIsNotNone(detection)
        self.assertEqual(stats["sitl_interceptor_position_enu_x_m"], 1.0)
        self.assertEqual(stats["sitl_expected_velocity_ned_n_m_s"], 4.0)
        self.assertEqual(stats["sitl_expected_velocity_ned_e_m_s"], 3.0)
        self.assertEqual(stats["sitl_expected_velocity_ned_d_m_s"], -5.0)
        self.assertAlmostEqual(stats["sitl_interceptor_pitch_frd_deg"], 0.0)
        self.assertAlmostEqual(stats["sitl_expected_msp_pitch_deg"], 0.0)
        self.assertTrue(math.isfinite(stats["sitl_projected_bbox_center_x"]))

    def test_sitl_truth_stats_are_available_to_rendered_detector(self):
        interceptor = self._pose((1.0, 2.0, 3.0), velocity=(4.0, 5.0, 6.0))
        target = self._pose((7.0, 8.0, 9.0))

        stats = sitl_truth_stats(interceptor, target, timestamp=1.1)

        self.assertEqual(stats["sitl_expected_velocity_ned_n_m_s"], 5.0)
        self.assertEqual(stats["sitl_expected_velocity_ned_e_m_s"], 4.0)
        self.assertEqual(stats["sitl_expected_velocity_ned_d_m_s"], -6.0)

    def test_gazebo_bridge_matches_flight_candidate_sensor_conventions(self):
        source = (
            Path(__file__).resolve().parents[1]
            / "sitl/gazebo/PngBetaflightSilBridge.cc"
        ).read_text(encoding="utf-8")

        self.assertIn("packet.orientationBodyFrdToNedWxyz", source)
        self.assertNotIn("bodyFrdToNed.Inverse()", source)
        self.assertIn(
            "packet.angularVelocityGazeboBodyFlu", source
        )
        self.assertIn(
            "angularFlu.X(), -angularFlu.Y(), -angularFlu.Z()", source
        )
        self.assertIn("horizontalApproachDecayS", source)
        self.assertIn("cameraAlignmentStartS", source)
        self.assertIn("AlignWithInterceptorCamera", source)
        self.assertIn("opticalAxisWorld", source)

    def test_cli_prompt_must_be_at_response_suffix(self):
        class ChunkSocket:
            def __init__(self):
                self.chunks = [b"echo contains\r\n# not-a-prompt", b"\r\nvalue\r\n# "]

            def recv(self, _size):
                return self.chunks.pop(0)

        response = configure_tool._receive_until(
            ChunkSocket(), b"\r\n# ", 0.5, require_suffix=True
        )

        self.assertTrue(response.endswith(b"\r\n# "))
        self.assertIn(b"value", response)

    def test_configure_restarts_sitl_to_verify_saved_eeprom(self):
        class FakeSocket:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return None

            def sendall(self, _value):
                return None

        class FakeProcess:
            def __init__(self, create_eeprom=None):
                self.returncode = None
                self.create_eeprom = create_eeprom

            def poll(self):
                return self.returncode

            def wait(self, timeout=None):
                del timeout
                if self.create_eeprom is not None:
                    self.create_eeprom.write_bytes(b"eeprom")
                self.returncode = 0
                return 0

            def terminate(self):
                self.returncode = 0

            def kill(self):
                self.returncode = -9

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root / "betaflight"
            binary.write_bytes(b"binary")
            source = root / "source"
            source.mkdir()
            cli = root / "commands.txt"
            cli.write_text("feature GPS\n", encoding="utf-8")
            run_dir = root / "run"
            eeprom = run_dir / "eeprom.bin"
            processes = [FakeProcess(eeprom), FakeProcess()]
            with (
                mock.patch.object(
                    configure_tool,
                    "sha256_path",
                    return_value=configure_tool.OFFICIAL_BETAFLIGHT_ELF_SHA256,
                ),
                mock.patch.object(
                    configure_tool,
                    "_git_commit",
                    return_value=configure_tool.OFFICIAL_BETAFLIGHT_COMMIT,
                ),
                mock.patch.object(
                    configure_tool, "_start_sitl", side_effect=processes
                ) as start,
                mock.patch.object(
                    configure_tool, "_wait_for_cli_port", return_value=FakeSocket()
                ),
                mock.patch.object(configure_tool, "_receive_until", return_value=b"\r\n# "),
                mock.patch.object(configure_tool, "_verify_saved_configuration") as verify,
            ):
                configure_tool.configure(
                    binary=binary,
                    source_tree=source,
                    cli_path=cli,
                    run_dir=run_dir,
                    timeout_s=1.0,
                )

            self.assertEqual(start.call_count, 2)
            verify.assert_called_once_with(processes[1], 1.0)

    def test_configure_rejects_unbound_elf_before_start(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root / "betaflight"
            binary.write_bytes(b"wrong")
            source = root / "source"
            source.mkdir()
            cli = root / "commands.txt"
            cli.write_text("feature GPS\n", encoding="utf-8")
            with (
                mock.patch.object(configure_tool, "sha256_path", return_value="bad"),
                mock.patch.object(
                    configure_tool,
                    "_git_commit",
                    return_value=configure_tool.OFFICIAL_BETAFLIGHT_COMMIT,
                ),
                mock.patch.object(configure_tool, "_start_sitl") as start,
            ):
                with self.assertRaisesRegex(RuntimeError, "ELF SHA256"):
                    configure_tool.configure(
                        binary=binary,
                        source_tree=source,
                        cli_path=cli,
                        run_dir=root / "run",
                        timeout_s=1.0,
                    )
            start.assert_not_called()

    def test_orchestrator_starts_fc_before_gazebo_and_runner(self):
        started = []

        class FakeProcess:
            returncode = None

            def poll(self):
                return self.returncode

        def start(name, command, **kwargs):
            del command, kwargs
            started.append(name)
            return orchestrator.ManagedProcess(name, FakeProcess(), Path(name), io.BytesIO())

        processes, sequence = orchestrator.start_runtime_stack(
            binary=Path("betaflight"),
            eeprom_dir=Path("eeprom"),
            gazebo_command=["gz"],
            runner_command=["runner"],
            root=Path("."),
            run_dir=Path("."),
            gazebo_env={},
            startup_wait_s=0.0,
            start_process=start,
            wait_for_listener=lambda *_args: None,
            wait_alive=lambda *_args: None,
            wait_for_gazebo=lambda *_args: None,
        )

        self.assertEqual(started, ["betaflight", "gazebo", "runner"])
        self.assertEqual(sequence, ["start_betaflight", "start_gazebo", "start_runner"])
        self.assertEqual([item.name for item in processes], started)

    def test_orchestrator_cleans_up_started_process_on_failure(self):
        class FakeProcess:
            def __init__(self):
                self.returncode = None
                self.terminated = False

            def poll(self):
                return self.returncode

            def terminate(self):
                self.terminated = True
                self.returncode = -15

            def wait(self, timeout=None):
                del timeout
                return self.returncode

        process = FakeProcess()
        stream = io.BytesIO()

        def start(name, command, **kwargs):
            del command, kwargs
            if name == "gazebo":
                raise RuntimeError("gazebo failed")
            return orchestrator.ManagedProcess(name, process, Path(name), stream)

        with self.assertRaisesRegex(RuntimeError, "gazebo failed"):
            orchestrator.start_runtime_stack(
                binary=Path("betaflight"),
                eeprom_dir=Path("eeprom"),
                gazebo_command=["gz"],
                runner_command=["runner"],
                root=Path("."),
                run_dir=Path("."),
                gazebo_env={},
                startup_wait_s=0.0,
                start_process=start,
                wait_for_listener=lambda *_args: None,
                wait_alive=lambda *_args: None,
                wait_for_gazebo=lambda *_args: None,
            )

        self.assertTrue(process.terminated)
        self.assertTrue(stream.closed)

    def test_runner_readiness_wait_uses_durable_log_marker(self):
        class FakeProcess:
            returncode = None

            def poll(self):
                return self.returncode

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runner.log"
            path.write_text("initializing\nMSP RAW_IMU gyro: available=1\n")

            orchestrator._wait_for_log_marker(
                FakeProcess(), path, orchestrator.RUNNER_READY_MARKER, 0.1
            )

    def test_runner_readiness_wait_rejects_early_exit(self):
        class FakeProcess:
            returncode = 1

            def poll(self):
                return self.returncode

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runner.log"
            with self.assertRaisesRegex(RuntimeError, "readiness marker"):
                orchestrator._wait_for_log_marker(
                    FakeProcess(), path, orchestrator.RUNNER_READY_MARKER, 0.1
                )

    def test_sil_timing_thresholds_accept_and_reject_boundary(self):
        passing = {
            "msp_set_raw_rc_write_rate_hz": "49.0",
            "msp_set_raw_rc_write_p999_interval_s": "0.040",
            "msp_set_raw_rc_write_max_interval_s": "0.060",
        }
        _, passing_violations = audit_tool.evaluate_msp_timing(passing)
        self.assertEqual(passing_violations, [])

        failing = dict(passing)
        failing["msp_set_raw_rc_write_rate_hz"] = "48.99"
        failing["msp_set_raw_rc_write_p999_interval_s"] = "0.041"
        failing["msp_set_raw_rc_write_max_interval_s"] = "0.061"
        _, failing_violations = audit_tool.evaluate_msp_timing(failing)
        self.assertEqual(len(failing_violations), 3)

    def test_motor_direction_audit_finds_delayed_response(self):
        rows = []
        commands = (-4.0, -2.0, 0.0, 2.0, 4.0) * 4
        delayed = (0.0, 0.0) + commands[:-2]
        for index, (command, response) in enumerate(zip(commands, delayed)):
            rows.append(
                {
                    "elapsed_s": f"{index * 0.02:.2f}",
                    "msp_publish_mode": "algorithm",
                    "sp_roll_rate_deg_s": str(command),
                    "motor_output_ch1": str(1300.0 - response),
                    "motor_output_ch2": str(1300.0 - response),
                    "motor_output_ch3": str(1300.0 + response),
                    "motor_output_ch4": str(1300.0 + response),
                }
            )

        metric = audit_tool._motor_direction_metrics(rows, "roll")

        self.assertGreater(float(metric["correlation"]), 0.99)
        self.assertAlmostEqual(float(metric["lag_s"]), 0.04, places=6)
        self.assertEqual(metric["sample_count"], 15)

    def test_motor_direction_audit_uses_takeover_onset_not_pid_braking(self):
        rows = []
        for index in range(30):
            command = float(index if index < 20 else 40 - index)
            response = command if index < 20 else -command
            rows.append(
                {
                    "elapsed_s": f"{index * 0.02:.2f}",
                    "msp_publish_mode": "algorithm",
                    "sp_roll_rate_deg_s": str(command),
                    "motor_output_ch1": str(1300.0 - response),
                    "motor_output_ch2": str(1300.0 - response),
                    "motor_output_ch3": str(1300.0 + response),
                    "motor_output_ch4": str(1300.0 + response),
                }
            )

        metric = audit_tool._motor_direction_metrics(rows, "roll")

        self.assertGreater(float(metric["correlation"]), 0.99)
        self.assertLessEqual(float(metric["lag_s"]), 0.02)

    def test_command_response_audit_uses_takeover_onset_not_pid_braking(self):
        rows = []
        for index in range(30):
            command = float(index if index < 20 else 40 - index)
            response = command if index < 20 else -command
            rows.append(
                {
                    "elapsed_s": f"{index * 0.02:.2f}",
                    "msp_publish_mode": "algorithm",
                    "sp_roll_rate_deg_s": str(command),
                    "gyro_roll_deg_s": str(response),
                }
            )

        metric = audit_tool._best_delayed_correlation(
            rows,
            command_field="sp_roll_rate_deg_s",
            response_field="gyro_roll_deg_s",
            onset_window_s=0.30,
        )

        self.assertGreater(float(metric["correlation"]), 0.99)
        self.assertLessEqual(float(metric["lag_s"]), 0.02)

    def test_ned_truth_audit_accounts_for_causal_filter_delay(self):
        rows = []
        delay_rows = 5
        truth = [1.0 if (index // 16) % 2 == 0 else -1.0 for index in range(96)]
        for index, value in enumerate(truth):
            delayed = truth[max(0, index - delay_rows)]
            rows.append(
                {
                    "elapsed_s": str(0.02 * index),
                    "sitl_expected_velocity_ned_n_m_s": str(value),
                    "kinematics_velocity_filtered_n_m_s": str(delayed),
                    "sitl_expected_velocity_ned_e_m_s": "1.0",
                    "kinematics_velocity_filtered_e_m_s": "1.0",
                    "sitl_expected_velocity_ned_d_m_s": "0.0",
                    "kinematics_velocity_filtered_d_m_s": "0.0",
                }
            )

        metrics = audit_tool._ned_truth_metrics(rows)

        self.assertAlmostEqual(float(metrics["n"]["lag_s"]), 0.1)
        self.assertEqual(metrics["n"]["sign_match_fraction"], 1.0)
        self.assertEqual(metrics["horizontal_axes_with_motion"], 2)

    def test_ned_truth_policy_requires_both_axes_for_noncollision(self):
        metrics = {
            "n": {"sample_count": 40, "sign_match_fraction": 0.75},
            "e": {"sample_count": 40, "sign_match_fraction": 1.0},
        }

        result = audit_tool.evaluate_ned_truth_policy(metrics, "noncollision")

        self.assertFalse(result["passed"])
        self.assertEqual(result["reason"], "horizontal_sign_invalid")

    def test_ned_truth_policy_accepts_one_axis_for_short_contact_window(self):
        metrics = {
            "n": {"sample_count": 40, "sign_match_fraction": 0.75},
            "e": {"sample_count": 40, "sign_match_fraction": 1.0},
        }

        result = audit_tool.evaluate_ned_truth_policy(metrics, "contact")

        self.assertTrue(result["passed"])
        self.assertEqual(result["valid_axes"], ["e"])

    def test_terminal_audit_rejects_policy_activity_after_takeover_release(self):
        rows = [
            {
                "elapsed_s": "1.0",
                "takeover_requested": "1",
                "msp_publish_mode": "algorithm",
                "intercept_phase": "TRACKING",
                "intercept_terminal_trigger": "",
            },
            {
                "elapsed_s": "1.1",
                "takeover_requested": "0",
                "msp_publish_mode": "live_passthrough",
                "intercept_phase": "TERMINAL_VISUAL",
                "intercept_terminal_trigger": "contact_ttc_terminal",
            },
            {
                "elapsed_s": "1.2",
                "takeover_requested": "0",
                "msp_publish_mode": "live_passthrough",
                "intercept_phase": "COMPLETE",
                "intercept_terminal_trigger": "contact_bbox_complete",
            },
        ]

        metrics = audit_tool.evaluate_terminal_policy(rows, "contact")

        self.assertFalse(metrics["passed"])
        self.assertEqual(metrics["phases"], ["TRACKING"])

    def test_terminal_audit_requires_contact_entry_during_takeover(self):
        rows = [
            {
                "elapsed_s": "1.0",
                "takeover_requested": "0",
                "msp_publish_mode": "live_passthrough",
                "intercept_phase": "TERMINAL_VISUAL",
                "intercept_terminal_trigger": "contact_ttc_terminal",
            },
            {
                "elapsed_s": "1.1",
                "takeover_requested": "1",
                "msp_publish_mode": "algorithm",
                "intercept_phase": "TERMINAL_VISUAL",
                "intercept_terminal_trigger": "contact_ttc_terminal",
            },
            {
                "elapsed_s": "1.2",
                "takeover_requested": "1",
                "msp_publish_mode": "algorithm",
                "intercept_phase": "COMPLETE",
                "intercept_terminal_trigger": "contact_bbox_complete",
            },
        ]

        metrics = audit_tool.evaluate_terminal_policy(rows, "contact")

        self.assertFalse(metrics["passed"])

    def test_terminal_audit_accepts_ordered_contact_transitions_during_takeover(self):
        rows = [
            {
                "elapsed_s": "1.0",
                "takeover_requested": "1",
                "msp_publish_mode": "algorithm",
                "intercept_phase": "TRACKING",
                "intercept_terminal_trigger": "",
            },
            {
                "elapsed_s": "1.1",
                "takeover_requested": "1",
                "msp_publish_mode": "algorithm",
                "intercept_phase": "TERMINAL_VISUAL",
                "intercept_terminal_trigger": "contact_ttc_terminal",
            },
            {
                "elapsed_s": "1.2",
                "takeover_requested": "1",
                "msp_publish_mode": "algorithm",
                "intercept_phase": "COMPLETE",
                "intercept_terminal_trigger": "contact_bbox_complete",
            },
        ]

        metrics = audit_tool.evaluate_terminal_policy(rows, "contact")

        self.assertTrue(metrics["passed"])

    def test_terminal_audit_accepts_noncollision_abort_during_takeover(self):
        rows = [
            {
                "elapsed_s": "1.0",
                "takeover_requested": "1",
                "msp_publish_mode": "algorithm",
                "intercept_phase": "TRACKING",
                "intercept_terminal_trigger": "",
            },
            {
                "elapsed_s": "1.1",
                "takeover_requested": "1",
                "msp_publish_mode": "algorithm",
                "intercept_phase": "ABORT",
                "intercept_terminal_trigger": "noncollision_bbox_abort",
            },
        ]

        metrics = audit_tool.evaluate_terminal_policy(rows, "noncollision")

        self.assertTrue(metrics["passed"])

    def test_active_integrity_accepts_bounded_contact_blind_hold(self):
        base = {
            "sp_valid": "1",
            "rc_active": "1",
            "intercept_detection_age_s": "0.38",
            "intercept_detection_update_age_s": "0.32",
            "intercept_phase": "BLIND_HOLD",
            "intercept_blind_age_s": "0.08",
            "intercept_blind_scale": "0.6",
            "sp_roll_rate_deg_s": "1.0",
            "sp_pitch_rate_deg_s": "-1.0",
            "map_requested_throttle_us": "1280",
            "rc_sent_ch1": "1502",
            "rc_sent_ch2": "1498",
            "rc_sent_ch3": "1280",
            "gyro_roll_deg_s": "0.5",
            "gyro_pitch_deg_s": "-0.5",
        }
        guidance = {
            "detection_result_age_limit_s": 0.2,
            "detection_timeout_s": 0.25,
            "blind_hold_s": 0.2,
        }

        invalid, nonfinite = audit_tool._active_command_integrity([base], guidance)

        self.assertEqual(invalid, 0)
        self.assertEqual(nonfinite, set())
        expired = dict(base, intercept_blind_age_s="0.201")
        invalid, _ = audit_tool._active_command_integrity([expired], guidance)
        self.assertEqual(invalid, 1)

    @staticmethod
    def _pose(position, velocity=(0.0, 0.0, 0.0)):
        return GazeboPoseSample(
            received_monotonic_s=1.0,
            simulation_time_s=1.0,
            position_enu_m=np.asarray(position, dtype=float),
            velocity_enu_m_s=np.asarray(velocity, dtype=float),
            orientation_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        )


if __name__ == "__main__":
    unittest.main()
