import hashlib
import json
import struct
import tempfile
import time
import unittest
from dataclasses import replace
from pathlib import Path

from vision_guidance.betaflight_msp import (
    MSP_MOTOR,
    MSP_ALTITUDE,
    MSP_RAW_GPS,
    MSP_RC,
    MSP_SET_RAW_RC,
    AnalogTelemetry,
    AltitudeTelemetry,
    AttitudeTelemetry,
    BetaflightMSPAdapter,
    BetaflightTelemetry,
    MspAdapterStats,
    RawImuTelemetry,
    RawGpsTelemetry,
    StatusTelemetry,
    decode_msp_frame,
    encode_msp_frame,
)
from vision_guidance.betaflight_runtime import (
    BetaflightMspIoWorker,
    MspRawImuGyroConfig,
    MspRuntimeConfig,
    ThrottleHandover,
    armed_from_telemetry,
    bind_msp_raw_imu_gyro,
    box_mode_active,
    merge_physical_rc,
    reorder_msp_rc_to_set_raw_rc,
    resolve_control_authorization,
)
from vision_guidance.flight_control import (
    GuidanceSetpoint,
    RcCommand,
    RcCommandMapper,
    RcMappingConfig,
)


class _Adapter:
    timeout_s = 0.1

    def __init__(self):
        self.sent = []
        self.operations = []
        self.telemetry = BetaflightTelemetry(
            timestamp=1.0,
            status=StatusTelemetry(100, 0, 0, 5, 0),
            # MSP_RC is logical roll, pitch, yaw, throttle, AUX1...
            rc_channels=(1500, 1500, 1500, 1000, 1800, 1200, 1300, 1400),
        )

    def read_telemetry(self):
        return self.telemetry

    def send_raw_rc(self, channels):
        self.operations.append("send")
        self.sent.append(tuple(channels))

    def read_status(self):
        self.operations.append("status")
        return self.telemetry.status

    def read_attitude(self):
        self.operations.append("attitude")
        return AttitudeTelemetry(1.0, 2.0, 3.0)

    def read_raw_imu(self):
        self.operations.append("raw_imu")
        return RawImuTelemetry((1, 2, 3), (4.0, 5.0, 6.0), (7, 8, 9))

    def read_raw_gps(self):
        self.operations.append("raw_gps")
        return RawGpsTelemetry(1, 12, 37.0, -122.0, 20.0, 3.0, 90.0, 80)

    def read_altitude(self):
        self.operations.append("altitude")
        return AltitudeTelemetry(2.0, -0.5)

    def read_motor_outputs(self):
        self.operations.append("motor")
        return (1000, 1010, 1020, 1030, 0, 0, 0, 0)

    def read_rc(self):
        self.operations.append("rc")
        return self.telemetry.rc_channels

    def read_analog(self):
        self.operations.append("analog")
        return AnalogTelemetry(16.0)

    def snapshot_stats(self):
        return MspAdapterStats(set_raw_rc_attempt_count=len(self.sent), set_raw_rc_success_count=len(self.sent))

    def last_set_raw_rc_ack_monotonic_s(self):
        return None

    def last_set_raw_rc_write_monotonic_s(self):
        return None


class _AsyncTransport:
    def __init__(self, *, first_set_delay_s=0.0):
        self.buffer = bytearray()
        self.writes = []
        self.write_times = []
        self.timeout = 0.1
        self.first_set_delay_s = float(first_set_delay_s)
        self._set_count = 0

    @property
    def in_waiting(self):
        return len(self.buffer)

    def write(self, data):
        frame = decode_msp_frame(data)
        if frame.command == MSP_SET_RAW_RC:
            self._set_count += 1
            if self._set_count == 1 and self.first_set_delay_s > 0.0:
                time.sleep(self.first_set_delay_s)
        self.writes.append(bytes(data))
        self.write_times.append(time.monotonic())
        return len(data)

    def read(self, size):
        count = min(int(size), len(self.buffer))
        result = bytes(self.buffer[:count])
        del self.buffer[:count]
        return result

    def inject(self, *frames):
        self.buffer.extend(b"".join(frames))


class BetaflightRuntimeTest(unittest.TestCase):
    @staticmethod
    def _fc_identity():
        return {
            "fc_variant": "BTFL",
            "fc_version_major": 25,
            "fc_version_minor": 12,
            "fc_version_patch": 2,
            "api_major": 1,
            "api_minor": 47,
        }

    def test_raw_imu_gyro_conversion_requires_exact_firmware_binding(self):
        config = MspRawImuGyroConfig.from_mapping(
            {
                "enabled": True,
                "scale_deg_s_per_lsb": 0.0625,
                "axis_order": ["x", "y", "z"],
                "axis_sign": [1, -1, 1],
                "output_frame": "body_frd",
                "expected_fc_variant": "BTFL",
                "expected_fc_version": [25, 12, 2],
                "expected_api_version": [1, 47],
            }
        )
        converter = bind_msp_raw_imu_gyro(config, self._fc_identity())

        self.assertTrue(converter.available)
        self.assertEqual(converter.reason, "firmware_binding_match")
        self.assertEqual(converter.convert((16, -32, 8)), (1.0, 2.0, 0.5))

        mismatched = dict(self._fc_identity(), fc_version_patch=3)
        rejected = bind_msp_raw_imu_gyro(config, mismatched)
        self.assertFalse(rejected.available)
        self.assertEqual(rejected.reason, "firmware_binding_mismatch")
        self.assertIsNone(rejected.convert((16, -32, 8)))

    def test_raw_imu_gyro_conversion_applies_configured_axis_order_and_sign(self):
        config = MspRawImuGyroConfig.from_mapping(
            {
                "enabled": True,
                "axis_order": ["y", "x", "z"],
                "axis_sign": [-1, 1, -1],
                "expected_fc_variant": "BTFL",
                "expected_fc_version": [25, 12, 2],
                "expected_api_version": [1, 47],
            }
        )
        converter = bind_msp_raw_imu_gyro(config, self._fc_identity())

        self.assertEqual(converter.convert((16, 32, -8)), (-2.0, 1.0, 0.5))

    def test_enabled_raw_imu_gyro_requires_polling_and_identity(self):
        values = {
            "enabled": True,
            "expected_fc_variant": "BTFL",
            "expected_fc_version": [25, 12, 2],
            "expected_api_version": [1, 47],
        }
        with self.assertRaisesRegex(ValueError, "explicit FC variant"):
            MspRawImuGyroConfig.from_mapping({"enabled": True})
        with self.assertRaisesRegex(ValueError, "raw_imu_poll_hz"):
            MspRuntimeConfig.from_mapping({"raw_imu_gyro": values})

    def test_runtime_config_records_cli_override_mode_id(self):
        config = MspRuntimeConfig.from_mapping({"override_mode_cli_id": 50})
        self.assertEqual(config.override_mode_cli_id, 50)

        with self.assertRaisesRegex(ValueError, "override_mode_cli_id"):
            MspRuntimeConfig.from_mapping({"override_mode_cli_id": 256})

    def test_per_command_polling_merges_samples_and_tracks_age(self):
        adapter = _Adapter()
        worker = BetaflightMspIoWorker(adapter, MspRuntimeConfig())

        for name in ("status", "attitude", "raw_imu", "motor", "rc", "analog"):
            worker._poll_one(name)
        snapshot = worker.snapshot()

        self.assertEqual(snapshot.telemetry.raw_imu.gyro_msp_raw, (4.0, 5.0, 6.0))
        self.assertEqual(snapshot.telemetry.attitude.roll_deg, 1.0)
        self.assertEqual(snapshot.telemetry.analog.vbat_v, 16.0)
        self.assertEqual(snapshot.telemetry.motor_outputs[:4], (1000, 1010, 1020, 1030))
        self.assertEqual(snapshot.poll_count, 6)
        self.assertIsNotNone(snapshot.status_age_s)
        self.assertIsNotNone(snapshot.raw_imu_age_s)
        self.assertIsNotNone(snapshot.motor_age_s)

    def test_opt_in_gps_altitude_polling_merges_and_tracks_age(self):
        adapter = _Adapter()
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(raw_gps_poll_hz=5.0, altitude_poll_hz=10.0),
        )

        worker._poll_one("raw_gps")
        worker._poll_one("altitude")
        snapshot = worker.snapshot()

        self.assertEqual(snapshot.telemetry.raw_gps.satellites, 12)
        self.assertEqual(snapshot.telemetry.altitude.vertical_speed_m_s, -0.5)
        self.assertIsNotNone(snapshot.raw_gps_age_s)
        self.assertIsNotNone(snapshot.altitude_age_s)

    def test_async_gps_response_is_merged_into_telemetry(self):
        transport = _AsyncTransport()
        adapter = BetaflightMSPAdapter("/dev/null", transport=transport)
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(transport_mode="async_pipeline", raw_gps_poll_hz=5.0),
        )
        adapter.begin_async_pipeline()
        request_id = adapter.queue_async_request(MSP_RAW_GPS)
        transport.inject(
            encode_msp_frame(
                MSP_RAW_GPS,
                struct.pack("<BBiiHHH", 1, 9, 100000000, 200000000, 20, 100, 900),
                direction=">",
            )
        )

        worker._handle_async_responses(adapter.drain_async_responses(10.0))

        self.assertIsNotNone(request_id)
        self.assertEqual(worker.snapshot().telemetry.raw_gps.satellites, 9)
        adapter.end_async_pipeline()

    def test_worker_publish_tick_precedes_telemetry_poll(self):
        adapter = _Adapter()
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(
                control_publish_hz=50.0,
                prefill_enabled=True,
                shutdown_passthrough_frames=0,
            ),
        )
        worker._poll_one("status")
        worker._poll_one("rc")
        worker.stage(None, output_enabled=True, algorithm_authorized=False, override_active=False)
        adapter.operations.clear()

        worker.start()
        time.sleep(0.035)
        worker.close()

        self.assertGreaterEqual(len(adapter.operations), 2)
        self.assertEqual(adapter.operations[0], "send")
        self.assertIn(adapter.operations[1], {"status", "attitude", "rc", "analog"})

    def test_worker_records_send_timing_and_handover_diagnostics(self):
        adapter = _Adapter()
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(prefill_enabled=False, throttle_handover_s=0.4),
        )
        worker._poll(1.0)
        worker.stage(
            RcCommand(timestamp=1.0, channels=(1600, 1400, 1200, 1500, 1000, 1500, 1500, 1500), active=True),
            output_enabled=True,
            algorithm_authorized=True,
            override_active=True,
        )

        worker._publish(1.01)
        worker._publish(1.03)
        snapshot = worker.snapshot(1.03)

        self.assertAlmostEqual(snapshot.publish_tick_interval_s, 0.02)
        self.assertAlmostEqual(snapshot.send_success_interval_s, 0.02)
        self.assertEqual(snapshot.consecutive_send_error_count, 0)
        self.assertEqual(snapshot.throttle_handover.source_us, 1000)
        self.assertEqual(snapshot.throttle_handover.target_us, 1200)
        self.assertGreater(snapshot.throttle_handover.alpha, 0.0)

    def test_box_mode_mapping_uses_boxids_order(self):
        box_ids = (0, 27, 50)
        self.assertTrue(box_mode_active(1 << 2, box_ids, 50))
        telemetry = BetaflightTelemetry(timestamp=1.0, status=StatusTelemetry(100, 0, 0, 1, 0))
        self.assertTrue(armed_from_telemetry(telemetry, box_ids))

    def test_authorization_defaults_closed(self):
        status = resolve_control_authorization({}, fc_identity={"fc_variant": "BTFL"}, box_ids=(0, 50))
        self.assertFalse(status.approved)
        self.assertEqual(status.reason, "authorization_disabled")

    def test_authorization_binds_scope_and_parameters_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            parameters = root / "config.json"
            parameters.write_text('{"profile":"noprop"}\n', encoding="utf-8")
            snapshot = root / "snapshot.json"
            snapshot.write_text(
                json.dumps({"readiness": {"log_only_ready": True}}) + "\n",
                encoding="utf-8",
            )
            approval = root / "approval.json"
            approval.write_text(
                json.dumps(
                    {
                        "approved": True,
                        "scope": "noprop_bench",
                        "source_conflicts_resolved": True,
                        "snapshot_manifest": str(snapshot),
                        "snapshot_sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest(),
                        "expected_fc_identity": {"fc_variant": "BTFL"},
                        "parameters_sha256": hashlib.sha256(parameters.read_bytes()).hexdigest(),
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            values = {
                "enabled": True,
                "required_scope": "noprop_bench",
                "approval_manifest": str(approval),
            }

            status = resolve_control_authorization(
                values,
                fc_identity={"fc_variant": "BTFL"},
                box_ids=(0, 50),
                parameters_path=parameters,
            )
            self.assertTrue(status.approved)
            self.assertEqual(status.scope, "noprop_bench")

            parameters.write_text('{"profile":"changed"}\n', encoding="utf-8")
            mismatch = resolve_control_authorization(
                values,
                fc_identity={"fc_variant": "BTFL"},
                box_ids=(0, 50),
                parameters_path=parameters,
            )
            self.assertFalse(mismatch.approved)
            self.assertEqual(mismatch.reason, "parameters_sha256_mismatch")

            wrong_scope = resolve_control_authorization(
                {**values, "required_scope": "flight"},
                fc_identity={"fc_variant": "BTFL"},
                box_ids=(0, 50),
                parameters_path=parameters,
            )
            self.assertFalse(wrong_scope.approved)
            self.assertEqual(wrong_scope.reason, "authorization_scope_mismatch")

    def test_authorization_binds_required_release_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            parameters = root / "config.json"
            parameters.write_text('{"profile":"supervised"}\n', encoding="utf-8")
            parameters_sha256 = hashlib.sha256(parameters.read_bytes()).hexdigest()
            snapshot = root / "snapshot.json"
            snapshot.write_text(
                json.dumps({"readiness": {"log_only_ready": True}}) + "\n",
                encoding="utf-8",
            )
            evidence = root / "mc100.json"
            evidence.write_text(
                json.dumps(
                    {
                        "release_passed": True,
                        "runtime_binding": {"sha256": parameters_sha256},
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            approval = root / "approval.json"
            approval.write_text(
                json.dumps(
                    {
                        "approved": True,
                        "scope": "flight_active_supervised",
                        "source_conflicts_resolved": True,
                        "snapshot_manifest": str(snapshot),
                        "snapshot_sha256": hashlib.sha256(
                            snapshot.read_bytes()
                        ).hexdigest(),
                        "expected_fc_identity": {"fc_variant": "BTFL"},
                        "parameters_sha256": parameters_sha256,
                        "release_evidence": {
                            "path": str(evidence),
                            "sha256": hashlib.sha256(evidence.read_bytes()).hexdigest(),
                        },
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            values = {
                "enabled": True,
                "required_scope": "flight_active_supervised",
                "approval_manifest": str(approval),
                "release_evidence_required": True,
            }

            status = resolve_control_authorization(
                values,
                fc_identity={"fc_variant": "BTFL"},
                box_ids=(0, 50),
                parameters_path=parameters,
            )
            self.assertTrue(status.approved)

            evidence.write_text('{"release_passed":false}\n', encoding="utf-8")
            mismatch = resolve_control_authorization(
                values,
                fc_identity={"fc_variant": "BTFL"},
                box_ids=(0, 50),
                parameters_path=parameters,
            )
            self.assertFalse(mismatch.approved)
            self.assertEqual(mismatch.reason, "release_evidence_sha256_mismatch")

            evidence.write_text('["not-an-object"]\n', encoding="utf-8")
            approval_data = json.loads(approval.read_text(encoding="utf-8"))
            approval_data["release_evidence"]["sha256"] = hashlib.sha256(
                evidence.read_bytes()
            ).hexdigest()
            approval.write_text(json.dumps(approval_data) + "\n", encoding="utf-8")
            invalid = resolve_control_authorization(
                values,
                fc_identity={"fc_variant": "BTFL"},
                box_ids=(0, 50),
                parameters_path=parameters,
            )
            self.assertFalse(invalid.approved)
            self.assertEqual(invalid.reason, "release_evidence_invalid")

    def test_reorders_msp_logical_rpyt_to_aetr_wire_order(self):
        logical = (1600, 1400, 1550, 1050, 900, 1200, 1800, 2000)

        wire = reorder_msp_rc_to_set_raw_rc(logical, "AETR1234")

        self.assertEqual(wire, (1600, 1400, 1050, 1550, 900, 1200, 1800, 2000))

    def test_merge_preserves_arm_and_aux_channels(self):
        physical = (1500, 1500, 1000, 1500, 1800, 1200, 1300, 1400, 1450, 1550)
        algorithm = (1600, 1400, 1300, 1550, 1000, 2000, 2000, 2000)
        merged = merge_physical_rc(
            physical,
            algorithm,
            override_channels_mask=15,
            aux_arm_channel_zero_based=4,
        )
        self.assertEqual(merged[:4], algorithm[:4])
        self.assertEqual(merged[4:], physical[4:])

    def test_throttle_handover_is_continuous(self):
        handover = ThrottleHandover(0.4)
        handover.reset(1.0, 1100)
        self.assertEqual(handover.apply(1.0, 1500), 1100)
        self.assertEqual(handover.apply(1.2, 1500), 1300)
        self.assertEqual(handover.apply(1.4, 1500), 1500)

    def test_worker_limits_throttle_around_takeover_reference(self):
        adapter = _Adapter()
        adapter.telemetry = replace(
            adapter.telemetry,
            rc_channels=(1500, 1500, 1500, 1275, 1800, 1200, 1300, 1400),
        )
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(
                override_channels_mask=7,
                throttle_handover_s=0.0,
                throttle_relative_limit_us=40,
                throttle_reference_min_us=1200,
                throttle_reference_max_us=1400,
                throttle_command_min_us=1000,
                throttle_command_max_us=1500,
            ),
        )
        worker._poll(1.0)
        worker.stage(
            RcCommand(1.0, (1600, 1400, 1500, 1500, 1000, 2000, 2000, 2000), True),
            authorized=True,
            override_active=True,
        )

        worker._publish(1.01)

        self.assertEqual(adapter.sent[-1][:4], (1600, 1400, 1315, 1500))
        snapshot = worker.snapshot(1.01).throttle_handover
        self.assertEqual(snapshot.source_us, 1275)
        self.assertEqual(snapshot.requested_target_us, 1500)
        self.assertEqual(snapshot.target_us, 1315)
        self.assertEqual(snapshot.lower_limit_us, 1235)
        self.assertEqual(snapshot.upper_limit_us, 1315)
        self.assertTrue(snapshot.target_limited)

    def test_worker_handover_uses_unslewed_mapper_target(self):
        adapter = _Adapter()
        adapter.telemetry = replace(
            adapter.telemetry,
            rc_channels=(1500, 1500, 1500, 1278, 1000, 1000, 2000, 1000),
        )
        mapper = RcCommandMapper(
            RcMappingConfig(
                channel_map="AETR1234",
                throttle_min_us=1000,
                throttle_hover_us=1275,
                throttle_max_us=1500,
                neutral_throttle_us=1000,
                max_delta_us_per_s=100.0,
            )
        )
        mapper.neutral(1.0)
        command = mapper.map_setpoint(
            GuidanceSetpoint(timestamp=1.01, thrust=0.5, source="guidance_eval")
        )
        self.assertLess(command.channels[2], 1100)
        self.assertEqual(command.target_channels[2], 1275)

        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(
                override_channels_mask=15,
                throttle_handover_s=0.8,
                throttle_relative_limit_us=40,
                throttle_reference_min_us=1200,
                throttle_reference_max_us=1400,
                throttle_command_min_us=1000,
                throttle_command_max_us=1500,
            ),
        )
        worker._poll(1.0)
        worker.stage(command, authorized=True, override_active=True)

        outputs = []
        for timestamp in (1.01, 1.21, 1.41, 1.61, 1.81):
            worker._publish(timestamp)
            outputs.append(adapter.sent[-1][2])

        snapshot = worker.snapshot(1.81).throttle_handover
        self.assertEqual(snapshot.source_us, 1278)
        self.assertEqual(snapshot.requested_target_us, 1275)
        self.assertEqual(snapshot.target_us, 1275)
        self.assertFalse(snapshot.target_limited)
        self.assertEqual(outputs[0], 1278)
        self.assertEqual(outputs[-1], 1275)
        self.assertGreaterEqual(min(outputs), 1275)

    def test_worker_applies_dedicated_throttle_slew_after_handover(self):
        adapter = _Adapter()
        adapter.telemetry = replace(
            adapter.telemetry,
            rc_channels=(1500, 1500, 1500, 1275, 1000, 1000, 2000, 1000),
        )
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(
                override_channels_mask=15,
                throttle_handover_s=0.0,
                throttle_slew_limit_us_per_s=600.0,
                throttle_reference_min_us=1200,
                throttle_reference_max_us=1400,
                throttle_command_min_us=1200,
                throttle_command_max_us=1500,
            ),
        )
        worker._poll(1.0)
        worker.stage(
            RcCommand(
                1.0,
                (1500, 1500, 1275, 1500, 1000, 1000, 2000, 1000),
                True,
                target_channels=(1500, 1500, 1275, 1500, 1000, 1000, 2000, 1000),
            ),
            authorized=True,
            override_active=True,
        )
        worker._publish(1.0)
        worker.stage(
            RcCommand(
                1.1,
                (1500, 1500, 1500, 1500, 1000, 1000, 2000, 1000),
                True,
                target_channels=(1500, 1500, 1500, 1500, 1000, 1000, 2000, 1000),
            ),
            authorized=True,
            override_active=True,
        )

        worker._publish(1.1)

        snapshot = worker.snapshot(1.1)
        self.assertEqual(adapter.sent[-1][2], 1335)
        self.assertTrue(snapshot.throttle_slew_limited)
        self.assertEqual(snapshot.throttle_slew_output_us, 1335)

    def test_worker_rejects_algorithm_for_out_of_range_throttle_reference(self):
        adapter = _Adapter()
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(
                override_channels_mask=7,
                throttle_relative_limit_us=40,
                throttle_reference_min_us=1200,
                throttle_reference_max_us=1400,
                throttle_command_min_us=1000,
                throttle_command_max_us=1500,
                prefill_enabled=True,
                prefill_min_frames=1,
            ),
        )
        worker._poll(1.0)
        worker.stage(None, output_enabled=True, algorithm_authorized=False, override_active=False)
        worker._publish(1.001)
        worker.stage(
            RcCommand(1.0, (1600, 1400, 1275, 1500, 1000, 2000, 2000, 2000), True),
            output_enabled=True,
            algorithm_authorized=True,
            override_active=True,
        )

        worker._publish(1.01)

        self.assertEqual(worker.snapshot(1.01).publish_mode, "throttle_reference_out_of_range")
        self.assertEqual(adapter.sent[-1][:4], (1500, 1500, 1000, 1500))

    def test_worker_never_sends_without_authorization(self):
        adapter = _Adapter()
        worker = BetaflightMspIoWorker(adapter, MspRuntimeConfig())
        worker._poll(1.0)
        worker.stage(RcCommand(1.0, (1600, 1400, 1200, 1500, 1000, 1500, 1500, 1500), True), authorized=False)
        worker._publish(1.01)
        self.assertEqual(adapter.sent, [])
        self.assertEqual(worker.snapshot(1.01).send_skip_count, 1)

    def test_worker_preserves_aux_when_authorized(self):
        adapter = _Adapter()
        worker = BetaflightMspIoWorker(adapter, MspRuntimeConfig(throttle_handover_s=0.0))
        worker._poll(1.0)
        worker.stage(
            RcCommand(1.0, (1600, 1400, 1200, 1550, 1000, 2000, 2000, 2000), True),
            authorized=True,
            override_active=True,
        )
        worker._publish(1.01)
        self.assertEqual(adapter.sent[0][:4], (1600, 1400, 1200, 1550))
        self.assertEqual(adapter.sent[0][4:], adapter.telemetry.rc_channels[4:])
        snapshot = worker.snapshot(1.01)
        self.assertTrue(snapshot.last_publish_algorithm_authorized)
        self.assertTrue(snapshot.last_publish_override_active)
        self.assertTrue(snapshot.last_publish_physical_rc_fresh)

    def test_worker_refuses_algorithm_when_override_is_inactive(self):
        adapter = _Adapter()
        worker = BetaflightMspIoWorker(adapter, MspRuntimeConfig(prefill_enabled=True, prefill_min_frames=1))
        worker._poll(1.0)
        command = RcCommand(1.0, (1600, 1400, 1200, 1550, 1000, 2000, 2000, 2000), True)
        worker.stage(command, output_enabled=True, algorithm_authorized=False, override_active=False)
        worker._publish(1.01)
        worker.stage(command, output_enabled=True, algorithm_authorized=True, override_active=False)

        worker._publish(1.02)

        self.assertEqual(worker.snapshot(1.02).publish_mode, "passthrough")
        self.assertEqual(adapter.sent[-1][:4], (1500, 1500, 1000, 1500))

    def test_worker_refuses_inactive_command_at_publish_boundary(self):
        adapter = _Adapter()
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(prefill_enabled=True, prefill_min_frames=1),
        )
        worker._poll(1.0)
        command = RcCommand(
            1.0,
            (1600, 1400, 1200, 1550, 1000, 2000, 2000, 2000),
            False,
            reason="guidance_missing",
        )
        worker.stage(command, output_enabled=True, algorithm_authorized=True, override_active=True)

        worker._publish(1.01)

        snapshot = worker.snapshot(1.01)
        self.assertNotEqual(snapshot.publish_mode, "algorithm")
        self.assertEqual(adapter.sent[-1][:4], (1500, 1500, 1000, 1500))
        self.assertFalse(snapshot.last_publish_command_active)
        self.assertEqual(snapshot.last_publish_command_reason, "guidance_missing")

    def test_override_release_grace_continues_latched_manual_frames(self):
        adapter = _Adapter()
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(
                prefill_enabled=True,
                prefill_min_frames=1,
                throttle_handover_s=0.0,
                override_grace_hold_s=0.35,
            ),
        )
        worker._poll(1.0)
        now = time.monotonic()
        command = RcCommand(
            now,
            (1600, 1400, 1200, 1550, 1000, 2000, 2000, 2000),
            True,
        )
        worker.stage(command, output_enabled=True, algorithm_authorized=True, override_active=True)
        worker._publish(now)
        worker.stage(command, output_enabled=True, algorithm_authorized=False, override_active=False)

        worker._publish(now + 0.02)

        snapshot = worker.snapshot(now + 0.02)
        self.assertTrue(snapshot.override_release_hold_active)
        self.assertTrue(snapshot.last_publish_override_release_hold_active)
        self.assertEqual(snapshot.publish_mode, "passthrough")
        self.assertEqual(adapter.sent[-1][:4], (1500, 1500, 1000, 1500))

        sent_count = len(adapter.sent)
        worker._publish(now + 0.40)
        self.assertEqual(len(adapter.sent), sent_count)
        self.assertEqual(worker.snapshot(now + 0.40).publish_mode, "physical_rc_stale")

    def test_async_motor_response_is_merged_into_telemetry(self):
        transport = _AsyncTransport()
        adapter = BetaflightMSPAdapter("/dev/null", transport=transport)
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(transport_mode="async_pipeline", motor_poll_hz=2.0),
        )
        adapter.begin_async_pipeline()
        request_id = adapter.queue_async_request(MSP_MOTOR)
        transport.inject(
            encode_msp_frame(MSP_MOTOR, b"\xe8\x03\xf2\x03\xfc\x03\x06\x04", direction=">")
        )

        worker._handle_async_responses(adapter.drain_async_responses(10.0))

        snapshot = worker.snapshot()
        self.assertIsNotNone(request_id)
        self.assertEqual(snapshot.telemetry.motor_outputs, (1000, 1010, 1020, 1030))
        self.assertIsNotNone(snapshot.motor_age_s)
        adapter.end_async_pipeline()

    def test_worker_prefills_physical_rc_before_algorithm_authorization(self):
        adapter = _Adapter()
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(prefill_enabled=True, prefill_min_frames=2, shutdown_passthrough_frames=0),
        )
        worker._poll(1.0)
        command = RcCommand(1.0, (1600, 1400, 1200, 1550, 1000, 2000, 2000, 2000), True)
        worker.stage(command, output_enabled=True, algorithm_authorized=False, override_active=False)

        worker._publish(1.01)
        self.assertEqual(adapter.sent[-1], (1500, 1500, 1000, 1500, 1800, 1200, 1300, 1400))
        self.assertFalse(worker.snapshot(1.01).prefill_ready)
        worker._publish(1.02)

        snapshot = worker.snapshot(1.02)
        self.assertTrue(snapshot.prefill_ready)
        self.assertEqual(snapshot.passthrough_send_count, 2)
        self.assertEqual(snapshot.algorithm_send_count, 0)

    def test_worker_enters_algorithm_only_after_prefill_and_handover(self):
        adapter = _Adapter()
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(
                prefill_enabled=True,
                prefill_min_frames=1,
                throttle_handover_s=0.4,
                shutdown_passthrough_frames=0,
            ),
        )
        worker._poll(1.0)
        command = RcCommand(1.0, (1600, 1400, 1500, 1550, 1000, 2000, 2000, 2000), True)
        worker.stage(command, output_enabled=True, algorithm_authorized=False, override_active=False)
        worker._publish(1.01)
        worker.stage(command, output_enabled=True, algorithm_authorized=True, override_active=True)

        worker._publish(1.02)
        worker._publish(1.22)

        self.assertEqual(adapter.sent[-2][:2], (1600, 1400))
        self.assertEqual(adapter.sent[-2][2], 1000)
        self.assertEqual(adapter.sent[-1][2], 1250)
        self.assertEqual(adapter.sent[-1][4:], adapter.telemetry.rc_channels[4:])
        self.assertEqual(worker.snapshot(1.22).algorithm_send_count, 2)

    def test_stale_algorithm_command_falls_back_to_latched_manual_rc(self):
        adapter = _Adapter()
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(
                prefill_enabled=True,
                prefill_min_frames=1,
                staged_command_timeout_s=0.1,
                shutdown_passthrough_frames=0,
            ),
        )
        worker._poll(1.0)
        command = RcCommand(1.0, (1600, 1400, 1500, 1550, 1000, 2000, 2000, 2000), True)
        worker.stage(command, output_enabled=True, algorithm_authorized=False, override_active=False)
        worker._publish(1.01)
        worker.stage(command, output_enabled=True, algorithm_authorized=True, override_active=True)
        with worker._lock:
            worker._staged_received_s = 1.0

        worker._publish(1.2)

        snapshot = worker.snapshot(1.2)
        self.assertEqual(adapter.sent[-1], (1500, 1500, 1000, 1500, 1800, 1200, 1300, 1400))
        self.assertEqual(snapshot.publish_mode, "passthrough")
        self.assertEqual(snapshot.stale_command_count, 1)

    def test_worker_refuses_prefill_when_started_with_override_active_and_885_rc(self):
        adapter = _Adapter()
        adapter.telemetry = BetaflightTelemetry(
            timestamp=1.0,
            status=StatusTelemetry(100, 0, 0, 0b11, 0),
            rc_channels=(885, 885, 885, 885, 1800, 1500, 1800, 1500),
        )
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(prefill_enabled=True, prefill_min_frames=1, shutdown_passthrough_frames=0),
            box_ids=(0, 50),
        )
        worker._poll(1.0)
        worker.stage(None, output_enabled=True, algorithm_authorized=False, override_active=True)

        worker._publish(1.01)

        snapshot = worker.snapshot(1.01)
        self.assertEqual(adapter.sent, [])
        self.assertFalse(snapshot.prefill_ready)
        self.assertEqual(snapshot.publish_mode, "physical_rc_invalid")

    def test_worker_normal_close_sends_configured_passthrough_frames(self):
        adapter = _Adapter()
        now = time.monotonic()
        adapter.telemetry = BetaflightTelemetry(
            timestamp=now,
            status=StatusTelemetry(100, 0, 0, 0, 0),
            rc_channels=(1500, 1500, 1500, 1000, 1800, 1200, 1300, 1400),
        )
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(prefill_enabled=True, prefill_min_frames=1, shutdown_passthrough_frames=3),
        )
        worker._poll(now)
        worker.stage(None, output_enabled=True, algorithm_authorized=False, override_active=False)
        worker._publish(time.monotonic())
        count_before_close = len(adapter.sent)

        worker.close()

        self.assertEqual(len(adapter.sent), count_before_close + 3)
        self.assertEqual(adapter.sent[-1][:4], (1500, 1500, 1000, 1500))

    def test_worker_send_error_resets_consecutive_prefill(self):
        class FailingAdapter(_Adapter):
            fail = False

            def send_raw_rc(self, channels):
                if self.fail:
                    raise OSError("serial write failed")
                super().send_raw_rc(channels)

        adapter = FailingAdapter()
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(prefill_enabled=True, prefill_min_frames=2, shutdown_passthrough_frames=0),
        )
        worker._poll(1.0)
        worker.stage(None, output_enabled=True, algorithm_authorized=False, override_active=False)
        worker._publish(1.01)
        self.assertEqual(worker.snapshot(1.01).prefill_success_count, 1)
        adapter.fail = True

        worker._publish(1.02)

        snapshot = worker.snapshot(1.02)
        self.assertEqual(snapshot.prefill_success_count, 0)
        self.assertFalse(snapshot.prefill_ready)
        self.assertEqual(snapshot.send_error_count, 1)

    def test_async_prefill_counts_acknowledged_passthrough_frames_only(self):
        transport = _AsyncTransport()
        adapter = BetaflightMSPAdapter("/dev/null", transport=transport)
        config = MspRuntimeConfig(
            transport_mode="async_pipeline",
            prefill_enabled=True,
            prefill_min_frames=1,
            shutdown_passthrough_frames=0,
        )
        worker = BetaflightMspIoWorker(adapter, config, box_ids=(0, 50))
        adapter.begin_async_pipeline()
        now = time.monotonic()
        worker._merge_poll_value("rc", (1500, 1500, 1500, 1000, 1800, 1200, 1300, 1400), now)
        worker._merge_poll_value("status", StatusTelemetry(100, 0, 0, 0, 0), now)
        worker.stage(None, output_enabled=True, algorithm_authorized=False, override_active=False)

        worker._publish(time.monotonic())
        self.assertFalse(worker.snapshot().prefill_ready)
        transport.inject(encode_msp_frame(MSP_SET_RAW_RC, direction=">"))
        worker._handle_async_responses(adapter.drain_async_responses(1.0))

        snapshot = worker.snapshot()
        self.assertTrue(snapshot.prefill_ready)
        self.assertTrue(snapshot.set_raw_rc_ack_fresh)
        self.assertEqual(snapshot.adapter_stats.set_raw_rc_write_success_count, 1)
        self.assertEqual(snapshot.adapter_stats.set_raw_rc_ack_count, 1)
        adapter.end_async_pipeline()

    def test_async_stale_set_ack_falls_back_to_passthrough(self):
        transport = _AsyncTransport()
        adapter = BetaflightMSPAdapter("/dev/null", transport=transport)
        config = MspRuntimeConfig(
            transport_mode="async_pipeline",
            prefill_enabled=True,
            prefill_min_frames=1,
            response_stale_s=0.001,
            shutdown_passthrough_frames=0,
        )
        worker = BetaflightMspIoWorker(adapter, config, box_ids=(0, 50))
        adapter.begin_async_pipeline()
        now = time.monotonic()
        worker._merge_poll_value("rc", (1500, 1500, 1500, 1000, 1800, 1200, 1300, 1400), now)
        worker._merge_poll_value("status", StatusTelemetry(100, 0, 0, 0, 0), now)
        worker.stage(None, output_enabled=True, algorithm_authorized=False, override_active=False)
        worker._publish(time.monotonic())
        transport.inject(encode_msp_frame(MSP_SET_RAW_RC, direction=">"))
        worker._handle_async_responses(adapter.drain_async_responses(1.0))
        time.sleep(0.003)
        worker._merge_poll_value("status", StatusTelemetry(100, 0, 0, 1 << 1, 0), time.monotonic())
        command = RcCommand(
            time.monotonic(),
            (1600, 1400, 1050, 1550, 1000, 2000, 2000, 2000),
            True,
        )
        worker.stage(command, output_enabled=True, algorithm_authorized=True, override_active=True)

        worker._publish(time.monotonic())

        snapshot = worker.snapshot()
        self.assertEqual(snapshot.publish_mode, "set_ack_stale")
        self.assertFalse(snapshot.set_raw_rc_ack_fresh)
        self.assertEqual(snapshot.last_sent_channels[:4], (1500, 1500, 1000, 1500))
        adapter.end_async_pipeline()

    def test_async_suspends_rc_poll_while_override_uses_latched_manual_rc(self):
        transport = _AsyncTransport()
        adapter = BetaflightMSPAdapter("/dev/null", transport=transport)
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(transport_mode="async_pipeline"),
            box_ids=(0, 50),
        )
        adapter.begin_async_pipeline()
        now = time.monotonic()
        worker._merge_poll_value("rc", (1500, 1500, 1500, 1000, 1800, 1200, 1800, 1400), now)
        worker._merge_poll_value("status", StatusTelemetry(100, 0, 0, 1 << 1, 0), now)
        worker._next_poll_s = {"rc": 0.0}

        worker._queue_one_async_poll(time.monotonic())
        self.assertEqual(transport.writes, [])
        self.assertTrue(worker.snapshot().rc_poll_suspended)

        worker._merge_poll_value("status", StatusTelemetry(100, 0, 0, 0, 0), time.monotonic())
        worker._next_poll_s["rc"] = 0.0
        worker._queue_one_async_poll(time.monotonic())
        self.assertEqual(decode_msp_frame(transport.writes[-1]).command, MSP_RC)
        adapter.end_async_pipeline()

    def test_async_cycle_writes_set_before_due_telemetry_poll(self):
        transport = _AsyncTransport()
        adapter = BetaflightMSPAdapter("/dev/null", transport=transport)
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(
                transport_mode="async_pipeline",
                prefill_enabled=True,
                shutdown_passthrough_frames=0,
            ),
        )
        adapter.begin_async_pipeline()
        now = time.monotonic()
        worker._merge_poll_value("rc", (1500, 1500, 1500, 1000, 1800, 1200, 1300, 1400), now)
        worker.stage(None, output_enabled=True, algorithm_authorized=False, override_active=False)
        worker._next_poll_s = {"status": 0.0}

        worker._publish(time.monotonic())
        worker._queue_one_async_poll(time.monotonic())

        commands = [decode_msp_frame(frame).command for frame in transport.writes]
        self.assertEqual(commands[0], MSP_SET_RAW_RC)
        self.assertNotEqual(commands[1], MSP_SET_RAW_RC)
        adapter.end_async_pipeline()

    def test_async_worker_skips_missed_publish_periods_without_bursting(self):
        transport = _AsyncTransport(first_set_delay_s=0.05)
        adapter = BetaflightMSPAdapter("/dev/null", transport=transport)
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(
                transport_mode="async_pipeline",
                control_publish_hz=50.0,
                prefill_enabled=True,
                shutdown_passthrough_frames=0,
                response_drain_budget_ms=0.0,
            ),
        )
        now = time.monotonic()
        worker._merge_poll_value("rc", (1500, 1500, 1500, 1000, 1800, 1200, 1300, 1400), now)
        worker.stage(None, output_enabled=True, algorithm_authorized=False, override_active=False)

        worker.start()
        time.sleep(0.15)
        worker.close()

        set_times = [
            timestamp
            for frame, timestamp in zip(transport.writes, transport.write_times)
            if decode_msp_frame(frame).command == MSP_SET_RAW_RC
        ]
        self.assertGreaterEqual(len(set_times), 3)
        self.assertGreaterEqual(min(b - a for a, b in zip(set_times, set_times[1:])), 0.015)

    def test_publish_hot_path_does_not_build_full_adapter_statistics(self):
        class HotPathAdapter(_Adapter):
            def __init__(self):
                super().__init__()
                self.snapshot_stats_calls = 0

            def snapshot_stats(self):
                self.snapshot_stats_calls += 1
                return super().snapshot_stats()

        adapter = HotPathAdapter()
        worker = BetaflightMspIoWorker(
            adapter,
            MspRuntimeConfig(prefill_enabled=True, shutdown_passthrough_frames=0),
        )
        worker._poll(1.0)
        worker.stage(None, output_enabled=True, algorithm_authorized=False, override_active=False)

        worker._publish(1.01)

        self.assertEqual(adapter.sent[-1][:4], (1500, 1500, 1000, 1500))
        self.assertEqual(adapter.snapshot_stats_calls, 0)


if __name__ == "__main__":
    unittest.main()
