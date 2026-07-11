import unittest

from vision_guidance.betaflight_msp import (
    BetaflightTelemetry,
    MspAdapterStats,
    StatusTelemetry,
)
from vision_guidance.betaflight_runtime import (
    BetaflightMspIoWorker,
    MspRuntimeConfig,
    ThrottleHandover,
    armed_from_telemetry,
    box_mode_active,
    merge_physical_rc,
    resolve_control_authorization,
)
from vision_guidance.flight_control import RcCommand


class _Adapter:
    timeout_s = 0.1

    def __init__(self):
        self.sent = []
        self.telemetry = BetaflightTelemetry(
            timestamp=1.0,
            status=StatusTelemetry(100, 0, 0, 5, 0),
            rc_channels=(1500, 1500, 1000, 1500, 1800, 1200, 1300, 1400),
        )

    def read_telemetry(self):
        return self.telemetry

    def send_raw_rc(self, channels):
        self.sent.append(tuple(channels))

    def snapshot_stats(self):
        return MspAdapterStats(set_raw_rc_attempt_count=len(self.sent), set_raw_rc_success_count=len(self.sent))


class BetaflightRuntimeTest(unittest.TestCase):
    def test_box_mode_mapping_uses_boxids_order(self):
        box_ids = (0, 27, 50)
        self.assertTrue(box_mode_active(1 << 2, box_ids, 50))
        telemetry = BetaflightTelemetry(timestamp=1.0, status=StatusTelemetry(100, 0, 0, 1, 0))
        self.assertTrue(armed_from_telemetry(telemetry, box_ids))

    def test_authorization_defaults_closed(self):
        status = resolve_control_authorization({}, fc_identity={"fc_variant": "BTFL"}, box_ids=(0, 50))
        self.assertFalse(status.approved)
        self.assertEqual(status.reason, "authorization_disabled")

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
        worker.stage(RcCommand(1.0, (1600, 1400, 1200, 1550, 1000, 2000, 2000, 2000), True), authorized=True)
        worker._publish(1.01)
        self.assertEqual(adapter.sent[0][:4], (1600, 1400, 1200, 1550))
        self.assertEqual(adapter.sent[0][4:], adapter.telemetry.rc_channels[4:])


if __name__ == "__main__":
    unittest.main()
