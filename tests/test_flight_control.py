import unittest

import numpy as np

from vision_guidance.flight_control import (
    BetaflightSafetyStateMachine,
    CommandWatchdog,
    GuidanceSetpoint,
    RcCommandMapper,
    RcMappingConfig,
    SafetyInputs,
    SafetyState,
    aux_range_enabled,
    guidance_eval_to_setpoint,
)
from vision_guidance.types import GuidanceEval


class FlightControlTest(unittest.TestCase):
    def test_rc_mapper_maps_rates_and_thrust_to_aetr_channels(self):
        mapper = RcCommandMapper(
            RcMappingConfig(
                channel_map="AETR1234",
                roll_rate_limit_deg_s=120.0,
                pitch_rate_limit_deg_s=120.0,
                yaw_rate_limit_deg_s=90.0,
            )
        )
        setpoint = GuidanceSetpoint(
            timestamp=1.0,
            roll_rate_deg_s=60.0,
            pitch_rate_deg_s=-120.0,
            yaw_rate_deg_s=90.0,
            thrust=0.75,
            source="test",
        )

        command = mapper.map_setpoint(setpoint)

        self.assertTrue(command.active)
        self.assertEqual(command.channels[:4], (1750, 1000, 1750, 2000))
        self.assertEqual(command.reason, "test")

    def test_rc_mapper_rejects_nonfinite_setpoint_to_neutral(self):
        mapper = RcCommandMapper(RcMappingConfig())
        command = mapper.map_setpoint(
            GuidanceSetpoint(
                timestamp=1.0,
                roll_rate_deg_s=float("nan"),
                pitch_rate_deg_s=0.0,
                yaw_rate_deg_s=0.0,
                thrust=0.5,
            )
        )

        self.assertFalse(command.active)
        self.assertEqual(command.reason, "setpoint_nonfinite")
        self.assertEqual(command.channels[:4], (1500, 1500, 1000, 1500))

    def test_rc_mapper_applies_slew_limit(self):
        mapper = RcCommandMapper(RcMappingConfig(max_delta_us_per_s=100.0))
        mapper.neutral(0.0)

        command = mapper.map_setpoint(
            GuidanceSetpoint(
                timestamp=1.0,
                roll_rate_deg_s=120.0,
                pitch_rate_deg_s=0.0,
                yaw_rate_deg_s=0.0,
                thrust=1.0,
            )
        )

        self.assertEqual(command.channels[0], 1600)
        self.assertEqual(command.channels[2], 1100)
        self.assertEqual(command.raw_channels[0], 2000)
        self.assertEqual(command.raw_channels[2], 2000)
        self.assertEqual(command.clipped_flags[0], 0)
        self.assertEqual(command.slew_limited_flags[0], 1)
        self.assertEqual(command.slew_limited_flags[2], 1)

    def test_rc_mapper_logs_clipped_flags_for_out_of_range_commands(self):
        mapper = RcCommandMapper(RcMappingConfig())

        command = mapper.map_setpoint(
            GuidanceSetpoint(
                timestamp=1.0,
                roll_rate_deg_s=240.0,
                pitch_rate_deg_s=-300.0,
                yaw_rate_deg_s=0.0,
                thrust=1.5,
            )
        )

        self.assertEqual(command.raw_channels[0], 2500)
        self.assertEqual(command.raw_channels[1], 250)
        self.assertEqual(command.raw_channels[2], 2500)
        self.assertEqual(command.channels[:4], (2000, 1000, 2000, 1500))
        self.assertEqual(command.clipped_flags[:4], (1, 1, 1, 0))
        self.assertEqual(command.slew_limited_flags[:4], (0, 0, 0, 0))

    def test_safety_state_machine_gates_control(self):
        safety = BetaflightSafetyStateMachine()

        decision = safety.update(SafetyInputs(control_requested=False))
        self.assertEqual(decision.state, SafetyState.LOG_ONLY)
        self.assertFalse(decision.command_active)

        decision = safety.update(
            SafetyInputs(
                control_requested=True,
                allow_control=True,
                telemetry_fresh=True,
                attitude_synced=True,
                voltage_ok=True,
                watchdog_ok=True,
                armed=True,
                override_available=True,
                override_active=True,
                physical_rc_fresh=True,
                snapshot_approved=True,
                config_conflict_free=True,
                aux_enabled=False,
                target_valid=True,
            )
        )
        self.assertEqual(decision.state, SafetyState.READY)
        self.assertFalse(decision.command_active)

        decision = safety.update(
            SafetyInputs(
                control_requested=True,
                allow_control=True,
                telemetry_fresh=True,
                attitude_synced=True,
                voltage_ok=True,
                watchdog_ok=True,
                armed=True,
                override_available=True,
                override_active=True,
                physical_rc_fresh=True,
                snapshot_approved=True,
                config_conflict_free=True,
                aux_enabled=True,
                target_valid=True,
            )
        )
        self.assertEqual(decision.state, SafetyState.ACTIVE)
        self.assertTrue(decision.command_active)

    def test_watchdog_freshness(self):
        watchdog = CommandWatchdog(timeout_s=0.5)
        self.assertFalse(watchdog.fresh(1.0))
        self.assertIsNone(watchdog.age_s(1.0))
        watchdog.kick(1.0)
        self.assertTrue(watchdog.fresh(1.4))
        self.assertAlmostEqual(watchdog.age_s(1.4), 0.4)
        self.assertFalse(watchdog.fresh(1.6))
        self.assertAlmostEqual(watchdog.age_s(1.6), 0.6)

    def test_control_authorization_gates_fail_closed(self):
        base = {
            "control_requested": True,
            "allow_control": True,
            "target_valid": True,
            "aux_enabled": True,
            "telemetry_fresh": True,
            "attitude_synced": True,
            "voltage_ok": True,
            "watchdog_ok": True,
            "armed": True,
            "override_available": True,
            "override_active": True,
            "physical_rc_fresh": True,
            "snapshot_approved": True,
            "config_conflict_free": True,
        }
        cases = (
            ("snapshot_approved", False, "snapshot_not_approved"),
            ("config_conflict_free", False, "config_conflict"),
            ("override_available", False, "msp_override_unavailable"),
            ("override_active", False, "msp_override_inactive"),
            ("armed", False, "not_armed"),
            ("physical_rc_fresh", False, "physical_rc_stale"),
        )
        for field, value, reason in cases:
            with self.subTest(field=field):
                inputs = dict(base)
                inputs[field] = value
                decision = BetaflightSafetyStateMachine().update(SafetyInputs(**inputs))
                self.assertFalse(decision.command_active)
                self.assertEqual(decision.reason, reason)

    def test_guidance_eval_to_setpoint_uses_gain_matrix(self):
        guidance = GuidanceEval(
            timestamp=2.0,
            g_eval=np.array([1.0, 2.0, 3.0]),
            valid=True,
            quality=1.0,
        )

        setpoint = guidance_eval_to_setpoint(
            guidance,
            rate_gain_matrix=[[10.0, 0.0, 0.0], [0.0, -5.0, 0.0], [0.0, 0.0, 2.0]],
            hover_thrust=0.55,
        )

        self.assertTrue(setpoint.valid)
        self.assertEqual(setpoint.roll_rate_deg_s, 10.0)
        self.assertEqual(setpoint.pitch_rate_deg_s, -10.0)
        self.assertEqual(setpoint.yaw_rate_deg_s, 6.0)
        self.assertEqual(setpoint.thrust, 0.55)

    def test_aux_range_enabled_uses_one_based_channel_index(self):
        self.assertTrue(aux_range_enabled((1500, 1500, 1500, 1500, 1800), channel_index=5, min_us=1700, max_us=2100))
        self.assertFalse(aux_range_enabled((1500, 1500, 1500, 1500, 1600), channel_index=5, min_us=1700, max_us=2100))


if __name__ == "__main__":
    unittest.main()
