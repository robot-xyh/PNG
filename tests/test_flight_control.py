import unittest

import numpy as np

from vision_guidance.flight_control import (
    BetaflightSafetyStateMachine,
    CommandWatchdog,
    EntryHandoffConfig,
    GuidanceCommandShaper,
    GuidanceCommandShaperConfig,
    GuidanceSetpoint,
    GuidanceSetpointHold,
    RcCommandMapper,
    RcMappingConfig,
    SafetyInputs,
    SafetyState,
    TiltEnvelopeConfig,
    aux_range_enabled,
    guidance_eval_to_setpoint,
)
from vision_guidance.types import GuidanceEval


class FlightControlTest(unittest.TestCase):
    @staticmethod
    def _shape(
        shaper,
        setpoint,
        *,
        timestamp=1.0,
        gate_open=True,
        attitude_deg=(0.0, 0.0),
        gyro_deg_s=(0.0, 0.0),
        gyro_age_s=0.0,
    ):
        return shaper.update(
            setpoint,
            timestamp=timestamp,
            gate_open=gate_open,
            attitude_deg=attitude_deg,
            gyro_deg_s=gyro_deg_s,
            gyro_age_s=gyro_age_s,
        )

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

    def test_betaflight_rate_inverse_matches_100_0_70_profile(self):
        mapper = RcCommandMapper(
            RcMappingConfig(
                rate_mapping_type="betaflight",
                betaflight_rc_rate=(1.0, 1.0, 1.0),
                betaflight_super_rate=(0.70, 0.70, 0.70),
                betaflight_expo=(0.0, 0.0, 0.0),
            )
        )

        command = mapper.map_setpoint(
            GuidanceSetpoint(timestamp=1.0, roll_rate_deg_s=120.0, pitch_rate_deg_s=-120.0, thrust=0.5)
        )

        self.assertEqual(command.channels[0], 1711)
        self.assertEqual(command.channels[1], 1289)
        full_stick = mapper.map_setpoint(
            GuidanceSetpoint(timestamp=2.0, roll_rate_deg_s=200.0 / 0.3, thrust=0.5)
        )
        self.assertEqual(full_stick.channels[0], 2000)

    def test_betaflight_mapping_applies_independent_noprop_rate_limit(self):
        mapper = RcCommandMapper(
            RcMappingConfig(
                rate_mapping_type="betaflight",
                betaflight_rc_rate=(1.0, 1.0, 1.0),
                betaflight_super_rate=(0.70, 0.70, 0.70),
                betaflight_expo=(0.0, 0.0, 0.0),
                roll_command_limit_deg_s=3.0,
                pitch_command_limit_deg_s=3.0,
                yaw_command_limit_deg_s=0.0,
            )
        )

        command = mapper.map_setpoint(
            GuidanceSetpoint(
                timestamp=1.0,
                roll_rate_deg_s=100.0,
                pitch_rate_deg_s=-100.0,
                yaw_rate_deg_s=50.0,
                thrust=0.5,
            )
        )

        self.assertEqual(command.channels[:4], (1507, 1493, 1500, 1500))
        self.assertEqual(command.clipped_flags[:4], (1, 1, 0, 1))
        self.assertEqual(command.requested_rates_deg_s, (100.0, -100.0, 50.0))
        self.assertEqual(command.limited_rates_deg_s, (3.0, -3.0, 0.0))
        self.assertAlmostEqual(command.stick_deflections[0], 0.0148, places=3)
        self.assertEqual(command.target_channels[:4], (1507, 1493, 1500, 1500))
        self.assertEqual(command.requested_thrust, 0.5)
        self.assertEqual(command.limited_thrust, 0.5)

    def test_thrust_mapping_hard_limits_noprop_pwm_envelope(self):
        mapper = RcCommandMapper(
            RcMappingConfig(
                thrust_min=0.0,
                thrust_hover=0.078,
                thrust_max=0.10,
                throttle_min_us=1000,
                throttle_hover_us=1078,
                throttle_max_us=1100,
                neutral_throttle_us=1000,
            )
        )

        hover = mapper.map_setpoint(GuidanceSetpoint(timestamp=1.0, thrust=0.078))
        excessive = mapper.map_setpoint(GuidanceSetpoint(timestamp=2.0, thrust=0.50))

        self.assertEqual(hover.channels[2], 1078)
        self.assertEqual(excessive.channels[2], 1100)
        self.assertEqual(excessive.clipped_flags[2], 1)

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

    def test_setpoint_hold_only_bridges_explicit_perception_gaps(self):
        hold = GuidanceSetpointHold(timeout_s=0.25)
        valid = GuidanceSetpoint(
            timestamp=1.0,
            roll_rate_deg_s=2.0,
            thrust=0.078,
            source="guidance_eval",
        )
        missing = GuidanceSetpoint(
            timestamp=1.05,
            valid=False,
            source="guidance_eval",
            reject_reason="guidance_missing",
        )

        self.assertIs(hold.update(valid, timestamp=1.0, allow_hold=False, gate_open=True), valid)
        bridged = hold.update(missing, timestamp=1.05, allow_hold=True, gate_open=True)
        self.assertTrue(bridged.valid)
        self.assertEqual(bridged.source, "guidance_hold")
        self.assertEqual(bridged.roll_rate_deg_s, 2.0)

        rejected = hold.update(missing, timestamp=1.10, allow_hold=False, gate_open=True)
        self.assertFalse(rejected.valid)
        self.assertFalse(hold.update(missing, timestamp=1.15, allow_hold=True, gate_open=True).valid)

    def test_setpoint_hold_expires_and_resets_when_gate_closes(self):
        hold = GuidanceSetpointHold(timeout_s=0.25)
        valid = GuidanceSetpoint(timestamp=1.0, pitch_rate_deg_s=-1.0, source="guidance_eval")
        missing = GuidanceSetpoint(timestamp=1.1, valid=False, reject_reason="guidance_missing")

        hold.update(valid, timestamp=1.0, allow_hold=False, gate_open=True)
        self.assertFalse(hold.update(missing, timestamp=1.30, allow_hold=True, gate_open=True).valid)
        hold.update(valid, timestamp=2.0, allow_hold=False, gate_open=True)
        self.assertFalse(hold.update(missing, timestamp=2.05, allow_hold=True, gate_open=False).valid)
        self.assertFalse(hold.update(missing, timestamp=2.10, allow_hold=True, gate_open=True).valid)

    def test_command_shaper_entry_handoff_uses_fresh_gyro_and_smoothstep(self):
        shaper = GuidanceCommandShaper(
            GuidanceCommandShaperConfig(
                entry_handoff=EntryHandoffConfig(
                    enabled=True,
                    duration_s=0.8,
                    gyro_max_age_s=0.25,
                    rate_source="gyro",
                )
            )
        )
        target = GuidanceSetpoint(
            timestamp=1.0,
            roll_rate_deg_s=10.0,
            pitch_rate_deg_s=20.0,
            yaw_rate_deg_s=5.0,
            thrust=0.078,
            source="guidance_eval",
        )

        start, start_diag = self._shape(
            shaper,
            target,
            timestamp=1.0,
            gyro_deg_s=(2.0, -4.0),
            gyro_age_s=0.1,
        )
        middle, middle_diag = self._shape(shaper, target, timestamp=1.4)
        end, end_diag = self._shape(shaper, target, timestamp=1.8)

        self.assertEqual((start.roll_rate_deg_s, start.pitch_rate_deg_s), (2.0, -4.0))
        self.assertEqual(start_diag.entry_source, "gyro")
        self.assertTrue(start_diag.entry_active)
        self.assertAlmostEqual(middle_diag.entry_progress, 0.5)
        self.assertAlmostEqual(middle.roll_rate_deg_s, 6.0)
        self.assertAlmostEqual(middle.pitch_rate_deg_s, 8.0)
        self.assertEqual(end.roll_rate_deg_s, target.roll_rate_deg_s)
        self.assertEqual(end.pitch_rate_deg_s, target.pitch_rate_deg_s)
        self.assertFalse(end_diag.entry_active)
        self.assertEqual(end.yaw_rate_deg_s, target.yaw_rate_deg_s)
        self.assertEqual(end.thrust, target.thrust)
        self.assertEqual(end.source, target.source)

    def test_command_shaper_entry_uses_zero_for_stale_gyro_and_resets_on_gate_close(self):
        shaper = GuidanceCommandShaper(
            GuidanceCommandShaperConfig(
                entry_handoff=EntryHandoffConfig(enabled=True, rate_source="gyro")
            )
        )
        target = GuidanceSetpoint(timestamp=1.0, roll_rate_deg_s=10.0, pitch_rate_deg_s=-5.0)

        first, first_diag = self._shape(
            shaper,
            target,
            timestamp=1.0,
            gyro_deg_s=(4.0, 3.0),
            gyro_age_s=0.3,
        )
        self.assertEqual((first.roll_rate_deg_s, first.pitch_rate_deg_s), (0.0, 0.0))
        self.assertEqual(first_diag.entry_source, "zero")

        self._shape(shaper, target, timestamp=1.1, gate_open=False)
        restarted, restarted_diag = self._shape(
            shaper,
            target,
            timestamp=2.0,
            gyro_deg_s=(-2.0, 1.0),
            gyro_age_s=0.0,
        )
        self.assertEqual((restarted.roll_rate_deg_s, restarted.pitch_rate_deg_s), (-2.0, 1.0))
        self.assertEqual(restarted_diag.entry_source, "gyro")
        self.assertAlmostEqual(restarted_diag.entry_progress, 0.0)

    def test_command_shaper_zero_rate_source_ignores_fresh_gyro(self):
        shaper = GuidanceCommandShaper(
            GuidanceCommandShaperConfig(
                entry_handoff=EntryHandoffConfig(enabled=True, rate_source="zero")
            )
        )
        target = GuidanceSetpoint(timestamp=1.0, roll_rate_deg_s=8.0, pitch_rate_deg_s=-6.0)

        start, diagnostics = self._shape(
            shaper,
            target,
            timestamp=1.0,
            gyro_deg_s=(300.0, -400.0),
            gyro_age_s=0.0,
        )

        self.assertEqual((start.roll_rate_deg_s, start.pitch_rate_deg_s), (0.0, 0.0))
        self.assertEqual(diagnostics.entry_source, "zero")

    def test_tilt_envelope_softcap_is_symmetric_and_preserves_inward_commands(self):
        shaper = GuidanceCommandShaper(
            GuidanceCommandShaperConfig(
                tilt_envelope=TiltEnvelopeConfig(
                    enabled=True,
                    max_roll_angle_deg=35.0,
                    max_pitch_angle_deg=35.0,
                    softcap_band_deg=10.0,
                    hardcap_margin_deg=5.0,
                    hardcap_level_kp=3.0,
                    hardcap_max_level_rate_deg_s=3.0,
                )
            )
        )
        positive = GuidanceSetpoint(timestamp=1.0, roll_rate_deg_s=10.0)
        shaped_positive, positive_diag = self._shape(
            shaper, positive, attitude_deg=(30.0, 0.0)
        )
        self.assertAlmostEqual(shaped_positive.roll_rate_deg_s, 5.0)
        self.assertAlmostEqual(positive_diag.roll_softcap_factor, 0.5)

        shaper.reset()
        negative = GuidanceSetpoint(timestamp=2.0, roll_rate_deg_s=-10.0)
        shaped_negative, negative_diag = self._shape(
            shaper, negative, timestamp=2.0, attitude_deg=(-30.0, 0.0)
        )
        self.assertAlmostEqual(shaped_negative.roll_rate_deg_s, -5.0)
        self.assertAlmostEqual(negative_diag.roll_softcap_factor, 0.5)

        shaper.reset()
        inward = GuidanceSetpoint(timestamp=3.0, roll_rate_deg_s=-10.0, pitch_rate_deg_s=4.0)
        shaped_inward, inward_diag = self._shape(
            shaper, inward, timestamp=3.0, attitude_deg=(30.0, -30.0)
        )
        self.assertEqual(shaped_inward.roll_rate_deg_s, -10.0)
        self.assertEqual(shaped_inward.pitch_rate_deg_s, 4.0)
        self.assertEqual(inward_diag.roll_softcap_factor, 1.0)
        self.assertEqual(inward_diag.pitch_softcap_factor, 1.0)

    def test_tilt_envelope_hard_region_blends_to_bounded_leveling_rate(self):
        shaper = GuidanceCommandShaper(
            GuidanceCommandShaperConfig(
                tilt_envelope=TiltEnvelopeConfig(
                    enabled=True,
                    max_roll_angle_deg=35.0,
                    max_pitch_angle_deg=35.0,
                    softcap_band_deg=10.0,
                    hardcap_margin_deg=5.0,
                    hardcap_level_kp=3.0,
                    hardcap_max_level_rate_deg_s=3.0,
                )
            )
        )
        target = GuidanceSetpoint(timestamp=1.0, roll_rate_deg_s=10.0, pitch_rate_deg_s=-10.0)

        middle, middle_diag = self._shape(
            shaper, target, attitude_deg=(37.5, -37.5)
        )
        self.assertAlmostEqual(middle.roll_rate_deg_s, -1.5)
        self.assertAlmostEqual(middle.pitch_rate_deg_s, 1.5)
        self.assertAlmostEqual(middle_diag.roll_level_weight, 0.5)
        self.assertFalse(middle_diag.hardcap_active)

        full, full_diag = self._shape(
            shaper, target, timestamp=1.1, attitude_deg=(40.0, -40.0)
        )
        self.assertEqual(full.roll_rate_deg_s, -3.0)
        self.assertEqual(full.pitch_rate_deg_s, 3.0)
        self.assertEqual(full_diag.roll_level_weight, 1.0)
        self.assertTrue(full_diag.hardcap_active)

    def test_tilt_envelope_fails_closed_without_finite_attitude(self):
        shaper = GuidanceCommandShaper(
            GuidanceCommandShaperConfig(tilt_envelope=TiltEnvelopeConfig(enabled=True))
        )
        target = GuidanceSetpoint(timestamp=1.0, roll_rate_deg_s=1.0)

        missing, missing_diag = self._shape(shaper, target, attitude_deg=None)
        nonfinite, nonfinite_diag = self._shape(
            shaper,
            target,
            timestamp=2.0,
            attitude_deg=(float("nan"), 0.0),
        )

        self.assertFalse(missing.valid)
        self.assertEqual(missing.reject_reason, "tilt_attitude_unavailable")
        self.assertFalse(missing_diag.valid)
        self.assertFalse(nonfinite.valid)
        self.assertEqual(nonfinite_diag.reason, "tilt_attitude_unavailable")

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
            ("prefill_ready", False, "msp_prefill_not_ready"),
            ("msp_response_fresh", False, "msp_set_raw_rc_ack_stale"),
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
