import unittest

import numpy as np

from vision_guidance.flight_control import (
    AccelerationTiltRateConfig,
    BetaflightSafetyStateMachine,
    CommandWatchdog,
    EntryHandoffConfig,
    GuidanceCommandShaper,
    GuidanceCommandShaperConfig,
    GuidanceSetpoint,
    GuidanceSetpointHold,
    MotorOutputInterlock,
    MotorOutputInterlockConfig,
    RcCommandMapper,
    RcMappingConfig,
    SafetyInputs,
    SafetyState,
    TakeoverDurationInterlock,
    TakeoverDurationInterlockConfig,
    TiltEnvelopeConfig,
    aux_range_enabled,
    guidance_eval_to_setpoint,
    inertial_vector_to_body_frd,
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

        decision = safety.update(
            SafetyInputs(
                control_requested=True,
                allow_control=True,
                telemetry_fresh=True,
                attitude_synced=True,
                motor_output_ok=False,
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
        self.assertEqual(decision.state, SafetyState.FAILSAFE)
        self.assertEqual(decision.reason, "motor_output_interlock")

    def test_motor_output_interlock_latches_until_disarm(self):
        interlock = MotorOutputInterlock(
            MotorOutputInterlockConfig(
                enabled=True,
                channel_count=4,
                max_output_us=1200,
                max_spread_us=150,
                telemetry_timeout_s=0.75,
                latch_until_disarm=True,
            )
        )

        safe = interlock.update(
            armed=True,
            motor_outputs=(1056, 1057, 1056, 1057),
            telemetry_age_s=0.1,
        )
        self.assertTrue(safe.ok)
        fault = interlock.update(
            armed=True,
            motor_outputs=(1363, 1456, 1056, 1431),
            telemetry_age_s=0.1,
        )
        self.assertFalse(fault.ok)
        self.assertTrue(fault.latched)
        self.assertEqual(fault.reason, "motor_output_high")
        self.assertEqual(fault.output_max_us, 1456)
        self.assertEqual(fault.output_spread_us, 400)

        still_latched = interlock.update(
            armed=True,
            motor_outputs=(1056, 1057, 1056, 1057),
            telemetry_age_s=0.1,
        )
        self.assertFalse(still_latched.ok)
        self.assertTrue(still_latched.latched)
        self.assertEqual(still_latched.output_max_us, 1456)
        self.assertTrue(
            interlock.update(
                armed=False,
                motor_outputs=(1000, 1000, 1000, 1000),
                telemetry_age_s=0.1,
            ).ok
        )

    def test_motor_output_interlock_fails_closed_on_stale_telemetry(self):
        interlock = MotorOutputInterlock(
            MotorOutputInterlockConfig(enabled=True, telemetry_timeout_s=0.5)
        )

        state = interlock.update(
            armed=True,
            motor_outputs=(1056, 1057, 1056, 1057),
            telemetry_age_s=0.6,
        )

        self.assertFalse(state.ok)
        self.assertTrue(state.latched)
        self.assertEqual(state.reason, "motor_telemetry_stale")

        recovered = interlock.update(
            armed=True,
            motor_outputs=(1056, 1057, 1056, 1057),
            telemetry_age_s=0.1,
        )
        self.assertFalse(recovered.ok)
        self.assertTrue(recovered.latched)
        self.assertEqual(recovered.reason, "motor_telemetry_stale")

    def test_motor_output_interlock_graces_only_persistent_spread(self):
        interlock = MotorOutputInterlock(
            MotorOutputInterlockConfig(
                enabled=True,
                max_output_us=1500,
                max_spread_us=250,
                violation_grace_s=1.0,
                latch_until_disarm=True,
            )
        )

        grace = interlock.update(
            armed=True,
            motor_outputs=(1100, 1400, 1100, 1400),
            telemetry_age_s=0.1,
            timestamp=20.0,
        )
        recovered = interlock.update(
            armed=True,
            motor_outputs=(1200, 1201, 1200, 1201),
            telemetry_age_s=0.1,
            timestamp=25.0,
        )
        interlock.update(
            armed=True,
            motor_outputs=(1100, 1400, 1100, 1400),
            telemetry_age_s=0.1,
            timestamp=30.0,
        )
        persistent = interlock.update(
            armed=True,
            motor_outputs=(1100, 1400, 1100, 1400),
            telemetry_age_s=0.1,
            timestamp=31.0,
        )

        self.assertTrue(grace.ok)
        self.assertEqual(grace.reason, "motor_output_spread_grace")
        self.assertTrue(recovered.ok)
        self.assertEqual(recovered.reason, "ok")
        self.assertFalse(persistent.ok)
        self.assertTrue(persistent.latched)
        self.assertEqual(persistent.reason, "motor_output_spread_high")

    def test_takeover_duration_interlock_limits_continuous_noprop_takeover(self):
        interlock = TakeoverDurationInterlock(
            TakeoverDurationInterlockConfig(
                enabled=True,
                max_duration_s=3.0,
                latch_until_disarm=True,
            )
        )

        started = interlock.update(timestamp=10.0, armed=True, takeover_active=True)
        self.assertTrue(started.ok)
        self.assertEqual(started.reason, "timing")
        self.assertEqual(started.active_duration_s, 0.0)
        self.assertTrue(
            interlock.update(timestamp=12.99, armed=True, takeover_active=True).ok
        )

        expired = interlock.update(timestamp=13.0, armed=True, takeover_active=True)
        self.assertFalse(expired.ok)
        self.assertTrue(expired.latched)
        self.assertEqual(expired.reason, "takeover_duration_exceeded")
        self.assertAlmostEqual(expired.active_duration_s, 3.0)

        still_latched = interlock.update(
            timestamp=13.1,
            armed=True,
            takeover_active=False,
        )
        self.assertFalse(still_latched.ok)
        self.assertTrue(still_latched.latched)

        reset = interlock.update(timestamp=13.2, armed=False, takeover_active=False)
        self.assertTrue(reset.ok)
        self.assertFalse(reset.latched)

    def test_disabled_takeover_duration_accepts_null_limit_indefinitely(self):
        config = TakeoverDurationInterlockConfig.from_mapping(
            {"enabled": False, "max_duration_s": None, "latch_until_disarm": False}
        )
        interlock = TakeoverDurationInterlock(config)

        started = interlock.update(timestamp=1.0, armed=True, takeover_active=True)
        continued = interlock.update(timestamp=3601.0, armed=True, takeover_active=True)

        self.assertTrue(started.ok)
        self.assertTrue(continued.ok)
        self.assertEqual(continued.reason, "disabled")
        self.assertEqual(continued.active_duration_s, 0.0)
        self.assertIsNone(continued.max_duration_s)
        self.assertIsNone(continued.remaining_s)

    def test_takeover_duration_interlock_resets_when_released_before_limit(self):
        interlock = TakeoverDurationInterlock(
            TakeoverDurationInterlockConfig(enabled=True, max_duration_s=3.0)
        )

        interlock.update(timestamp=1.0, armed=True, takeover_active=True)
        released = interlock.update(timestamp=2.0, armed=True, takeover_active=False)
        restarted = interlock.update(timestamp=5.0, armed=True, takeover_active=True)

        self.assertTrue(released.ok)
        self.assertEqual(released.reason, "inactive")
        self.assertTrue(restarted.ok)
        self.assertEqual(restarted.active_duration_s, 0.0)

    def test_takeover_duration_interlock_allows_only_one_control_start_per_arm(self):
        interlock = TakeoverDurationInterlock(
            TakeoverDurationInterlockConfig(
                enabled=True,
                max_duration_s=2.0,
                latch_until_disarm=True,
                max_takeovers_per_arm=1,
            )
        )
        first = interlock.update(
            timestamp=1.0,
            armed=True,
            takeover_requested=True,
            control_active=True,
        )
        interlock.update(
            timestamp=1.5,
            armed=True,
            takeover_requested=True,
            control_active=False,
        )
        second = interlock.update(
            timestamp=1.6,
            armed=True,
            takeover_requested=True,
            control_active=True,
        )

        self.assertTrue(first.ok)
        self.assertEqual(first.takeover_count, 1)
        self.assertFalse(second.ok)
        self.assertTrue(second.latched)
        self.assertEqual(second.reason, "takeover_count_exceeded")

    def test_takeover_duration_interlock_rearms_after_continuous_release_dwell(self):
        interlock = TakeoverDurationInterlock(
            TakeoverDurationInterlockConfig(
                enabled=True,
                max_duration_s=5.0,
                latch_until_disarm=False,
                rearm_release_s=0.5,
            )
        )

        self.assertTrue(
            interlock.update(timestamp=10.0, armed=True, takeover_active=True).ok
        )
        expired = interlock.update(timestamp=15.0, armed=True, takeover_active=True)
        still_held = interlock.update(timestamp=15.1, armed=True, takeover_active=True)
        release_started = interlock.update(
            timestamp=15.2,
            armed=True,
            takeover_active=False,
        )
        release_complete = interlock.update(
            timestamp=15.7,
            armed=True,
            takeover_active=False,
        )
        restarted = interlock.update(timestamp=15.8, armed=True, takeover_active=True)

        self.assertFalse(expired.ok)
        self.assertFalse(expired.latched)
        self.assertEqual(expired.reason, "takeover_duration_exceeded")
        self.assertFalse(still_held.ok)
        self.assertEqual(still_held.reason, "takeover_release_required")
        self.assertTrue(release_started.ok)
        self.assertEqual(release_started.reason, "takeover_rearm_wait")
        self.assertTrue(release_complete.ok)
        self.assertEqual(release_complete.reason, "inactive")
        self.assertTrue(restarted.ok)
        self.assertEqual(restarted.active_duration_s, 0.0)

    def test_takeover_duration_counts_only_actual_control_and_requires_release(self):
        interlock = TakeoverDurationInterlock(
            TakeoverDurationInterlockConfig(
                enabled=True,
                max_duration_s=10.0,
                latch_until_disarm=False,
                rearm_release_s=0.5,
            )
        )

        waiting = interlock.update(
            timestamp=1.0,
            armed=True,
            takeover_requested=True,
            control_active=False,
        )
        still_waiting = interlock.update(
            timestamp=6.0,
            armed=True,
            takeover_requested=True,
            control_active=False,
        )
        started = interlock.update(
            timestamp=6.1,
            armed=True,
            takeover_requested=True,
            control_active=True,
        )
        running = interlock.update(
            timestamp=11.1,
            armed=True,
            takeover_requested=True,
            control_active=True,
        )
        paused = interlock.update(
            timestamp=13.1,
            armed=True,
            takeover_requested=True,
            control_active=False,
        )
        resumed = interlock.update(
            timestamp=13.2,
            armed=True,
            takeover_requested=True,
            control_active=True,
        )
        expired = interlock.update(
            timestamp=18.2,
            armed=True,
            takeover_requested=True,
            control_active=True,
        )

        self.assertEqual(waiting.reason, "waiting_for_control")
        self.assertEqual(still_waiting.active_duration_s, 0.0)
        self.assertEqual(started.active_duration_s, 0.0)
        self.assertAlmostEqual(running.active_duration_s, 5.0)
        self.assertAlmostEqual(paused.active_duration_s, 5.0)
        self.assertAlmostEqual(resumed.active_duration_s, 5.0)
        self.assertFalse(expired.ok)
        self.assertAlmostEqual(expired.active_duration_s, 10.0)

        held = interlock.update(
            timestamp=19.0,
            armed=True,
            takeover_requested=True,
            control_active=False,
        )
        release_started = interlock.update(
            timestamp=19.1,
            armed=True,
            takeover_requested=False,
            control_active=False,
        )
        too_soon = interlock.update(
            timestamp=19.5,
            armed=True,
            takeover_requested=True,
            control_active=False,
        )
        interlock.update(
            timestamp=20.0,
            armed=True,
            takeover_requested=False,
            control_active=False,
        )
        rearmed = interlock.update(
            timestamp=20.5,
            armed=True,
            takeover_requested=False,
            control_active=False,
        )

        self.assertEqual(held.reason, "takeover_release_required")
        self.assertTrue(release_started.ok)
        self.assertEqual(release_started.reason, "takeover_rearm_wait")
        self.assertFalse(too_soon.ok)
        self.assertEqual(too_soon.reason, "takeover_release_required")
        self.assertTrue(rearmed.ok)
        self.assertEqual(rearmed.active_duration_s, 0.0)

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
            R_IB=np.eye(3),
            rate_gain_matrix=[[10.0, 0.0, 0.0], [0.0, -5.0, 0.0], [0.0, 0.0, 2.0]],
            hover_thrust=0.55,
        )

        self.assertTrue(setpoint.valid)
        self.assertEqual(setpoint.roll_rate_deg_s, 10.0)
        self.assertEqual(setpoint.pitch_rate_deg_s, -10.0)
        self.assertEqual(setpoint.yaw_rate_deg_s, 6.0)
        self.assertEqual(setpoint.thrust, 0.55)
        self.assertEqual(setpoint.mapping_type, "direct_rate_matrix")

    def test_accel_tilt_rate_maps_acceleration_through_attitude_error(self):
        guidance = GuidanceEval(
            timestamp=2.0,
            g_eval=np.array([0.0, 1.0, 0.0]),
            valid=True,
            quality=1.0,
        )

        setpoint = guidance_eval_to_setpoint(
            guidance,
            R_IB=np.eye(3),
            rate_gain_matrix=np.zeros((3, 3)),
            hover_thrust=0.4,
            mapping_type="accel_tilt_rate",
            accel_tilt_rate={
                "roll_attitude_kp_s_inv": 4.0,
                "pitch_attitude_kp_s_inv": 4.0,
                "max_roll_tilt_deg": 20.0,
                "max_pitch_tilt_deg": 20.0,
                "max_roll_rate_deg_s": 60.0,
                "max_pitch_rate_deg_s": 60.0,
            },
        )

        desired_roll = np.rad2deg(np.arctan2(1.0, 9.80665))
        self.assertTrue(setpoint.valid)
        self.assertEqual(setpoint.mapping_type, "accel_tilt_rate")
        self.assertAlmostEqual(setpoint.desired_roll_angle_deg, desired_roll)
        self.assertAlmostEqual(setpoint.desired_pitch_angle_deg, 0.0)
        self.assertAlmostEqual(setpoint.roll_rate_deg_s, 4.0 * desired_roll)
        self.assertAlmostEqual(setpoint.pitch_rate_deg_s, 0.0)
        self.assertEqual(setpoint.thrust, 0.4)

    def test_accel_tilt_rate_measured_thrust_maps_hover_and_max_load(self):
        config = {
            "thrust_feedforward": {
                "enabled": True,
                "model": "measured_load_factor",
                "hover_load_factor_g": 1.0,
                "max_load_factor_g": 2.37,
                "minimum_tilt_cosine": 0.5,
                "calibration_id": "LOG00062_1275_1500",
            }
        }
        hover = guidance_eval_to_setpoint(
            GuidanceEval(timestamp=1.0, g_eval=np.zeros(3), valid=True, quality=1.0),
            R_IB=np.eye(3),
            rate_gain_matrix=np.zeros((3, 3)),
            hover_thrust=0.5,
            mapping_type="accel_tilt_rate",
            accel_tilt_rate=config,
        )
        max_load = guidance_eval_to_setpoint(
            GuidanceEval(
                timestamp=2.0,
                g_eval=np.array([0.0, 0.0, -1.37 * 9.80665]),
                valid=True,
                quality=1.0,
            ),
            R_IB=np.eye(3),
            rate_gain_matrix=np.zeros((3, 3)),
            hover_thrust=0.5,
            mapping_type="accel_tilt_rate",
            accel_tilt_rate=config,
        )

        self.assertEqual(hover.thrust_model, "measured_load_factor")
        self.assertAlmostEqual(hover.thrust_load_factor_raw_g, 1.0)
        self.assertAlmostEqual(hover.thrust, 0.5)
        self.assertFalse(hover.thrust_command_limited)
        self.assertAlmostEqual(max_load.thrust_load_factor_raw_g, 2.37)
        self.assertAlmostEqual(max_load.thrust_command_raw, 1.0)
        self.assertAlmostEqual(max_load.thrust, 1.0)

    def test_accel_tilt_rate_measured_thrust_compensates_current_tilt(self):
        roll_rad = np.deg2rad(35.0)
        R_IB = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, np.cos(roll_rad), -np.sin(roll_rad)],
                [0.0, np.sin(roll_rad), np.cos(roll_rad)],
            ]
        )
        setpoint = guidance_eval_to_setpoint(
            GuidanceEval(timestamp=1.0, g_eval=np.zeros(3), valid=True, quality=1.0),
            R_IB=R_IB,
            rate_gain_matrix=np.zeros((3, 3)),
            hover_thrust=0.5,
            mapping_type="accel_tilt_rate",
            accel_tilt_rate={
                "thrust_feedforward": {
                    "enabled": True,
                    "model": "measured_load_factor",
                    "hover_load_factor_g": 1.0,
                    "max_load_factor_g": 2.37,
                    "minimum_tilt_cosine": 0.5,
                    "calibration_id": "LOG00062_1275_1500",
                }
            },
        )

        expected_load = 1.0 / np.cos(roll_rad)
        expected_thrust = 0.5 + 0.5 * (expected_load - 1.0) / 1.37
        self.assertAlmostEqual(setpoint.thrust_load_factor_raw_g, expected_load)
        self.assertAlmostEqual(setpoint.thrust, expected_thrust)

    def test_accel_tilt_rate_measured_thrust_clamps_excess_load(self):
        setpoint = guidance_eval_to_setpoint(
            GuidanceEval(
                timestamp=1.0,
                g_eval=np.array([0.0, 0.0, -30.0]),
                valid=True,
                quality=1.0,
            ),
            R_IB=np.eye(3),
            rate_gain_matrix=np.zeros((3, 3)),
            hover_thrust=0.5,
            mapping_type="accel_tilt_rate",
            accel_tilt_rate={
                "thrust_feedforward": {
                    "enabled": True,
                    "model": "measured_load_factor",
                    "hover_load_factor_g": 1.0,
                    "max_load_factor_g": 2.37,
                    "minimum_tilt_cosine": 0.5,
                    "calibration_id": "LOG00062_1275_1500",
                }
            },
        )

        self.assertGreater(setpoint.thrust_command_raw, 1.0)
        self.assertEqual(setpoint.thrust, 1.0)
        self.assertTrue(setpoint.thrust_command_limited)

    def test_accel_tilt_rate_uses_current_attitude_and_explicit_output_sign(self):
        pitch_rad = np.deg2rad(-3.0)
        R_IB = np.array(
            [
                [np.cos(pitch_rad), 0.0, np.sin(pitch_rad)],
                [0.0, 1.0, 0.0],
                [-np.sin(pitch_rad), 0.0, np.cos(pitch_rad)],
            ]
        )
        guidance = GuidanceEval(
            timestamp=2.0,
            g_eval=np.array([1.0, 0.0, 0.0]),
            valid=True,
            quality=1.0,
        )

        setpoint = guidance_eval_to_setpoint(
            guidance,
            R_IB=R_IB,
            rate_gain_matrix=np.zeros((3, 3)),
            hover_thrust=0.4,
            mapping_type="accel_tilt_rate",
            accel_tilt_rate={"pitch_rate_sign": -1.0},
        )

        desired_pitch = -np.rad2deg(np.arctan2(1.0, 9.80665))
        pitch_error = desired_pitch - (-3.0)
        self.assertAlmostEqual(setpoint.current_pitch_angle_deg, -3.0)
        self.assertAlmostEqual(setpoint.pitch_attitude_error_deg, pitch_error)
        self.assertAlmostEqual(setpoint.pitch_rate_deg_s, -4.0 * pitch_error)

    def test_accel_tilt_rate_limits_tilt_and_rate(self):
        guidance = GuidanceEval(
            timestamp=2.0,
            g_eval=np.array([0.0, 100.0, 0.0]),
            valid=True,
            quality=1.0,
        )

        setpoint = guidance_eval_to_setpoint(
            guidance,
            R_IB=np.eye(3),
            rate_gain_matrix=np.zeros((3, 3)),
            hover_thrust=0.4,
            mapping_type="accel_tilt_rate",
            accel_tilt_rate={
                "roll_attitude_kp_s_inv": 10.0,
                "max_roll_tilt_deg": 10.0,
                "max_roll_rate_deg_s": 30.0,
            },
        )

        self.assertEqual(setpoint.desired_roll_angle_deg, 10.0)
        self.assertEqual(setpoint.roll_rate_deg_s, 30.0)

    def test_accel_tilt_rate_config_rejects_invalid_axis_sign(self):
        with self.assertRaisesRegex(ValueError, "roll_rate_sign must be -1 or 1"):
            AccelerationTiltRateConfig(roll_rate_sign=0.0)

    def test_guidance_eval_to_setpoint_rotates_inertial_vector_into_body_frd(self):
        guidance = GuidanceEval(
            timestamp=2.0,
            g_eval=np.array([1.0, 0.0, 0.0]),
            valid=True,
            quality=1.0,
        )
        yaw_right_90_R_IB = np.array(
            [
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )

        body = inertial_vector_to_body_frd(guidance.g_eval, yaw_right_90_R_IB)
        setpoint = guidance_eval_to_setpoint(
            guidance,
            R_IB=yaw_right_90_R_IB,
            rate_gain_matrix=[[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            hover_thrust=0.5,
        )

        self.assertTrue(np.allclose(body, np.array([0.0, -1.0, 0.0]), atol=1e-9))
        self.assertAlmostEqual(setpoint.roll_rate_deg_s, -1.0)
        self.assertAlmostEqual(setpoint.pitch_rate_deg_s, 0.0)

    def test_guidance_eval_to_setpoint_rejects_missing_or_invalid_attitude(self):
        guidance = GuidanceEval(
            timestamp=2.0,
            g_eval=np.array([1.0, 0.0, 0.0]),
            valid=True,
            quality=1.0,
        )
        matrix = np.eye(3)

        missing = guidance_eval_to_setpoint(
            guidance,
            R_IB=None,
            rate_gain_matrix=matrix,
            hover_thrust=0.5,
        )
        invalid = guidance_eval_to_setpoint(
            guidance,
            R_IB=np.zeros((3, 3)),
            rate_gain_matrix=matrix,
            hover_thrust=0.5,
        )

        self.assertFalse(missing.valid)
        self.assertEqual(missing.reject_reason, "guidance_attitude_missing")
        self.assertFalse(invalid.valid)
        self.assertEqual(invalid.reject_reason, "invalid_guidance_frame_transform")

    def test_aux_range_enabled_uses_one_based_channel_index(self):
        self.assertTrue(aux_range_enabled((1500, 1500, 1500, 1500, 1800), channel_index=5, min_us=1700, max_us=2100))
        self.assertFalse(aux_range_enabled((1500, 1500, 1500, 1500, 1600), channel_index=5, min_us=1700, max_us=2100))


if __name__ == "__main__":
    unittest.main()
