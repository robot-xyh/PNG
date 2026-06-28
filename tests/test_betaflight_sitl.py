import math
import socket
import struct
import threading
import unittest
from types import SimpleNamespace

import numpy as np

from examples.run_airsim_betaflight_truth_png import _any_motor_saturated, _png_accel_for_not_closing
from vision_guidance.betaflight_sitl import (
    FDM_PACKET_STRUCT,
    LEGACY_FDM_PACKET_STRUCT,
    RC_PACKET_STRUCT,
    SERVO_PACKET_STRUCT,
    SERVO_RAW_PACKET_STRUCT,
    AngleRCConfig,
    BodyRateRCConfig,
    BetaflightFdmPacket,
    BetaflightMSPClient,
    BetaflightSITLBridge,
    MSP_ATTITUDE,
    MSP_MOTOR,
    MSP_SET_RAW_RC,
    MSP_STATUS,
    RateCommand,
    angle_rc_from_png_accel,
    betaflight_to_airsim_motor_pwms,
    betaflight_to_airsim_motor_order,
    body_rate_rc_from_rate_command,
    decode_msp_v1_frame,
    encode_msp_set_raw_rc,
    encode_msp_v1,
    pack_fdm_packet,
    pack_rc_packet,
    fdm_packet_from_airsim,
    gazebo_bridge_fdm_quat_from_airsim,
    quat_multiply_wxyz,
    parse_msp_attitude,
    parse_msp_motor,
    parse_msp_status,
    rate_command_from_body_rate_output,
    transform_betaflight_motor_output,
    unpack_servo_packet,
    unpack_servo_raw_packet,
    yaw_from_quat_wxyz,
)
from vision_guidance.truth_png import compute_truth_png


class BetaflightSITLProtocolTest(unittest.TestCase):
    @staticmethod
    def _free_udp_port():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
        sock.close()
        return port

    def test_fdm_packet_sizes_match_betaflight_structs(self):
        packet = BetaflightFdmPacket(
            timestamp_s=1.25,
            imu_angular_velocity_rpy=np.array([0.1, 0.2, 0.3]),
            imu_linear_acceleration_xyz=np.array([1.0, 2.0, 3.0]),
            imu_orientation_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            velocity_xyz=np.array([4.0, 5.0, 6.0]),
            position_xyz=np.array([7.0, 8.0, -9.0]),
            pressure_pa=100000.0,
        )

        current = pack_fdm_packet(packet)
        legacy = pack_fdm_packet(packet, legacy_without_pressure=True)

        self.assertEqual(len(current), FDM_PACKET_STRUCT.size)
        self.assertEqual(FDM_PACKET_STRUCT.size, 18 * 8)
        self.assertEqual(len(legacy), LEGACY_FDM_PACKET_STRUCT.size)
        self.assertEqual(LEGACY_FDM_PACKET_STRUCT.size, 17 * 8)
        self.assertAlmostEqual(struct.unpack("<18d", current)[0], 1.25)
        self.assertAlmostEqual(struct.unpack("<18d", current)[-1], 100000.0)

    def test_rc_packet_is_fixed_16_channel_aetr(self):
        data = pack_rc_packet(2.0, [1500, 1501, 1100, 1499, 2000])
        decoded = RC_PACKET_STRUCT.unpack(data)

        self.assertEqual(len(data), RC_PACKET_STRUCT.size)
        self.assertEqual(decoded[0], 2.0)
        self.assertEqual(decoded[1:6], (1500, 1501, 1100, 1499, 2000))
        self.assertEqual(decoded[6:], (1000,) * 11)

    def test_rc_packet_clamps_channels(self):
        decoded = RC_PACKET_STRUCT.unpack(pack_rc_packet(0.0, [900, 2100, 1500, 1500]))

        self.assertEqual(decoded[1:5], (1000, 2000, 1500, 1500))

    def test_servo_packet_unpack_and_motor_order(self):
        packet = unpack_servo_packet(SERVO_PACKET_STRUCT.pack(0.1, 0.2, 0.3, 0.4))

        self.assertTrue(np.allclose(packet.motor_speed, (0.1, 0.2, 0.3, 0.4)))
        self.assertTrue(np.allclose(betaflight_to_airsim_motor_order(packet.motor_speed), (0.2, 0.3, 0.4, 0.1)))

    def test_motor_output_transform_modes(self):
        motor_speed = (0.0, 0.25, 0.64, 1.2)

        self.assertTrue(np.allclose(transform_betaflight_motor_output(motor_speed), (0.0, 0.25, 0.64, 1.0)))
        self.assertTrue(np.allclose(transform_betaflight_motor_output(motor_speed, mode="sqrt"), (0.0, 0.5, 0.8, 1.0)))
        self.assertTrue(
            np.allclose(
                transform_betaflight_motor_output(motor_speed, mode="gamma", gamma=2.0),
                (0.0, 0.0625, 0.4096, 1.0),
            )
        )
        self.assertTrue(
            np.allclose(
                transform_betaflight_motor_output(motor_speed, mode="scale_bias", scale=0.5, bias=0.1),
                (0.1, 0.225, 0.42, 0.6),
            )
        )

    def test_motor_output_transform_rejects_invalid_gamma(self):
        with self.assertRaises(ValueError):
            transform_betaflight_motor_output((0.1, 0.2, 0.3, 0.4), mode="gamma", gamma=0.0)

    def test_motor_pwm_mapping_applies_transform_before_order(self):
        pwms = betaflight_to_airsim_motor_pwms(
            (0.1, 0.2, 0.3, 0.4),
            transform="scale_bias",
            scale=2.0,
            bias=0.1,
        )

        self.assertTrue(np.allclose(pwms, (0.5, 0.7, 0.9, 0.3)))

    def test_servo_raw_packet_uses_c_padding(self):
        data = SERVO_RAW_PACKET_STRUCT.pack(4, *[1000.0 + i for i in range(16)])
        packet = unpack_servo_raw_packet(data)

        self.assertEqual(len(data), 68)
        self.assertEqual(packet.motor_count, 4)
        self.assertEqual(packet.pwm_output_raw[:4], (1000.0, 1001.0, 1002.0, 1003.0))

    def test_msp_set_raw_rc_encoder(self):
        frame = encode_msp_set_raw_rc([1500, 1500, 1000, 1500, 2000, 1000, 1000, 1000])

        self.assertEqual(frame[:3], b"$M<")
        self.assertEqual(frame[3], 16)
        self.assertEqual(frame[4], 200)
        self.assertEqual(len(frame), 22)
        checksum = 0
        for byte in frame[3:-1]:
            checksum ^= byte
        self.assertEqual(frame[-1], checksum)

    def test_msp_v1_frame_decode_and_payload_parsers(self):
        frame = encode_msp_v1(MSP_ATTITUDE, struct.pack("<hhh", -123, 45, 270), direction=b">")
        decoded = decode_msp_v1_frame(frame)
        attitude = parse_msp_attitude(decoded.payload)

        self.assertEqual(decoded.command, MSP_ATTITUDE)
        self.assertEqual(decoded.direction, b">")
        self.assertAlmostEqual(attitude.roll_deg, -12.3)
        self.assertAlmostEqual(attitude.pitch_deg, 4.5)
        self.assertAlmostEqual(attitude.yaw_deg, 270.0)

        motor = parse_msp_motor(struct.pack("<8H", 1000, 1100, 1200, 1300, 0, 0, 0, 0))
        self.assertEqual(motor.outputs_us[:4], (1000, 1100, 1200, 1300))

        status_payload = struct.pack("<HHHIBHHBI", 125, 0, 33, 0x12345678, 2, 456, 0, 0, 0xA5A5A5A5)
        status = parse_msp_status(status_payload)
        self.assertEqual(status.cycle_time_us, 125)
        self.assertEqual(status.sensor_flags, 33)
        self.assertEqual(status.flight_mode_flags, 0x12345678)
        self.assertAlmostEqual(status.average_system_load_percent, 45.6)
        self.assertEqual(status.arming_disable_flags, 0xA5A5A5A5)

    def test_msp_client_requests_frame_from_tcp_server(self):
        port = self._free_udp_port()
        ready = threading.Event()

        def server():
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("127.0.0.1", port))
            srv.listen(1)
            ready.set()
            conn, _addr = srv.accept()
            try:
                request = conn.recv(64)
                self.assertEqual(decode_msp_v1_frame(request).command, MSP_ATTITUDE)
                conn.sendall(encode_msp_v1(MSP_ATTITUDE, struct.pack("<hhh", 10, -20, 90), direction=b">"))
            finally:
                conn.close()
                srv.close()

        thread = threading.Thread(target=server, daemon=True)
        thread.start()
        self.assertTrue(ready.wait(1.0))
        with BetaflightMSPClient(host="127.0.0.1", port=port, timeout_s=1.0) as client:
            attitude = client.read_attitude(timeout_s=1.0)
        thread.join(1.0)

        self.assertAlmostEqual(attitude.roll_deg, 1.0)
        self.assertAlmostEqual(attitude.pitch_deg, -2.0)
        self.assertAlmostEqual(attitude.yaw_deg, 90.0)

    def test_msp_client_sends_raw_rc_frame_to_tcp_server(self):
        port = self._free_udp_port()
        ready = threading.Event()
        received = []

        def server():
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("127.0.0.1", port))
            srv.listen(1)
            ready.set()
            conn, _addr = srv.accept()
            try:
                data = b""
                while len(data) < 22:
                    chunk = conn.recv(22 - len(data))
                    if not chunk:
                        break
                    data += chunk
                received.append(data)
            finally:
                conn.close()
                srv.close()

        thread = threading.Thread(target=server, daemon=True)
        thread.start()
        self.assertTrue(ready.wait(1.0))
        with BetaflightMSPClient(host="127.0.0.1", port=port, timeout_s=1.0) as client:
            client.send_raw_rc([1500, 1501, 1250, 1499, 2000, 1000, 1000, 1000])
        thread.join(1.0)

        self.assertEqual(len(received), 1)
        frame = decode_msp_v1_frame(received[0])
        self.assertEqual(frame.command, MSP_SET_RAW_RC)
        self.assertEqual(struct.unpack("<8H", frame.payload), (1500, 1501, 1250, 1499, 2000, 1000, 1000, 1000))

    def test_yaw_from_quaternion(self):
        half = math.radians(45.0)
        yaw = yaw_from_quat_wxyz([math.cos(half), 0.0, 0.0, math.sin(half)])

        self.assertAlmostEqual(math.degrees(yaw), 90.0)

    def test_gazebo_bridge_quaternion_cancels_sitl_pre_rotation(self):
        yaw_half = math.radians(15.0)
        airsim_quat = np.array([math.cos(yaw_half), 0.0, 0.0, math.sin(yaw_half)])

        fdm_quat = gazebo_bridge_fdm_quat_from_airsim(airsim_quat)
        sitl_rz_plus_90 = np.array([math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)])
        recovered = quat_multiply_wxyz(sitl_rz_plus_90, fdm_quat)

        self.assertTrue(np.allclose(recovered, airsim_quat, atol=1.0e-6))

    def test_fdm_packet_gazebo_bridge_keeps_pitch_gyro_for_sitl_flip(self):
        vector = lambda x, y, z: SimpleNamespace(x_val=x, y_val=y, z_val=z)
        quat = SimpleNamespace(w_val=1.0, x_val=0.0, y_val=0.0, z_val=0.0)
        kin = SimpleNamespace(
            position=vector(0.0, 0.0, -1.0),
            angular_velocity=vector(0.1, 0.2, 0.3),
            linear_acceleration=vector(1.0, 2.0, 3.0),
            linear_velocity=vector(4.0, 5.0, 6.0),
            orientation=quat,
        )

        direct = fdm_packet_from_airsim(kin, timestamp_s=0.0, frame_mode="airsim_direct")
        gazebo = fdm_packet_from_airsim(kin, timestamp_s=0.0, frame_mode="gazebo_bridge")

        self.assertTrue(np.allclose(direct.imu_angular_velocity_rpy, [0.1, 0.2, 0.3]))
        self.assertTrue(np.allclose(gazebo.imu_angular_velocity_rpy, [0.1, 0.2, 0.3]))
        self.assertAlmostEqual(math.degrees(yaw_from_quat_wxyz(gazebo.imu_orientation_quat_wxyz)), -90.0)

    def test_bridge_receives_udp_motor_packet(self):
        pwm_port = self._free_udp_port()
        state_port = self._free_udp_port()
        rc_port = self._free_udp_port()
        with BetaflightSITLBridge(
            host="127.0.0.1",
            bind_host="127.0.0.1",
            state_port=state_port,
            rc_port=rc_port,
            pwm_port=pwm_port,
        ) as bridge:
            tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                tx.sendto(SERVO_PACKET_STRUCT.pack(0.11, 0.22, 0.33, 0.44), ("127.0.0.1", pwm_port))
                motor_speed = bridge.poll_motor_output(timeout_s=1.0)
            finally:
                tx.close()

        self.assertTrue(np.allclose(motor_speed, (0.11, 0.22, 0.33, 0.44)))


class BetaflightSITLControlTest(unittest.TestCase):
    def test_rate_command_maps_neutral_rates_to_body_rate_rc(self):
        result = body_rate_rc_from_rate_command(RateCommand(0.0, 0.0, 0.0, 0.5))

        self.assertEqual(result.command.roll, 1500)
        self.assertEqual(result.command.pitch, 1500)
        self.assertEqual(result.command.yaw, 1500)
        self.assertEqual(result.command.throttle, 1500)
        self.assertEqual(result.command.aux[:2], (2000, 1000))
        self.assertAlmostEqual(result.throttle_norm, 0.5)

    def test_rate_command_clamps_rates_and_thrust(self):
        cfg = BodyRateRCConfig(max_roll_rate_rad_s=1.0, max_pitch_rate_rad_s=1.0, max_yaw_rate_rad_s=1.0)

        high = body_rate_rc_from_rate_command(RateCommand(2.0, 2.0, 2.0, 2.0), cfg)
        low = body_rate_rc_from_rate_command(RateCommand(-2.0, -2.0, -2.0, -1.0), cfg)

        self.assertEqual((high.command.roll, high.command.pitch, high.command.yaw, high.command.throttle), (2000, 2000, 2000, 2000))
        self.assertEqual((low.command.roll, low.command.pitch, low.command.yaw, low.command.throttle), (1000, 1000, 1000, 1000))
        self.assertAlmostEqual(high.roll_norm, 1.0)
        self.assertAlmostEqual(low.roll_norm, -1.0)

    def test_rate_command_mapper_can_invert_pitch_sign(self):
        cfg = BodyRateRCConfig(max_pitch_rate_rad_s=1.0, pitch_sign=-1.0)
        result = body_rate_rc_from_rate_command(RateCommand(0.0, 0.5, 0.0, 0.5), cfg)

        self.assertEqual(result.command.pitch, 1250)
        self.assertAlmostEqual(result.pitch_norm, -0.5)

    def test_rate_command_from_body_rate_output_uses_current_png_body_rate_result(self):
        command = rate_command_from_body_rate_output(
            {
                "body_rates_rad_s": np.array([0.1, -0.2, 0.3]),
                "thrust": 0.6,
            }
        )

        self.assertAlmostEqual(command.roll_rate_rad_s, 0.1)
        self.assertAlmostEqual(command.pitch_rate_rad_s, -0.2)
        self.assertAlmostEqual(command.yaw_rate_rad_s, 0.3)
        self.assertAlmostEqual(command.thrust_z, 0.6)

    def test_png_acceleration_maps_to_angle_rc(self):
        cfg = AngleRCConfig(max_tilt_deg=35.0, hover_throttle=1500)
        accel = np.array([
            math.tan(math.radians(10.0)) * 9.80665,
            math.tan(math.radians(5.0)) * 9.80665,
            0.0,
        ])

        result = angle_rc_from_png_accel(
            accel,
            relative_position_ned=np.array([50.0, 50.0, -10.0]),
            relative_velocity_ned=np.array([-5.0, 0.0, 0.0]),
            current_yaw_rad=0.0,
            config=cfg,
        )

        self.assertGreater(result.command.pitch, 1500)
        self.assertGreater(result.command.roll, 1500)
        self.assertGreater(result.command.yaw, 1500)
        self.assertGreater(result.command.throttle, 1500)
        self.assertAlmostEqual(result.pitch_target_deg, 10.0, places=6)
        self.assertAlmostEqual(result.roll_target_deg, 5.0, places=6)

    def test_png_acceleration_maps_in_current_body_heading(self):
        cfg = AngleRCConfig(max_tilt_deg=35.0)
        accel_world_y = np.array([0.0, math.tan(math.radians(10.0)) * 9.80665, 0.0])

        result = angle_rc_from_png_accel(
            accel_world_y,
            relative_position_ned=np.array([0.0, 50.0, 0.0]),
            relative_velocity_ned=np.zeros(3),
            current_yaw_rad=math.radians(90.0),
            config=cfg,
        )

        self.assertGreater(result.command.pitch, 1500)
        self.assertAlmostEqual(result.roll_target_deg, 0.0, places=6)
        self.assertAlmostEqual(result.pitch_target_deg, 10.0, places=6)

    def test_angle_targets_share_combined_tilt_limit(self):
        cfg = AngleRCConfig(max_tilt_deg=25.0, hover_throttle=1500)
        accel = np.array([
            math.tan(math.radians(30.0)) * 9.80665,
            math.tan(math.radians(40.0)) * 9.80665,
            0.0,
        ])

        result = angle_rc_from_png_accel(
            accel,
            relative_position_ned=np.array([50.0, 0.0, 0.0]),
            relative_velocity_ned=np.zeros(3),
            current_yaw_rad=0.0,
            config=cfg,
        )

        self.assertAlmostEqual(result.desired_tilt_deg, 25.0, places=6)
        self.assertLess(result.tilt_scale, 1.0)
        self.assertLessEqual(math.hypot(result.roll_target_deg, result.pitch_target_deg), 25.0 + 1.0e-6)
        self.assertAlmostEqual(result.pitch_target_deg / result.roll_target_deg, 30.0 / 40.0, places=6)
        self.assertLess(result.command.roll, 2000)
        self.assertLess(result.command.pitch, 2000)

    def test_vertical_acceleration_increases_throttle_for_up_command(self):
        cfg = AngleRCConfig(
            max_tilt_deg=25.0,
            hover_throttle=1500,
            vertical_position_gain_rc_per_m=0.0,
            vertical_velocity_gain_rc_per_mps=0.0,
            vertical_accel_gain_rc_per_mps2=25.0,
            tilt_throttle_compensation=False,
        )

        result = angle_rc_from_png_accel(
            np.array([0.0, 0.0, -2.0]),
            relative_position_ned=np.zeros(3),
            relative_velocity_ned=np.zeros(3),
            current_yaw_rad=0.0,
            config=cfg,
        )

        self.assertEqual(result.command.throttle, 1550)
        self.assertAlmostEqual(result.vertical_accel_throttle_delta, 50.0)

    def test_tilt_throttle_compensation_adds_throttle(self):
        accel = np.array([math.tan(math.radians(20.0)) * 9.80665, 0.0, 0.0])
        common = dict(
            max_tilt_deg=25.0,
            hover_throttle=1500,
            vertical_position_gain_rc_per_m=0.0,
            vertical_velocity_gain_rc_per_mps=0.0,
            vertical_accel_gain_rc_per_mps2=0.0,
        )

        without_comp = angle_rc_from_png_accel(
            accel,
            relative_position_ned=np.zeros(3),
            relative_velocity_ned=np.zeros(3),
            current_yaw_rad=0.0,
            config=AngleRCConfig(**common, tilt_throttle_compensation=False),
        )
        with_comp = angle_rc_from_png_accel(
            accel,
            relative_position_ned=np.zeros(3),
            relative_velocity_ned=np.zeros(3),
            current_yaw_rad=0.0,
            config=AngleRCConfig(**common, tilt_throttle_compensation=True),
        )

        self.assertEqual(without_comp.command.throttle, 1500)
        self.assertGreater(with_comp.command.throttle, without_comp.command.throttle)
        self.assertGreater(with_comp.tilt_throttle_delta, 0.0)

    def test_yaw_command_uses_deadband_and_delta_limit(self):
        cfg = AngleRCConfig(max_yaw_rc_delta=150.0, yaw_deadband_deg=2.0, yaw_full_scale_deg=45.0)

        small = angle_rc_from_png_accel(
            np.zeros(3),
            relative_position_ned=np.array([50.0, math.tan(math.radians(1.0)) * 50.0, 0.0]),
            relative_velocity_ned=np.zeros(3),
            current_yaw_rad=0.0,
            config=cfg,
        )
        large = angle_rc_from_png_accel(
            np.zeros(3),
            relative_position_ned=np.array([0.0, 50.0, 0.0]),
            relative_velocity_ned=np.zeros(3),
            current_yaw_rad=0.0,
            config=cfg,
        )

        self.assertEqual(small.command.yaw, 1500)
        self.assertEqual(large.command.yaw, 1650)
        self.assertAlmostEqual(large.yaw_rc_delta, 150.0)

    def test_not_closing_zero_mode_preserves_default_invalid_accel(self):
        rel_pos = np.array([20.0, 10.0, 0.0])
        rel_vel = np.array([4.0, 0.0, 0.0])
        result = compute_truth_png(rel_pos, rel_vel, navigation_constant=3.0, max_accel=15.0)

        accel, valid, source = _png_accel_for_not_closing(
            result,
            rel_pos,
            rel_vel,
            SimpleNamespace(not_closing_mode="zero", not_closing_hold_s=0.35, navigation_constant=3.0, max_accel=15.0),
            last_valid_accel=None,
            last_valid_age_s=float("inf"),
        )

        self.assertFalse(valid)
        self.assertEqual(source, "not_closing_zero")
        self.assertTrue(np.allclose(accel, np.zeros(3)))

    def test_not_closing_abs_png_keeps_lateral_command(self):
        rel_pos = np.array([20.0, 10.0, 0.0])
        rel_vel = np.array([4.0, 0.0, 0.0])
        result = compute_truth_png(rel_pos, rel_vel, navigation_constant=3.0, max_accel=15.0)

        accel, valid, source = _png_accel_for_not_closing(
            result,
            rel_pos,
            rel_vel,
            SimpleNamespace(not_closing_mode="abs_png", not_closing_hold_s=0.35, navigation_constant=3.0, max_accel=15.0),
            last_valid_accel=None,
            last_valid_age_s=float("inf"),
        )

        self.assertTrue(valid)
        self.assertEqual(source, "not_closing_abs_png")
        self.assertGreater(float(np.linalg.norm(accel)), 0.0)

    def test_not_closing_hold_last_respects_age(self):
        rel_pos = np.array([20.0, 10.0, 0.0])
        rel_vel = np.array([4.0, 0.0, 0.0])
        result = compute_truth_png(rel_pos, rel_vel, navigation_constant=3.0, max_accel=15.0)
        last = np.array([1.0, 2.0, 3.0])
        args = SimpleNamespace(not_closing_mode="hold_last", not_closing_hold_s=0.35, navigation_constant=3.0, max_accel=15.0)

        accel, valid, source = _png_accel_for_not_closing(
            result,
            rel_pos,
            rel_vel,
            args,
            last_valid_accel=last,
            last_valid_age_s=0.2,
        )
        self.assertTrue(valid)
        self.assertEqual(source, "not_closing_hold_last")
        self.assertTrue(np.allclose(accel, last))

        accel, valid, source = _png_accel_for_not_closing(
            result,
            rel_pos,
            rel_vel,
            args,
            last_valid_accel=last,
            last_valid_age_s=0.5,
        )
        self.assertFalse(valid)
        self.assertEqual(source, "not_closing_hold_expired")
        self.assertTrue(np.allclose(accel, np.zeros(3)))

    def test_motor_saturation_detection(self):
        self.assertFalse(_any_motor_saturated((0.2, 0.3, 0.4, 0.5)))
        self.assertTrue(_any_motor_saturated((0.055, 0.3, 0.4, 0.5)))
        self.assertTrue(_any_motor_saturated((0.2, 0.3, 0.4, 1.0)))


if __name__ == "__main__":
    unittest.main()
