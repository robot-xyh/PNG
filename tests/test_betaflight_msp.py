import struct
import unittest

import numpy as np

from vision_guidance.betaflight_msp import (
    MSP_ANALOG,
    MSP_API_VERSION,
    MSP_ATTITUDE,
    MSP_BOXNAMES,
    MSP_RAW_IMU,
    MSP_RC,
    MSP_SET_RAW_RC,
    MSP_STATUS,
    BetaflightMSPAdapter,
    MSPError,
    decode_msp_frame,
    encode_msp_frame,
    pack_rc_channels,
    parse_analog,
    parse_api_version,
    parse_attitude,
    parse_box_ids,
    parse_box_names,
    parse_rc_channels,
    parse_raw_imu,
    parse_status,
)
from vision_guidance.flight_control import RcCommand


class FakeTransport:
    def __init__(self, responses):
        self.buffer = bytearray(b"".join(responses))
        self.writes = []
        self.closed = False

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)

    def read(self, n):
        if not self.buffer:
            return b""
        count = min(n, len(self.buffer))
        data = bytes(self.buffer[:count])
        del self.buffer[:count]
        return data

    def flush(self):
        pass

    def close(self):
        self.closed = True


class BetaflightMSPTest(unittest.TestCase):
    def test_encode_decode_round_trip(self):
        frame = encode_msp_frame(MSP_API_VERSION, b"\x01\x02\x03", direction="<")

        decoded = decode_msp_frame(frame)

        self.assertEqual(decoded.direction, "<")
        self.assertEqual(decoded.command, MSP_API_VERSION)
        self.assertEqual(decoded.payload, b"\x01\x02\x03")

    def test_decode_rejects_bad_checksum(self):
        frame = bytearray(encode_msp_frame(MSP_API_VERSION, b"\x01"))
        frame[-1] ^= 0xFF

        with self.assertRaisesRegex(MSPError, "checksum"):
            decode_msp_frame(frame)

    def test_decodes_msp_v1_jumbo_frame(self):
        payload = (b"ARM;ANGLE;" * 35)[:350]
        size_raw = struct.pack("<H", len(payload))
        checksum = 255 ^ MSP_BOXNAMES
        for value in size_raw + payload:
            checksum ^= value
        frame = b"$M>" + bytes([255, MSP_BOXNAMES]) + size_raw + payload + bytes([checksum])

        decoded = decode_msp_frame(frame)

        self.assertEqual(decoded.command, MSP_BOXNAMES)
        self.assertEqual(decoded.payload, payload)

    def test_parses_common_telemetry_payloads(self):
        api = parse_api_version(b"\x00\x02\x05")
        self.assertEqual((api.protocol_version, api.api_major, api.api_minor), (0, 2, 5))

        attitude = parse_attitude(struct.pack("<hhh", 123, -45, 270))
        self.assertAlmostEqual(attitude.roll_deg, 12.3)
        self.assertAlmostEqual(attitude.pitch_deg, -4.5)
        self.assertAlmostEqual(attitude.yaw_deg, 270.0)
        self.assertEqual(attitude.R_IB.shape, (3, 3))
        self.assertTrue(np.all(np.isfinite(attitude.R_IB)))

        analog = parse_analog(bytes([121]) + struct.pack("<HHh", 345, 987, -123))
        self.assertAlmostEqual(analog.vbat_v, 12.1)
        self.assertEqual(analog.mah_drawn, 345)
        self.assertEqual(analog.rssi, 987)
        self.assertAlmostEqual(analog.amperage_a, -1.23)

        status = parse_status(struct.pack("<HHHI", 1000, 2, 3, 4) + b"\x01")
        self.assertEqual(status.cycle_time_us, 1000)
        self.assertEqual(status.mode_flags, 4)
        self.assertEqual(status.profile, 1)

        rc = parse_rc_channels(struct.pack("<HHHH", 1000, 1500, 1600, 2000))
        self.assertEqual(rc, (1000, 1500, 1600, 2000))

        self.assertEqual(parse_box_ids(bytes([0, 1, 50, 27])), (0, 1, 50, 27))
        self.assertEqual(parse_box_names(b"ARM;ANGLE;MSP OVERRIDE;"), ("ARM", "ANGLE", "MSP OVERRIDE"))

    def test_parses_signed_raw_imu_payload(self):
        raw_imu = parse_raw_imu(struct.pack("<9h", -10, 20, -30, -40, 50, -60, 70, -80, 90))

        self.assertEqual(raw_imu.acc_raw, (-10, 20, -30))
        self.assertEqual(raw_imu.gyro_deg_s, (-40.0, 50.0, -60.0))
        self.assertEqual(raw_imu.mag_raw, (70, -80, 90))
        with self.assertRaisesRegex(MSPError, "too short"):
            parse_raw_imu(b"\x00" * 17)

    def test_adapter_tracks_per_command_rtt_and_errors(self):
        response = encode_msp_frame(MSP_RAW_IMU, struct.pack("<9h", *range(9)), direction=">")
        adapter = BetaflightMSPAdapter("/dev/null", timeout_s=0.01, transport=FakeTransport([response]))

        adapter.read_raw_imu()
        success = adapter.snapshot_stats().for_command(MSP_RAW_IMU)

        self.assertIsNotNone(success)
        self.assertEqual(success.attempt_count, 1)
        self.assertEqual(success.success_count, 1)
        self.assertEqual(success.error_count, 0)
        self.assertGreaterEqual(success.last_rtt_ms, 0.0)
        self.assertIsNotNone(success.last_success_monotonic_s)

        failing = BetaflightMSPAdapter("/dev/null", timeout_s=0.01, transport=FakeTransport([]))
        with self.assertRaises(TimeoutError):
            failing.read_raw_imu()
        error = failing.snapshot_stats().for_command(MSP_RAW_IMU)
        self.assertEqual(error.attempt_count, 1)
        self.assertEqual(error.error_count, 1)
        self.assertIn("TimeoutError", error.last_error)

    def test_box_ids_rejects_empty_payload(self):
        with self.assertRaisesRegex(MSPError, "must not be empty"):
            parse_box_ids(b"")

        with self.assertRaisesRegex(MSPError, "must not be empty"):
            parse_box_names(b"")

    def test_adapter_reads_box_names(self):
        response = encode_msp_frame(MSP_BOXNAMES, b"ARM;ANGLE;", direction=">")
        adapter = BetaflightMSPAdapter("/dev/null", transport=FakeTransport([response]))

        self.assertEqual(adapter.read_box_names(), ("ARM", "ANGLE"))

    def test_adapter_reads_jumbo_box_names(self):
        payload = b";".join(f"MODE{index}".encode("ascii") for index in range(50)) + b";"
        size_raw = struct.pack("<H", len(payload))
        checksum = 255 ^ MSP_BOXNAMES
        for value in size_raw + payload:
            checksum ^= value
        response = b"$M>" + bytes([255, MSP_BOXNAMES]) + size_raw + payload + bytes([checksum])
        adapter = BetaflightMSPAdapter("/dev/null", transport=FakeTransport([response]))

        self.assertEqual(len(adapter.read_box_names()), 50)

    def test_adapter_request_writes_command_and_reads_response(self):
        response = encode_msp_frame(MSP_API_VERSION, b"\x00\x02\x01", direction=">")
        transport = FakeTransport([response])
        adapter = BetaflightMSPAdapter("/dev/null", transport=transport)

        version = adapter.read_api_version()

        self.assertEqual((version.protocol_version, version.api_major, version.api_minor), (0, 2, 1))
        self.assertEqual(transport.writes, [encode_msp_frame(MSP_API_VERSION, direction="<")])

    def test_adapter_reads_full_telemetry(self):
        responses = [
            encode_msp_frame(MSP_STATUS, struct.pack("<HHHI", 1000, 0, 7, 9) + b"\x02", direction=">"),
            encode_msp_frame(MSP_ATTITUDE, struct.pack("<hhh", 10, 20, 30), direction=">"),
            encode_msp_frame(MSP_ANALOG, bytes([120]) + struct.pack("<HHh", 10, 20, 30), direction=">"),
            encode_msp_frame(MSP_RC, struct.pack("<" + "H" * 8, *([1500] * 8)), direction=">"),
        ]
        adapter = BetaflightMSPAdapter("/dev/null", transport=FakeTransport(responses))

        telemetry = adapter.read_telemetry()

        self.assertEqual(telemetry.status.profile, 2)
        self.assertAlmostEqual(telemetry.attitude.roll_deg, 1.0)
        self.assertAlmostEqual(telemetry.analog.vbat_v, 12.0)
        self.assertEqual(len(telemetry.rc_channels), 8)

    def test_send_raw_rc_packs_channels(self):
        response = encode_msp_frame(MSP_SET_RAW_RC, b"", direction=">")
        transport = FakeTransport([response])
        adapter = BetaflightMSPAdapter("/dev/null", transport=transport)
        command = RcCommand(timestamp=1.0, channels=(1000, 1500, 1500, 1500, 1800, 1500, 1500, 1500), active=True)

        adapter.send_raw_rc(command)

        expected_payload = pack_rc_channels(command.channels)
        self.assertEqual(transport.writes, [encode_msp_frame(MSP_SET_RAW_RC, expected_payload, direction="<")])
        stats = adapter.snapshot_stats()
        self.assertEqual(stats.set_raw_rc_attempt_count, 1)
        self.assertEqual(stats.set_raw_rc_success_count, 1)
        self.assertEqual(stats.request_error_count, 0)


if __name__ == "__main__":
    unittest.main()
