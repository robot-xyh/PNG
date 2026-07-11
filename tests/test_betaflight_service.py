import unittest
from pathlib import Path


class BetaflightServiceTest(unittest.TestCase):
    def test_service_is_permanently_log_only_by_default(self):
        root = Path(__file__).resolve().parents[1]
        text = (root / "deploy/systemd/png-betaflight-log-only.service.in").read_text(encoding="utf-8")
        self.assertIn("--control-mode log_only", text)
        self.assertIn("--duration-s 0", text)
        self.assertIn("--detector-source rknn_bytetrack", text)
        self.assertNotIn("--allow-control", text)

    def test_installer_forces_service_disabled_and_inactive(self):
        root = Path(__file__).resolve().parents[1]
        text = (root / "tools/install_betaflight_log_only_service.sh").read_text(encoding="utf-8")
        self.assertIn('disable --now "${UNIT_NAME}"', text)
        self.assertIn("Refusing to install a service containing --allow-control", text)


if __name__ == "__main__":
    unittest.main()
