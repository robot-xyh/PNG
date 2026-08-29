import unittest

import numpy as np

from vision_guidance.png_eval import FixedVmGuidanceEvaluator
from vision_guidance.types import LOSEstimate, TTCState


def _los(*, valid=True, quality=0.8):
    return LOSEstimate(
        timestamp=1.0,
        lambda_I=np.array([1.0, 0.0, 0.0]),
        lambda_dot_I=np.array([0.0, 2.0, 0.0]),
        omega_los=np.array([0.0, 0.0, 2.0]),
        innovation_norm=0.1,
        quality=quality,
        valid=valid,
        reject_reason=None if valid else "innovation_reject",
    )


def _invalid_ttc():
    return TTCState(
        timestamp=1.0,
        ttc=None,
        quality=0.0,
        area_filtered=100.0,
        area_dot_filtered=0.0,
        valid=False,
        reject_reason="non_closing",
    )


class FixedVmGuidanceEvaluatorTest(unittest.TestCase):
    def test_uses_fixed_vm_cross_product_and_bypasses_ttc(self):
        evaluator = FixedVmGuidanceEvaluator(
            navigation_constant=3.0,
            fixed_vm_m_s=2.0,
            max_norm=100.0,
        )

        result = evaluator.evaluate(_los(), _invalid_ttc())

        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.g_eval, np.array([0.0, 12.0, 0.0]))
        self.assertEqual(result.quality, 0.8)

    def test_clamps_vector_norm_without_changing_direction(self):
        evaluator = FixedVmGuidanceEvaluator(3.0, 2.0, max_norm=3.0)

        result = evaluator.evaluate(_los(), _invalid_ttc())

        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.g_eval, np.array([0.0, 3.0, 0.0]))

    def test_rejects_invalid_los(self):
        evaluator = FixedVmGuidanceEvaluator(3.0, 2.0, max_norm=3.0)

        result = evaluator.evaluate(_los(valid=False), _invalid_ttc())

        self.assertFalse(result.valid)
        self.assertEqual(result.reject_reason, "innovation_reject")

    def test_rejects_nonpositive_or_nonfinite_configuration(self):
        for args in ((0.0, 2.0, 3.0), (3.0, -1.0, 3.0), (3.0, 2.0, np.inf)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                FixedVmGuidanceEvaluator(*args)


if __name__ == "__main__":
    unittest.main()
