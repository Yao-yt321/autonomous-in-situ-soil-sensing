#!/usr/bin/env python3
import copy
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from fit_force_estimator import (
    read_dataset,
    calibrate,
    invert_response,
    model_monotonic,
    quadratic_is_preferred,
    covariance_weights,
    nested_loocv,
    write_outputs,
)


def test_end_to_end_calibration_workflow():
    with tempfile.TemporaryDirectory() as td:
        csv_path = os.path.join(td, "calibration_verification.csv")
        subprocess.check_call(
            [sys.executable, os.path.join(HERE, "prepare_calibration_verification_data.py"), csv_path],
            stdout=subprocess.DEVNULL,
        )
        rows = read_dataset(csv_path)
        result, grouped = calibrate(rows)

        assert len(grouped) == 16
        assert result.joint3.order == 2, result.joint3
        assert result.joint4.order == 1, result.joint4
        assert abs(result.joint3.beta1 - 0.038872) < 0.01
        assert abs(result.joint3.beta2 - 0.000347) < 0.0002
        assert abs(result.joint4.beta1 - 0.017294) < 0.005
        assert math.isfinite(result.joint3.calibration_r2)
        assert math.isfinite(result.joint4.calibration_r2)
        assert result.joint3.calibration_r2 > 0.95
        assert result.joint4.calibration_r2 > 0.95
        assert abs(sum(result.fusion_weights) - 1.0) < 1e-9
        assert result.nested_loocv_rmse_n < 4.0
        assert result.nested_loocv_r2 > 0.97
        return result, grouped


def test_exact_five_percent_rule():
    assert quadratic_is_preferred(True, 10.0, 9.5)
    assert not quadratic_is_preferred(True, 10.0, 9.5000001)
    assert not quadratic_is_preferred(False, 10.0, 1.0)


def test_monotonicity_rejection():
    assert model_monotonic(0.05, 0.0002, 2, 73.575)
    assert not model_monotonic(0.05, -0.0010, 2, 73.575)
    assert not model_monotonic(-0.01, 0.0010, 2, 73.575)


def test_covariance_weight_formula():
    # Sigma=diag(4,1) -> inverse-variance weights = (0.2, 0.8)
    w3, w4 = covariance_weights([[4.0, 0.0], [0.0, 1.0]])
    assert abs(w3 - 0.2) < 1e-12, (w3, w4)
    assert abs(w4 - 0.8) < 1e-12, (w3, w4)
    assert abs(w3 + w4 - 1.0) < 1e-12


def test_out_of_range_is_not_clipped():
    force_max = 73.575
    beta = 0.02
    response = beta * 80.0
    inv = invert_response(response, beta, 0.0, 1, force_max)
    assert inv.real_solution
    assert not inv.within_calibration_range
    assert abs(inv.estimate_n - 80.0) < 1e-12, inv


def test_nested_loocv_no_outer_fold_leakage(grouped):
    directions = {3: 1.0, 4: -1.0}
    force_max = max(g["force_n"] for g in grouped)
    baseline = nested_loocv(grouped, directions, 1.345, force_max)

    changed = copy.deepcopy(grouped)
    idx = len(changed) // 2
    held_force = changed[idx]["force_n"]
    # Change only the outer test sample's responses.  For the fold that holds
    # this load level out, model selection and covariance weights must remain
    # unchanged if there is no data leakage from test to training.
    changed[idx]["means"][3] += 0.25
    changed[idx]["means"][4] -= 0.15
    modified = nested_loocv(changed, directions, 1.345, force_max)

    a = next(x for x in baseline if x["force_n"] == held_force)
    b = next(x for x in modified if x["force_n"] == held_force)
    assert a["joint3_order"] == b["joint3_order"]
    assert a["joint4_order"] == b["joint4_order"]
    assert abs(a["w3"] - b["w3"]) < 1e-12
    assert abs(a["w4"] - b["w4"]) < 1e-12
    # The prediction itself should change because the held-out response changed.
    assert abs(a["fused_estimate_n"] - b["fused_estimate_n"]) > 1e-6


def test_model_config_fields(result, grouped):
    with tempfile.TemporaryDirectory() as td:
        write_outputs(result, grouped, td)
        config_path = os.path.join(td, "estimator_model_config.txt")
        with open(config_path, "r", encoding="utf-8") as f:
            text = f.read()
        assert "joint3_reference_torque_nm =" in text
        assert "joint4_reference_torque_nm =" in text
        assert "joint3_linear_coefficient =" in text
        assert "joint4_linear_coefficient = -" in text
        assert "joint3_weight =" in text
        assert "joint4_weight =" in text
        assert "calibration_force_max_n =" in text


def main():
    result, grouped = test_end_to_end_calibration_workflow()
    test_exact_five_percent_rule()
    test_monotonicity_rejection()
    test_covariance_weight_formula()
    test_out_of_range_is_not_clipped()
    test_nested_loocv_no_outer_fold_leakage(grouped)
    test_model_config_fields(result, grouped)
    print("calibration_tool_tests: PASS")


if __name__ == "__main__":
    main()
