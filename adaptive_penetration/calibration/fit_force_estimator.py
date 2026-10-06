#!/usr/bin/env python3
"""Offline calibration/validation utility for the joint-torque axial-force estimator.

The implementation follows Section 2.3.1 of the manuscript:
- no-contact torque referencing (Eq. 20/23)
- all-joint zero-intercept linear screening
- signed Joint-3 / Joint-4 responses
- zero-intercept order-1/order-2 Huber robust regression (Eq. 21)
- monotonicity and >=5% force-domain LOOCV improvement rule
- covariance-weighted fusion (Eq. 25)
- nested LOOCV evaluation

The Huber tuning constant is configurable; the default value is 1.345.
"""

import argparse
import csv
import json
import math
import os
import statistics
from dataclasses import dataclass, asdict
from typing import Dict, List, Tuple, Sequence

EPS = 1e-12


@dataclass
class ModelFit:
    order: int
    beta1: float
    beta2: float
    monotonic: bool
    loocv_rmse_n: float
    calibration_r2: float
    loocv_out_of_range_count: int


@dataclass
class InversionResult:
    estimate_n: float
    within_calibration_range: bool
    real_solution: bool


@dataclass
class CalibrationResult:
    reference_torques_nm: List[float]
    screening: List[dict]
    joint3: ModelFit
    joint4: ModelFit
    covariance: List[List[float]]
    fusion_weights: List[float]
    nested_loocv_r2: float
    nested_loocv_rmse_n: float
    nested_predictions: List[dict]
    huber_k: float
    calibration_force_max_n: float


def _mean(xs: Sequence[float]) -> float:
    return sum(xs) / len(xs)


def _sd(xs: Sequence[float]) -> float:
    if len(xs) < 2:
        return 0.0
    m = _mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def _median(xs: Sequence[float]) -> float:
    return statistics.median(xs)


def _mad_scale(residuals: Sequence[float]) -> float:
    if not residuals:
        return 1.0
    med = _median(residuals)
    mad = _median([abs(r - med) for r in residuals])
    scale = mad / 0.6744897501960817 if mad > EPS else 0.0
    if scale <= EPS:
        rms = math.sqrt(sum(r * r for r in residuals) / max(1, len(residuals)))
        scale = rms if rms > EPS else 1.0
    return scale


def _solve_wls(forces: Sequence[float], responses: Sequence[float], weights: Sequence[float], order: int) -> Tuple[float, float]:
    if order == 1:
        den = sum(w * f * f for f, w in zip(forces, weights))
        if abs(den) <= EPS:
            raise ValueError("Degenerate order-1 regression design.")
        num = sum(w * f * y for f, y, w in zip(forces, responses, weights))
        return num / den, 0.0
    if order != 2:
        raise ValueError("Only model orders 1 and 2 are supported.")

    a11 = sum(w * f ** 2 for f, w in zip(forces, weights))
    a12 = sum(w * f ** 3 for f, w in zip(forces, weights))
    a22 = sum(w * f ** 4 for f, w in zip(forces, weights))
    b1 = sum(w * f * y for f, y, w in zip(forces, responses, weights))
    b2 = sum(w * f ** 2 * y for f, y, w in zip(forces, responses, weights))
    det = a11 * a22 - a12 * a12
    if abs(det) <= EPS:
        raise ValueError("Degenerate order-2 regression design.")
    beta1 = (b1 * a22 - b2 * a12) / det
    beta2 = (a11 * b2 - a12 * b1) / det
    return beta1, beta2


def huber_fit(forces: Sequence[float], responses: Sequence[float], order: int, huber_k: float = 1.345,
              max_iter: int = 100, tol: float = 1e-12) -> Tuple[float, float]:
    weights = [1.0] * len(forces)
    beta1, beta2 = _solve_wls(forces, responses, weights, order)
    for _ in range(max_iter):
        residuals = [y - (beta1 * f + beta2 * f * f) for f, y in zip(forces, responses)]
        scale = _mad_scale(residuals)
        c = huber_k * scale
        new_weights = []
        for r in residuals:
            ar = abs(r)
            new_weights.append(1.0 if ar <= c or c <= EPS else c / ar)
        nb1, nb2 = _solve_wls(forces, responses, new_weights, order)
        if max(abs(nb1 - beta1), abs(nb2 - beta2)) < tol:
            beta1, beta2 = nb1, nb2
            break
        beta1, beta2 = nb1, nb2
        weights = new_weights
    return beta1, beta2


def model_monotonic(beta1: float, beta2: float, order: int, force_max: float) -> bool:
    if order == 1:
        return beta1 > 0.0
    # derivative beta1 + 2 beta2 F is linear, so endpoints are sufficient.
    return beta1 > 0.0 and (beta1 + 2.0 * beta2 * force_max) > 0.0


def invert_response(response: float, beta1: float, beta2: float, order: int, force_max: float) -> InversionResult:
    """Invert a monotonic calibration relation without silently clipping the result.

    LOOCV errors must be calculated from the mathematical inverse, not from a value
    projected onto [0, force_max].  The range flag is reported separately so that an
    extrapolative prediction remains visible to the caller.
    """
    if order == 1 or abs(beta2) <= EPS:
        if beta1 <= EPS:
            raise ValueError("Non-positive linear coefficient cannot be inverted monotonically.")
        f = response / beta1
        return InversionResult(f, -1e-9 <= f <= force_max + 1e-9, True)

    disc = beta1 * beta1 + 4.0 * beta2 * response
    if disc < -1e-12:
        return InversionResult(float("nan"), False, False)
    disc = max(0.0, disc)
    sqrt_disc = math.sqrt(disc)

    # This is the inverse branch continuous from F=0 when beta1>0.  It is
    # algebraically equivalent to (-beta1 + sqrt(disc))/(2*beta2) but is
    # numerically better behaved near beta2=0.
    denom = beta1 + sqrt_disc
    if abs(denom) > EPS:
        f = 2.0 * response / denom
    else:
        f = (-beta1 + sqrt_disc) / (2.0 * beta2)
    return InversionResult(f, -1e-9 <= f <= force_max + 1e-9, math.isfinite(f))


def group_load_levels(rows: List[dict], ref: Sequence[float], directions: Dict[int, float]) -> List[dict]:
    by_force: Dict[float, List[dict]] = {}
    for row in rows:
        by_force.setdefault(row["force_n"], []).append(row)
    grouped = []
    for force in sorted(by_force):
        reps = by_force[force]
        item = {"force_n": force, "n": len(reps), "means": [], "sds": [], "signed": {}}
        for j in range(6):
            dts = [r["tau"][j] - ref[j] for r in reps]
            item["means"].append(_mean(dts))
            item["sds"].append(_sd(dts))
            if j in directions:
                item["signed"][j] = directions[j] * _mean(dts)
        grouped.append(item)
    return grouped


def zero_intercept_screening(grouped: Sequence[dict]) -> List[dict]:
    out = []
    forces = [g["force_n"] for g in grouped]
    for j in range(6):
        ys = [g["means"][j] for g in grouped]
        den = sum(f * f for f in forces)
        beta = sum(f * y for f, y in zip(forces, ys)) / den if den > EPS else 0.0
        preds = [beta * f for f in forces]
        ybar = _mean(ys)
        sse = sum((y - p) ** 2 for y, p in zip(ys, preds))
        sst = sum((y - ybar) ** 2 for y in ys)
        r2 = 1.0 - sse / sst if sst > EPS else 1.0
        diffs = [ys[i + 1] - ys[i] for i in range(len(ys) - 1)]
        nondec = sum(d >= -1e-12 for d in diffs)
        noninc = sum(d <= 1e-12 for d in diffs)
        monotonic_fraction = max(nondec, noninc) / max(1, len(diffs))
        out.append({
            "joint": j,
            "linear_beta": beta,
            "r2": r2,
            "monotonic_fraction": monotonic_fraction,
        })
    return out


def loocv_force_predictions(grouped: Sequence[dict], joint: int, direction: float, order: int,
                            huber_k: float, force_max: float) -> List[dict]:
    preds = []
    if len(grouped) < 4:
        raise ValueError("At least four load levels are required for LOOCV.")
    for i, test in enumerate(grouped):
        train = [g for k, g in enumerate(grouped) if k != i]
        forces = [g["force_n"] for g in train]
        resp = [direction * g["means"][joint] for g in train]
        b1, b2 = huber_fit(forces, resp, order, huber_k)
        test_resp = direction * test["means"][joint]
        inv = invert_response(test_resp, b1, b2, order, force_max)
        residual = inv.estimate_n - test["force_n"] if inv.real_solution else float("nan")
        preds.append({
            "force_n": test["force_n"],
            "estimate_n": inv.estimate_n,
            "residual_n": residual,
            "within_calibration_range": inv.within_calibration_range,
            "real_solution": inv.real_solution,
        })
    return preds


def rmse(preds: Sequence[dict]) -> float:
    residuals = [p["residual_n"] for p in preds]
    if any(not math.isfinite(r) for r in residuals):
        return float("inf")
    return math.sqrt(_mean([r ** 2 for r in residuals]))


def response_domain_r2(forces: Sequence[float], responses: Sequence[float], beta1: float, beta2: float) -> float:
    preds = [beta1 * f + beta2 * f * f for f in forces]
    ybar = _mean(responses)
    sse = sum((y - p) ** 2 for y, p in zip(responses, preds))
    sst = sum((y - ybar) ** 2 for y in responses)
    return 1.0 - sse / sst if sst > EPS else 1.0


def quadratic_is_preferred(monotonic2: bool, rmse1: float, rmse2: float) -> bool:
    # Manuscript criterion: at least 5% lower force-domain LOOCV RMSE.
    return monotonic2 and math.isfinite(rmse2) and rmse2 <= 0.95 * rmse1


def select_order(grouped: Sequence[dict], joint: int, direction: float, huber_k: float, force_max: float) -> ModelFit:
    p1 = loocv_force_predictions(grouped, joint, direction, 1, huber_k, force_max)
    rmse1 = rmse(p1)
    f = [g["force_n"] for g in grouped]
    y = [direction * g["means"][joint] for g in grouped]
    b11, b12 = huber_fit(f, y, 1, huber_k)
    monotonic1 = model_monotonic(b11, b12, 1, force_max)

    p2 = loocv_force_predictions(grouped, joint, direction, 2, huber_k, force_max)
    rmse2 = rmse(p2)
    b21, b22 = huber_fit(f, y, 2, huber_k)
    monotonic2 = model_monotonic(b21, b22, 2, force_max)

    # Manuscript rule: retain quadratic only if monotonic over the calibration range
    # and force-domain LOOCV RMSE is at least 5% lower than the linear model.
    choose2 = quadratic_is_preferred(monotonic2, rmse1, rmse2)
    if choose2:
        return ModelFit(
            2, b21, b22, monotonic2, rmse2,
            response_domain_r2(f, y, b21, b22),
            sum(not p["within_calibration_range"] for p in p2),
        )
    return ModelFit(
        1, b11, 0.0, monotonic1, rmse1,
        response_domain_r2(f, y, b11, 0.0),
        sum(not p["within_calibration_range"] for p in p1),
    )


def covariance2(xs: Sequence[float], ys: Sequence[float]) -> List[List[float]]:
    if len(xs) != len(ys) or len(xs) < 2:
        raise ValueError("At least two paired residuals are required for covariance.")
    mx, my = _mean(xs), _mean(ys)
    n = len(xs)
    vxx = sum((x - mx) ** 2 for x in xs) / (n - 1)
    vyy = sum((y - my) ** 2 for y in ys) / (n - 1)
    vxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (n - 1)
    return [[vxx, vxy], [vxy, vyy]]


def covariance_weights(cov: Sequence[Sequence[float]]) -> Tuple[float, float]:
    a, b = cov[0][0], cov[0][1]
    c, d = cov[1][0], cov[1][1]
    # small ridge only for numerical singularity; does not materially alter a regular covariance matrix
    scale = max(abs(a), abs(d), 1.0)
    ridge = 1e-12 * scale
    a += ridge
    d += ridge
    det = a * d - b * c
    if abs(det) <= EPS:
        invv3 = 1.0 / max(a, EPS)
        invv4 = 1.0 / max(d, EPS)
        s = invv3 + invv4
        return invv3 / s, invv4 / s
    inv00, inv01 = d / det, -b / det
    inv10, inv11 = -c / det, a / det
    z0 = inv00 + inv01
    z1 = inv10 + inv11
    denom = z0 + z1
    if abs(denom) <= EPS:
        return 0.5, 0.5
    return z0 / denom, z1 / denom


def fit_selected(grouped: Sequence[dict], joint: int, direction: float, model: ModelFit,
                 huber_k: float) -> Tuple[float, float]:
    f = [g["force_n"] for g in grouped]
    y = [direction * g["means"][joint] for g in grouped]
    return huber_fit(f, y, model.order, huber_k)


def residuals_for_order(grouped: Sequence[dict], joint: int, direction: float, order: int,
                        huber_k: float, force_max: float) -> List[float]:
    return [p["residual_n"] for p in loocv_force_predictions(grouped, joint, direction, order, huber_k, force_max)]


def nested_loocv(grouped: Sequence[dict], directions: Dict[int, float], huber_k: float,
                 force_max: float) -> List[dict]:
    outer_predictions = []
    for outer_i, test in enumerate(grouped):
        train = [g for i, g in enumerate(grouped) if i != outer_i]
        m3 = select_order(train, 3, directions[3], huber_k, force_max)
        m4 = select_order(train, 4, directions[4], huber_k, force_max)
        b31, b32 = fit_selected(train, 3, directions[3], m3, huber_k)
        b41, b42 = fit_selected(train, 4, directions[4], m4, huber_k)
        r3 = residuals_for_order(train, 3, directions[3], m3.order, huber_k, force_max)
        r4 = residuals_for_order(train, 4, directions[4], m4.order, huber_k, force_max)
        cov = covariance2(r3, r4)
        w3, w4 = covariance_weights(cov)
        s3 = directions[3] * test["means"][3]
        s4 = directions[4] * test["means"][4]
        inv3 = invert_response(s3, b31, b32, m3.order, force_max)
        inv4 = invert_response(s4, b41, b42, m4.order, force_max)
        if not inv3.real_solution or not inv4.real_solution:
            raise ValueError(
                f"Nested LOOCV fold at force={test['force_n']} N produced no real inverse solution."
            )
        f3 = inv3.estimate_n
        f4 = inv4.estimate_n
        fused = w3 * f3 + w4 * f4
        outer_predictions.append({
            "force_n": test["force_n"],
            "joint3_estimate_n": f3,
            "joint4_estimate_n": f4,
            "fused_estimate_n": fused,
            "residual_n": fused - test["force_n"],
            "joint3_order": m3.order,
            "joint4_order": m4.order,
            "joint3_within_calibration_range": inv3.within_calibration_range,
            "joint4_within_calibration_range": inv4.within_calibration_range,
            "w3": w3,
            "w4": w4,
        })
    return outer_predictions


def r2_and_rmse(preds: Sequence[dict]) -> Tuple[float, float]:
    ys = [p["force_n"] for p in preds]
    ph = [p["fused_estimate_n"] for p in preds]
    ybar = _mean(ys)
    sse = sum((a - b) ** 2 for a, b in zip(ys, ph))
    sst = sum((a - ybar) ** 2 for a in ys)
    r2 = 1.0 - sse / sst if sst > EPS else 1.0
    return r2, math.sqrt(sse / len(ys))


def read_dataset(path: str) -> List[dict]:
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required = ["force_n"] + [f"tau{j}_nm" for j in range(6)]
        missing = [c for c in required if c not in (reader.fieldnames or [])]
        if missing:
            raise ValueError("Missing required CSV columns: " + ", ".join(missing))
        for row_data in reader:
            force_n = float(row_data["force_n"])
            joint_torques_nm = [float(row_data[f"tau{j}_nm"]) for j in range(6)]
            if not all(math.isfinite(value) for value in [force_n] + joint_torques_nm):
                raise ValueError("Dataset contains NaN/inf values.")
            rows.append({
                "force_n": force_n,
                "tau": joint_torques_nm,
                "replicate": row_data.get("replicate", ""),
            })
    if not rows:
        raise ValueError("Calibration CSV is empty.")
    return rows


def calibrate(rows: List[dict], huber_k: float = 1.345, zero_tol: float = 1e-9) -> Tuple[CalibrationResult, List[dict]]:
    zero = [r for r in rows if abs(r["force_n"]) <= zero_tol]
    if not zero:
        raise ValueError("No zero-load rows were found to establish no-contact reference torques.")
    ref = [_mean([r["tau"][j] for r in zero]) for j in range(6)]
    directions = {3: 1.0, 4: -1.0}
    grouped = group_load_levels(rows, ref, directions)
    force_max = max(g["force_n"] for g in grouped)
    screening = zero_intercept_screening(grouped)

    m3 = select_order(grouped, 3, directions[3], huber_k, force_max)
    m4 = select_order(grouped, 4, directions[4], huber_k, force_max)
    b31, b32 = fit_selected(grouped, 3, directions[3], m3, huber_k)
    b41, b42 = fit_selected(grouped, 4, directions[4], m4, huber_k)
    m3.beta1, m3.beta2 = b31, b32
    m4.beta1, m4.beta2 = b41, b42
    full_forces = [g["force_n"] for g in grouped]
    y3 = [directions[3] * g["means"][3] for g in grouped]
    y4 = [directions[4] * g["means"][4] for g in grouped]
    m3.calibration_r2 = response_domain_r2(full_forces, y3, b31, b32)
    m4.calibration_r2 = response_domain_r2(full_forces, y4, b41, b42)

    r3 = residuals_for_order(grouped, 3, directions[3], m3.order, huber_k, force_max)
    r4 = residuals_for_order(grouped, 4, directions[4], m4.order, huber_k, force_max)
    cov = covariance2(r3, r4)
    w3, w4 = covariance_weights(cov)

    nested = nested_loocv(grouped, directions, huber_k, force_max)
    r2, nested_rmse = r2_and_rmse(nested)

    result = CalibrationResult(
        reference_torques_nm=ref,
        screening=screening,
        joint3=m3,
        joint4=m4,
        covariance=cov,
        fusion_weights=[w3, w4],
        nested_loocv_r2=r2,
        nested_loocv_rmse_n=nested_rmse,
        nested_predictions=nested,
        huber_k=huber_k,
        calibration_force_max_n=force_max,
    )
    return result, grouped


def write_outputs(result: CalibrationResult, grouped: Sequence[dict], outdir: str) -> None:
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "calibration_summary.json"), "w", encoding="utf-8") as f:
        json.dump(asdict(result), f, indent=2)

    with open(os.path.join(outdir, "load_level_summary.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        header = ["force_n", "n"]
        for j in range(6):
            header += [f"dtau{j}_mean_nm", f"dtau{j}_sd_nm"]
        w.writerow(header)
        for g in grouped:
            row = [g["force_n"], g["n"]]
            for j in range(6):
                row += [g["means"][j], g["sds"][j]]
            w.writerow(row)

    with open(os.path.join(outdir, "nested_loocv_predictions.csv"), "w", newline="", encoding="utf-8") as f:
        fields = ["force_n", "joint3_estimate_n", "joint4_estimate_n", "fused_estimate_n", "residual_n",
                  "joint3_order", "joint4_order", "joint3_within_calibration_range",
                  "joint4_within_calibration_range", "w3", "w4"]
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in result.nested_predictions:
            w.writerow(row)

    with open(os.path.join(outdir, "estimator_model_config.txt"), "w", encoding="utf-8") as f:
        f.write("# Generated from the supplied calibration dataset.\n")
        f.write("# Generated model parameters for the C++ estimator interface.\n")
        f.write(f"joint3_reference_torque_nm = {result.reference_torques_nm[3]:.12g}\n")
        f.write(f"joint4_reference_torque_nm = {result.reference_torques_nm[4]:.12g}\n")
        f.write(f"joint3_order = {result.joint3.order}\n")
        f.write(f"joint3_linear_coefficient = {result.joint3.beta1:.12g}\n")
        f.write(f"joint3_quadratic_coefficient = {result.joint3.beta2:.12g}\n")
        f.write(f"joint3_calibration_r2 = {result.joint3.calibration_r2:.12g}\n")
        f.write(f"joint3_loocv_out_of_range_count = {result.joint3.loocv_out_of_range_count}\n")
        f.write(f"joint4_order = {result.joint4.order}\n")
        f.write(f"joint4_linear_coefficient = {-result.joint4.beta1:.12g}\n")
        f.write(f"joint4_calibration_r2 = {result.joint4.calibration_r2:.12g}\n")
        f.write(f"joint4_loocv_out_of_range_count = {result.joint4.loocv_out_of_range_count}\n")
        f.write(f"joint3_weight = {result.fusion_weights[0]:.12g}\n")
        f.write(f"joint4_weight = {result.fusion_weights[1]:.12g}\n")
        f.write(f"calibration_force_max_n = {result.calibration_force_max_n:.12g}\n")
        f.write(f"nested_loocv_r2 = {result.nested_loocv_r2:.12g}\n")
        f.write(f"nested_loocv_rmse_n = {result.nested_loocv_rmse_n:.12g}\n")


def main() -> None:
    ap = argparse.ArgumentParser(description="Fit and validate the joint-torque axial-force estimator.")
    ap.add_argument("csv", help="Calibration CSV with force_n and tau0_nm..tau5_nm columns.")
    ap.add_argument("--output-dir", default="calibration_output", help="Directory for reports.")
    ap.add_argument("--huber-k", type=float, default=1.345,
                    help="Huber tuning constant (default: 1.345).")
    args = ap.parse_args()
    rows = read_dataset(args.csv)
    result, grouped = calibrate(rows, args.huber_k)
    write_outputs(result, grouped, args.output_dir)

    print("Calibration complete")
    print(f"load_levels: {len(grouped)}")
    print(f"force_range_N: 0 .. {result.calibration_force_max_n:.6f}")
    print(f"J3: order={result.joint3.order}, beta1={result.joint3.beta1:.9f}, beta2={result.joint3.beta2:.9f}, calibration_R2={result.joint3.calibration_r2:.4f}, LOOCV_RMSE_N={result.joint3.loocv_rmse_n:.4f}, out_of_range={result.joint3.loocv_out_of_range_count}")
    print(f"J4(signed): order={result.joint4.order}, beta1={result.joint4.beta1:.9f}, beta2={result.joint4.beta2:.9f}, calibration_R2={result.joint4.calibration_r2:.4f}, LOOCV_RMSE_N={result.joint4.loocv_rmse_n:.4f}, out_of_range={result.joint4.loocv_out_of_range_count}")
    print(f"fusion_weights: w3={result.fusion_weights[0]:.4f}, w4={result.fusion_weights[1]:.4f}")
    print(f"nested_LOOCV: R2={result.nested_loocv_r2:.4f}, RMSE_N={result.nested_loocv_rmse_n:.4f}")
    oor = result.joint3.loocv_out_of_range_count + result.joint4.loocv_out_of_range_count
    if oor:
        print(f"note: {oor} single-joint LOOCV prediction(s) were outside the calibration range; values were reported without clipping")
    print(f"reports: {args.output_dir}")


if __name__ == "__main__":
    main()
