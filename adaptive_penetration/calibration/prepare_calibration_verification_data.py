#!/usr/bin/env python3
"""Prepare a deterministic calibration dataset for software verification."""
import csv
import math
import random
import sys

out = sys.argv[1] if len(sys.argv) > 1 else "calibration_verification_data.csv"
random.seed(20261005)
reference_torques_nm = [0.72, -0.31, 0.48, 1.15, -0.84, 0.23]
forces = [4.905 * i for i in range(16)]  # 0 to 73.575 N at 0.5-kg increments with g=9.81

with open(out, "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["force_n", "replicate"] + [f"tau{j}_nm" for j in range(6)])
    for force in forces:
        for rep in range(1, 4):
            noise = [random.gauss(0.0, s) for s in [0.035, 0.030, 0.025, 0.018, 0.012, 0.030]]
            # Non-selected joints: weaker responses with controlled variation and mild non-monotonicity.
            d0 = 0.0025 * force + 0.045 * math.sin(force / 9.0) + noise[0]
            d1 = -0.0035 * force + 0.030 * math.sin(force / 11.0) + noise[1]
            d2 = 0.0050 * force - 0.000050 * force * force + noise[2]
            # Selected-joint relations follow the manuscript model form with controlled perturbations.
            d3 = 0.038872 * force + 0.000347 * force * force + noise[3]
            d4 = -0.017294 * force + noise[4]
            d5 = -0.0040 * force + 0.050 * math.sin(force / 7.0) + noise[5]
            joint_torques_nm = [
                reference_torques_nm[j] + increment
                for j, increment in enumerate([d0, d1, d2, d3, d4, d5])
            ]
            w.writerow([f"{force:.6f}", rep] + [f"{torque:.9f}" for torque in joint_torques_nm])
print(out)
