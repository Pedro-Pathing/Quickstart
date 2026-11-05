#!/usr/bin/env python3
"""Does the convergence proof cover this tuning?

TurretStep.converges (Convergence.lean) holds for a plant in a box and gains meeting the Witness
conditions. This script reads the turret gains from config.yaml, searches for the witnesses
(a, b, m) at every corner of the plant box, for both directions, at a range of target distances,
and prints where the proof applies. Exit code 1 if it does not apply up to --max-distance.

Run it after changing any turret gain (pidf, ks_*, max_power_output, tolerance.*):
  python3 conditions.py [--config path/to/config.yaml] [--max-distance 140]
The plant box defaults to the one TurretClosedLoopProperties tests against; override it with the
options below once the plant is measured (Turret Plant Calibration OpMode).
"""
import argparse
import itertools
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(
    HERE, "../../src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.yaml")


def read_yaml_scalars(path):
    """Dotted key -> scalar for a plain, indentation-structured YAML file (no PyYAML needed)."""
    out, stack = {}, []
    for raw in open(path):
        line = raw.split("#", 1)[0].rstrip()
        m = re.match(r"^(\s*)([A-Za-z_][\w]*):\s*(.*)$", line)
        if not m:
            continue
        indent, key, value = len(m.group(1)), m.group(2), m.group(3).strip()
        while stack and stack[-1][0] >= indent:
            stack.pop()
        dotted = ".".join([k for _, k in stack] + [key])
        if value == "":
            stack.append((indent, key))
        else:
            out[dotted] = value.strip('"')
    return out


def number(cfg, key):
    return float(cfg[key])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    ap.add_argument("--max-distance", type=float, default=140.0,
                    help="in; the proof must apply at every distance up to this (default 140)")
    ap.add_argument("--k", type=float, nargs=2, default=[80.0, 200.0], help="servo gain range, deg/s per power")
    ap.add_argument("--lag", type=float, default=0.04, help="largest servo lag, s")
    ap.add_argument("--dt-max", type=float, default=0.026, help="longest loop period, s")
    ap.add_argument("--noise", type=float, default=0.15, help="encoder noise bound, deg")
    ap.add_argument("--friction-scale", type=float, default=1.3,
                    help="plant friction up to ks * scale + offset (calibration error)")
    ap.add_argument("--friction-offset", type=float, default=0.01)
    args = ap.parse_args()

    cfg = read_yaml_scalars(args.config)
    t = "turret."
    p, i, d, f = (number(cfg, t + "pidf." + g) for g in "pidf")
    ksP, ksN = number(cfg, t + "ks_positive"), number(cfg, t + "ks_negative")
    maxP = min(1.0, number(cfg, t + "max_power_output"))
    X = max(1.0, number(cfg, t + "tolerance.exit_factor"))
    drift, tmin, tmax = (number(cfg, t + "tolerance." + k) for k in ("max_drift", "min_deg", "max_deg"))
    trav_lo, trav_hi = number(cfg, t + "travel.min_angle"), number(cfg, t + "travel.max_angle")
    enc = t + "analog_encoder."
    z, dpv = number(cfg, enc + "zero_voltage"), number(cfg, enc + "degrees_per_volt")
    inv = cfg.get(enc + "inverted", "false").lower() == "true"
    stops = sorted(((v - z) * dpv * (-1 if inv else 1))
                   for v in (number(cfg, enc + "min_voltage"), number(cfg, enc + "max_voltage")))
    noise, dtMax = args.noise, args.dt_max

    problems = []
    if i != 0 or d != 0 or f != 0:
        problems.append(f"pidf i={i}, d={d}, f={f}: the proof assumes the autotuner's i = d = f = 0")
    if not (-100 <= stops[0] <= trav_lo and trav_hi <= stops[1] <= 100):
        problems.append(f"stops {stops[0]:.1f}..{stops[1]:.1f} deg do not enclose the travel")

    def tol_at(dist):
        return min(tmax, max(tmin, math.degrees(math.asin(min(1.0, drift / 2 / dist)))))

    def box_ok(tol):
        return 2 * noise < tol <= 10 and 2 * noise <= (X - 1) * tol and X * tol <= 20

    def witness(k, lag, s, ks, tol):
        m = min(maxP - s, p * tol + ks - s)
        if m <= 0:
            return f"drive margin {m:.4f} <= 0: friction beats P + kS at the band's edge"
        if k * dtMax * (m / 2) > tol - noise:
            return "clampOk fails"
        for a in (x / 4000 for x in range(1, 4000)):
            if a * k * lag >= 1:
                break
            c, dd = 1 + a * k * dtMax, 1 - a * k * lag
            if c * p > dd * a:
                continue
            b = max(0.0, c * (p * tol + ks) / dd - a * (tol - noise))
            if k * dtMax * b > (tol - noise) * (2 + a * k * dtMax):
                continue
            if (tol - noise) * (1 - k * lag * a) + k * lag * b > X * tol - noise:
                continue
            return None
        return "no servo bound (a, b) meets slope, edge, reach and land"

    sPmax = ksP * args.friction_scale + args.friction_offset
    sNmax = ksN * args.friction_scale + args.friction_offset
    print(f"gains p={p} ksP={ksP} ksN={ksN} maxPower={maxP} exitFactor={X}; stops "
          f"{stops[0]:.1f}..{stops[1]:.1f}; plant k {args.k[0]}..{args.k[1]}, lag <= {args.lag}, "
          f"dt <= {dtMax}, noise <= {noise}, friction up to {sPmax:.4f} / {sNmax:.4f}")
    covered_to = 0.0
    for dist in (20, 40, 60, 80, 100, 120, 140, 160, 180, 200):
        tol = tol_at(dist)
        fails = [] if box_ok(tol) else ["tolerance too tight for the noise"]
        for k, lag, sP, sN in itertools.product(args.k, (0.0, args.lag / 2, args.lag),
                                                (0.0, sPmax), (0.0, sNmax)):
            for name, s, ks in (("up", sP, ksP), ("down", sN, ksN)):
                why = witness(k, lag, s, ks, tol)
                if why:
                    fails.append(f"{name} k={k:g} lag={lag:g} friction={s:.4f}: {why}")
        print(f"  {dist:3d} in (tol {tol:.2f} deg): " +
              ("proven" if not fails else f"NOT covered, {len(fails)} corner(s), e.g. {fails[0]}"))
        if not fails and covered_to == dist - 20:
            covered_to = dist
    if covered_to < args.max_distance:
        problems.append(f"proof covers targets only up to {covered_to:g} in (need {args.max_distance:g})")
    for problem in problems:
        print("NOT COVERED:", problem)
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()
