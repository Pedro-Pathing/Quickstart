#!/usr/bin/env python3
"""Builds the BIOBUZZ robot guide.

  python3 doc/guide/build.py          # index.html (interactive) from the live config
  python3 doc/guide/build.py --pdf    # also BIOBUZZ-Robot-Master-Guide.pdf from print.html

index.html embeds config.yaml, config-docs.yaml and hwmap.yaml as they are right now, so every
value, description and status it shows matches the robot. Re-run it after changing the config.
Needs PyYAML (pip install pyyaml); --pdf also needs pypdf and Google Chrome.
"""

import argparse
import datetime
import fnmatch
import json
import pathlib
import re
import shutil
import subprocess
import sys

import yaml

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent.parent
TEAMCODE = ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
CONFIG = TEAMCODE / "robot/config/config.yaml"
DOCS = TEAMCODE / "robot/config/config-docs.yaml"
HWMAP = TEAMCODE / "hwmap.yaml"

# Status of every config key, first matching glob wins. Codes:
#   M measure on the robot   H placeholder to replace   T tuned, leave
#   P policy choice          S on/off switch           F field geometry
#   I model only (not used for live aim)
# "by" names the guide procedure that sets the key. A key matching no rule is reported by the
# build, so a new key cannot silently go without a status.
STATUS = [
    ("meta.*", "F", None),
    ("field.*", "F", None),
    ("hive.up_height_threshold_in", "F", None),
    ("hive.*", "P", None),
    ("match.auto_ms", "F", None),
    ("match.teleop_ms", "F", None),
    ("match.flower_unlock_remaining_ms", "F", None),
    ("match.*", "P", None),
    ("inventory.capacity_count", "F", None),
    ("inventory.preload_count", "F", None),
    ("inventory.entry_sensor_enabled", "S", None),
    ("inventory.entry_sensor_active_low", "S", None),
    ("inventory.*", "M", None),
    ("pedro.motion.max_path_speed", "P", "pedro"),
    ("pedro.*", "M", "pedro"),
    ("turret.enabled", "S", None),
    ("turret.servo_direction_inverted", "S", "turret-plant"),
    ("turret.stall.enabled", "S", None),
    ("turret.runaway.enabled", "S", None),
    ("turret.analog_encoder.enabled", "S", None),
    ("turret.analog_encoder.inverted", "S", "turret-center"),
    ("turret.max_power_output", "P", None),
    ("turret.ks_*", "M", "turret-plant"),
    ("turret.kv", "M", "turret-plant"),
    ("turret.pidf.*", "M", "turret-plant"),
    ("turret.orientation.*", "M", "turret-verify"),
    ("turret.travel.*", "M", "turret-center"),
    ("turret.analog_encoder.*", "M", "turret-center"),
    ("turret.pivot_*", "M", "measure"),
    ("turret.*", "P", None),
    ("vision.enabled", "S", None),
    ("vision.continuous", "S", None),
    ("vision.camera_name", "M", "vision"),
    ("vision.camera.*", "H", "vision"),
    ("vision.*", "P", None),
    ("sentinel.robot_width", "M", "measure"),
    ("sentinel.rotation_lookahead_time", "M", "friction"),
    ("sentinel.protect_full_hive_footprint", "S", None),
    ("sentinel.frame_base_width_in", "F", None),
    ("sentinel.geometry_epsilon_in", "F", None),
    ("sentinel.*", "P", None),
    ("casablanca.enable_*", "S", None),
    ("casablanca.smoothing.teleop_enabled", "S", None),
    ("casablanca.smoothing.auto_enabled", "S", None),
    ("casablanca.heading_lock.enabled", "S", None),
    ("casablanca.friction.*", "T", "friction"),
    ("casablanca.smoothing.max_loop_dt", "P", None),
    ("casablanca.smoothing.*", "M", "podcontact"),
    ("casablanca.heading_lock.pid.*", "M", "headinglock"),
    ("casablanca.heading_lock.ks", "M", "headinglock"),
    ("casablanca.heading_lock.ks_moving", "M", "friction"),
    ("casablanca.heading_lock.*", "P", None),
    ("casablanca.heading_tuner.*", "P", "headinglock"),
    ("casablanca.jump_tuner.*", "P", "podcontact"),
    ("casablanca.*", "P", None),
    ("shooter.use_ftc_pid", "S", None),
    ("shooter.ks", "M", "characterize"),
    ("shooter.nominal_voltage", "M", "characterize"),
    ("shooter.max_rpm", "M", "sanity"),
    ("shooter.pidf.*", "M", "characterize"),
    ("shooter.motor_pidf.*", "M", None),
    ("shooter.kick.wheel_tau_sec", "M", "characterize"),
    ("shooter.kick.sensor_lag_sec", "M", "characterize"),
    ("shooter.kick.*", "T", None),
    ("shooter.ball_detection.*", "T", "timing"),
    ("shooter.integral.*", "T", None),
    ("shooter.shot_table.calibrated", "S", "ballistics"),
    ("shooter.shot_table.nectar_enabled", "S", None),
    ("shooter.shot_table.*", "H", "ballistics"),
    ("shooter.ballistics.min_hood_servo_pos", "M", "hood"),
    ("shooter.ballistics.max_hood_servo_pos", "M", "hood"),
    ("shooter.ballistics.flight_time_*", "H", None),
    ("shooter.ballistics.v0", "I", None),
    ("shooter.ballistics.k", "I", None),
    ("shooter.ballistics.magnus_l", "I", None),
    ("shooter.ballistics.launch_height_inches", "I", "measure"),
    ("shooter.ballistics.g_inches_per_sec2", "I", None),
    ("shooter.ballistics.min_hood_angle_deg", "I", "hood"),
    ("shooter.ballistics.max_hood_angle_deg", "I", "hood"),
    ("shooter.ballistics.v0_reference_rpm", "I", None),
    ("shooter.ballistics.v0_margin_fraction", "I", None),
    ("shooter.*", "P", None),
    ("auto.shot_time_points", "H", "timing"),
    ("auto.cell_*", "S", None),
    ("auto.*", "P", None),
    ("teleop.field_centric", "S", None),
    ("teleop.flower_hold_enabled", "S", None),
    ("teleop.poses.*.flower", "P", "poses"),
    ("teleop.poses.*", "M", "poses"),
    ("teleop.*", "P", None),
    ("auto_poses.*", "M", "poses"),
]

KEY_LINE = re.compile(r"^(\s*)([A-Za-z0-9_]+):(.*)$")


def leaves(node, prefix=""):
    if isinstance(node, dict):
        for k, v in node.items():
            yield from leaves(v, f"{prefix}.{k}" if prefix else str(k))
    else:
        yield prefix, node


def yaml_comments(path):
    """Maps each key path to the comment lines directly above it (and any trailing comment)."""
    out, stack, pending = {}, [], []
    for line in path.read_text().splitlines():
        stripped = line.strip()
        if stripped.startswith("#"):
            text = stripped.lstrip("#").strip()
            if text and not text.startswith("yaml-language-server"):
                pending.append(text)
            continue
        if not stripped:
            continue
        m = KEY_LINE.match(line)
        if not m:
            pending = []  # a flow-list row; comments above it belong to no key
            continue
        indent, key, rest = len(m.group(1)), m.group(2), m.group(3)
        while stack and stack[-1][0] >= indent:
            stack.pop()
        stack.append((indent, key))
        full = ".".join(k for _, k in stack)
        notes = list(pending)
        if "#" in rest and not rest.strip().startswith(("'", '"')):
            trailing = rest.split("#", 1)[1].strip()
            if trailing:
                notes.append(trailing)
        if notes:
            out[full] = " ".join(notes)
        pending = []
    return out


def status_for(path):
    for pattern, code, by in STATUS:
        if fnmatch.fnmatchcase(path, pattern):
            return code, by
    return None, None


def git_rev():
    try:
        return subprocess.run(
            ["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def build_data():
    cfg = yaml.safe_load(CONFIG.read_text())
    docs = yaml.safe_load(DOCS.read_text()) or {}
    hw = yaml.safe_load(HWMAP.read_text())
    comments = yaml_comments(CONFIG)
    size = float(cfg["field"]["size_in"])

    keys, unmatched = [], []
    for path, value in leaves(cfg):
        code, by = status_for(path)
        if code is None:
            unmatched.append(path)
        parts = path.split(".")
        doc = docs.get(parts[-1]) or {}
        parent = next((docs[p] for p in reversed(parts[:-1]) if isinstance(docs.get(p), dict)), {})
        entry = {
            "path": path,
            "value": value,
            "status": code or "P",
            "by": by,
            "desc": (doc.get("desc") or "").strip(),
            "parentDesc": (parent.get("desc") or "").strip() if not doc.get("desc") else "",
            "min": doc.get("min"),
            "max": doc.get("max"),
            "note": comments.get(path, ""),
        }
        if value == "m":
            red = cfg
            for p in [("red" if q == "blue" else q) for q in parts]:
                red = red.get(p) if isinstance(red, dict) else None
            if isinstance(red, list) and len(red) >= 3:
                x, y, h = red[:3]
                entry["mirror"] = [round(size - x, 3), round(size - y, 3), round((h + 180) % 360, 3)]
        keys.append(entry)

    if unmatched:
        print("warning: no status rule for:", ", ".join(unmatched), file=sys.stderr)

    # Section comments (the block above e.g. "shooter:") for the explorer's group headers.
    sections = {k: comments.get(k, "") for k in cfg}
    return {
        "config": cfg,
        "keys": keys,
        "sections": sections,
        "comments": comments,
        "hwmap": hw,
        "built": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
        "rev": git_rev(),
        "configPath": str(CONFIG.relative_to(ROOT)),
    }


def build_app():
    data = build_data()
    template = (HERE / "app.src.html").read_text()
    blob = json.dumps(data, separators=(",", ":"), default=str).replace("</", "<\\/")
    out = template.replace("/*__GUIDE_DATA__*/null", blob)
    (HERE / "index.html").write_text(out)
    print(f"wrote {HERE / 'index.html'} ({len(data['keys'])} config keys, rev {data['rev']})")


def find_chrome():
    for c in ("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
              shutil.which("google-chrome") or "", shutil.which("chromium") or ""):
        if c and pathlib.Path(c).exists():
            return c
    sys.exit("Chrome not found; pass --chrome PATH")


def render_pdf(chrome, html, pdf):
    subprocess.run([chrome, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                    "--generate-pdf-document-outline", f"--print-to-pdf={pdf}", html.as_uri()],
                   capture_output=True, check=False)


def build_pdf(chrome):
    from pypdf import PdfReader  # only needed for the PDF

    src = (HERE / "print.html").read_text()
    tmp = HERE / ".print-pass.html"
    first = HERE / ".pass1.pdf"
    out = HERE / "BIOBUZZ-Robot-Master-Guide.pdf"
    # Pass 1 finds the page of every heading; pass 2 writes those numbers into the contents.
    render_pdf(chrome, HERE / "print.html", first)
    reader = PdfReader(str(first))
    pages = {n.lstrip("/"): reader.get_destination_page_number(d) + 1
             for n, d in reader.named_destinations.items()}
    filled = re.sub(r'<span class="pg" data-for="([^"]+)"></span>',
                    lambda m: f'<span class="pg">{pages.get(m.group(1), "")}</span>', src)
    tmp.write_text(filled)
    render_pdf(chrome, tmp, out)
    tmp.unlink()
    first.unlink()
    print(f"wrote {out} ({len(PdfReader(str(out)).pages)} pages)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pdf", action="store_true", help="also rebuild the printable PDF")
    ap.add_argument("--chrome", help="path to Chrome/Chromium for --pdf")
    args = ap.parse_args()
    build_app()
    if args.pdf:
        build_pdf(args.chrome or find_chrome())


if __name__ == "__main__":
    main()
