# Robot guide

A guide to bringing up, tuning and verifying the BIOBUZZ robot, written for someone who can
deploy code but is new to robotics.

| File | What it is |
| --- | --- |
| `index.html` | **The interactive guide.** Open it in any browser; it is one self-contained file and works offline. Generated: do not edit. |
| `app.src.html` | Source of `index.html`: layout, procedures, gamepad maps, troubleshooting. Edit this. |
| `build.py` | Builds `index.html` from `app.src.html` plus the live `config.yaml`, `config-docs.yaml` and `hwmap.yaml`; `--pdf` also rebuilds the PDF. |
| `print.html` | Source of the printable PDF (a hand-written snapshot, values as of 2026-11-05). |
| `BIOBUZZ-Robot-Master-Guide.pdf` | Printable version for the pit. |

## Rebuild

```bash
pip install pyyaml pypdf
python3 doc/guide/build.py          # interactive guide only
python3 doc/guide/build.py --pdf    # also the PDF (needs Google Chrome)
```

Re-run it whenever `config.yaml` changes: the guide embeds the config at build time, so its
values, live status panel, field map, shot zones and checkers always describe the config it was
built from. The build prints a warning for any config key that has no status rule; add one to
`STATUS` in `build.py` (first matching glob wins).

## What the interactive guide does that the PDF cannot

- Search everything with ⌘K or `/`: sections, procedures, all 334 config keys, every gamepad button in every OpMode, fixes and terms.
- A live status panel computed from the config (shot table calibrated? camera measured? turret inside the proof?).
- A config explorer showing every key's value, description, limits, YAML comments, status and the procedure that sets it, with BLUE `m` poses resolved.
- An interactive field map drawn from the config. It shows poses with robot footprints, the HIVE, zones and valid shot zones. Click it to pick coordinates and copy them as a pose.
- A gamepad explorer that lights up the buttons each OpMode uses.
- Shot-table and shot-window checkers that apply the same limits the robot does, plus calculators for BLUE mirrors, turret gains, aim tolerance and flywheel units.
- Checklists that are saved in the browser (`localStorage`): bring-up progress, procedure steps, acceptance-test results (exportable as Markdown) and the pre-match list.
