# BIOBUZZ TeleOp Controls

One `TeleOp` OpMode serves both alliances. Poses are configured for RED in `config.yaml` (`teleop.poses.red.*`) and rotated 180 degrees about the field centre for BLUE.

## 0. INIT

| Input | Action |
| :--- | :--- |
| **B** (either gamepad) | Select RED alliance (only when no AUTO ran before this TeleOp) |
| **X** (either gamepad) | Select BLUE alliance (only when no AUTO ran before this TeleOp) |

If an AUTO ran, TeleOp inherits its alliance, final pose, HIVE state (which CELL is up, how many TIPS) and inventory. The HIVE/inventory handoff is ignored if it is more than two minutes old, so a previous match's TIPS never carry into a new one. The Driver Station also warns if an ADB-pushed `config.yaml` was ignored because it is from another season.

## 1. Driver Controls (Gamepad 1)

| Button / Input | Trigger | Action | Details |
| :--- | :--- | :--- | :--- |
| **Left Stick** | Live | Translate | Cubic response, clamped to `teleop.max_speed`, passed through `Casablanca.adjustDriveInput()` (friction compensation, HIVE-frame slow-down/repulsion, heading hold). Any deflection beyond 0.2 cancels a pose hold. |
| **Right Stick X** | Live | Rotate | Cubic response. Releasing it holds the current heading; steering also releases the face-the-CELL lock. |
| **X** | Press | Toggle "face the upward CELL" | Heading lock on the bearing to whichever CELL the HIVE tracker says is up; follows the robot as it translates and swaps CELL on a TIP. Translation stays manual. |
| **Right Trigger** | Press | Hold shooting pose | Drives to and holds `shoot_a` or `shoot_b`, whichever faces the CELL that is up right now. |
| **Left Trigger** | Press | Hold park pose | Drives to and holds `park` (partly inside the LOADING ZONE, clear of the wall). Use it when the park alert fires (see Match Cues). |
| **DPad Left** | Press | Hold FLOWER pose | Drives to and holds `flower`, with the intake at the alliance's side-wall FLOWER retrieval opening. POLLEN may only come out of the bottom of a FLOWER (G418). Does nothing while `teleop.flower_hold_enabled` is false (the current robot has no FLOWER pickup). |
| **DPad Right** | Press | Cancel pose hold | Restores manual driving. Stick deflection does the same. |

Pose holds check their direct approach against the HIVE frame and field boundary before moving. If blocked, the robot stops and `Pose assist` telemetry shows the reason. Cancel the hold, drive to a clear approach, and retry.

## 2. Operator Controls (Gamepad 2)

| Button / Input | Trigger | Action | Details |
| :--- | :--- | :--- | :--- |
| **A** | Press | Start intake | Collects at `teleop.intake_power` / `teleop.transfer_power`. Stops itself at a known count of 4 (G407), including a known full preload. An entry sensor maintains the count during collection; without one, collection below capacity makes the count unknown and the operator must stop at 4. |
| **B** | Press | Stop intake | |
| **DPad Right** | Hold | Eject | Runs the intake in reverse at `teleop.eject_power` to spit out a fifth element (G407) or opponent NECTAR (G408). |
| **Right Trigger** | Hold | Aim and shoot | Holds chassis position, spins up, aims the turret at the upward CELL and feeds only when every gate passes: flywheel in band, turret aligned, a valid and current solution, HIVE settled after a TIP. The telemetry "Shot" line names the gate that is holding. |
| **Y** | Hold | Turret aim only | Tracks the upward CELL without spinning up. |
| **X** | Hold | Rev flywheel | `teleop.manual_rev_power`, no feed. Hold X + Y to aim and pre-spin. |
| **DPad Up** | Press | Fixed-power shot | Constant flywheel speed, with the operator responsible for alignment. Feeding still requires flywheel readiness, a valid current shot solution, and a settled HIVE. Uncalibrated tables block feeding. Cancel with Left Trigger. |
| **Left Trigger** | Press | Cancel everything | Shot, aim, rev, manual shot, intake and eject. |
| **DPad Down** | Hold | Vision relocalize | Re-anchors odometry on the upward CELL opening while the HIVE is settled; requires a fresh camera frame and is rejected if the fix is more than `vision.relocalize.max_correction_in` from odometry. |
| **Left Bumper** | Press | Record a TIP | For a TIP the camera missed: flips which CELL is up and adds one to the TIP count (and the human player's NECTAR allowance). |
| **Back** | Press | Undo a TIP | Reverses the last flip and TIP count. |
| **Right Bumper** | Press | Toggle POLLEN / NECTAR | Selects which shot table the solver uses. Does nothing while `shooter.shot_table.nectar_enabled` is false (the current robot cannot handle NECTAR, so it always shoots POLLEN). |
| **Left Stick Button** | Press | Mark robot empty | Sets the inventory to 0 when the robot is visibly empty. |

## 3. Match Cues and Telemetry

- **1:00 left** — two rumble blips: NECTAR may now enter FLOWERS (G410) and the human player may enter every remaining NECTAR (G426). "NECTAR->FLOWERS OK" appears next to the clock.
- **Park alert** — from 0:20 the robot estimates the drive from where it is to the park pose (around the HIVE). Once the time left is that drive plus 4 s (`teleop.park_alert_margin_ms`), and until the robot is PARKED: both gamepads rumble and flash red every second, the Driver Station says "park now" once, and **PARK NOW** heads the telemetry. A `Park` line shows the estimated drive and time left from 0:20. With no clear route, or a lost pose, it fires at 0:20. F310 gamepads have no rumble motors or lights: only the voice and telemetry reach the drivers. It never drives the robot.
- **Human NECTAR allowed** — one NECTAR per TIP of this alliance's HIVE (including AUTO and partner TIPS the camera saw), all five after 1:00. The human player subtracts what they have already entered. The robot's clock starts when TeleOp is started; the field timer remains the authority.
