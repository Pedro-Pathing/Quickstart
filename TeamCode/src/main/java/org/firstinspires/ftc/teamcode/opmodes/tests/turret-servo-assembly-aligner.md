# Turret Servo Assembly Aligner Guide

`TurretServoAssemblyAligner` is a guided, step-by-step setup for the paired turret servos (`turretL` and `turretR`). Each step is prompted on the Driver Station. Paired-motion checks and encoder comparison are in a separate op mode, `TurretServoVerifier` (see the end of this guide).

## Op mode and hardware

- Driver Station name: `TurretServoAssemblyAligner`, group `Test`
- Source: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tests/TurretServoAssemblyAligner.java`
- Required configured devices: servos `turretL` and `turretR`, and analog input `floodgate`.

## Safety

- Support the turret and keep hands clear of the gears. A servo that is unpowered has no holding torque.
- The Floodgate current is always shown on the first telemetry line, with the limit. Watch it, especially during the PWM limit test.
- Current protection: if the current stays above `currentLimitAmps` (default `8.0`) for `currentTripMs` (default `500`), both servos are disabled and the op mode shows a `FAULT` screen. If the current is still above the limit `currentStopMs` (default `1500`) later, the op mode stops itself. After a fault, find the cause (for example binding gears) and restart the op mode.
- The Floodgate current is converted using the same scale as the rest of the code (`voltage / 3.3 * 80 A`). Adjust `currentLimitAmps` for your servos and supply if it trips too early or too late.
- Right stick button at any time (except after a fault) unpowers both servos and jumps to the DONE screen.

## Steps

Press **A** to advance unless noted. Servos only move in the steps that say they are powered.

1. **Start.** Both servos are unpowered. Press **A** to begin.
2. **Test turretL.** Only `turretL` is powered. Push the left stick up and down and verify it moves smoothly. Press **A** when it works; it is unpowered again.
3. **Test turretR.** Same as step 2 for `turretR`.
4. **PWM limits (optional).** "Test/verify the limits of each servo's PWM range?" **A** = yes, **B** = skip.
   - If yes, each servo steps through these pulse widths, one per **A** press: center (1500 us), 850 down to 500 us in 50 us steps, center, 2150 up to 2500 us in 50 us steps, center. Then the same for the other servo.
   - Watch for buzzing, straining, or a rise in the Floodgate current. Unload the servo first, since the ends of the range may be beyond the mechanism's safe travel.
   - **B** skips the remaining PWM tests at any point.
   - The step size and start points are `pwmTestStepUs` (50), `pwmTestLowStartUs` (850), and `pwmTestHighStartUs` (2150). The end points are `pwmMinUs` and `pwmMaxUs` (500 and 2500).
5. **Review values.** Servos are unpowered. The screen shows `centerPos`, `targetPos`, `rightOffset` (and its allowed range), `invertRight`, `minPos`/`maxPos`, and the resulting `LeftCmd`/`RightCmd` with pulse widths. Change values in Panels or in the source if needed. Press **A** to power both servos at `centerPos`.
6. **Align and mesh.** Both servos are powered.
   1. **A** moves both servos to `centerPos`.
   2. Fit/mesh the gear while the servos hold that position.
   3. If it does not mesh, nudge `rightOffset`: **D-pad up/down** (0.001 per press) or **right/left trigger** (0.005 per press; release between presses). Left stick, **D-pad left/right** (0.0025), and bumpers (0.01) move `targetPos` if the shared position needs adjusting.
   4. When done, note `rightOffset` and press **B**. Both servos are unpowered and the DONE screen shows the final values.

`rightOffset` changes made with the controller exist only for the current session. Copy the final value into the `rightOffset` initializer in the source if it should persist. `rightOffset` is a servo-position offset, not degrees.

## How the commands are computed

- `LeftCmd = targetPos`
- `RightCmd = targetPos + rightOffset` (or `1 - targetPos + rightOffset` if `invertRight` is true)
- Both are clamped to `minPos`/`maxPos`, and `targetPos` is limited to the range where both commands fit.

## Logging

Floodgate current is logged as `FloodgateAmps` and `TurretAligner/FloodgateAmps`. Other keys under `TurretAligner/`: `Step`, `LeftEnabled`, `RightEnabled`, `TargetPos`, `LeftCmd`, `RightCmd`, `RightOffset`.

## TurretServoVerifier

`TurretServoVerifier` (group `Test`) holds the checks that used to be modes of the aligner. It shares `centerPos`, `rightOffset`, `invertRight`, and the step sizes with the aligner. Outputs are disabled while idle.

- **START** (idle only) switches between VERIFY and CHARACTERIZE. **A** starts the selected mode. **Right stick button** stops it.
- **VERIFY**: both servos powered and moved together. Stick, D-pad left/right, and bumpers move the target; **BACK** and **A** center it. **X** toggles an automatic sweep between `verifySweepMin` and `verifySweepMax` (0.35 and 0.65) at `verifySweepRatePerSec`; **Y** reverses it. D-pad up/down and triggers adjust `rightOffset`.
- **CHARACTERIZE**: powers one servo at a time and records command/encoder samples (needs the optional `turretAnalog` input). **A** captures a sample, **Y** swaps the active servo, **X** compares the left and right sample sets and suggests a `rightOffset` change, **B** clears the active servo's samples. Up to `maxCharacterizeSamples` (20) per servo; samples match when their commands differ by no more than `compareCommandTolerance` (0.01).
- Floodgate current is shown on screen and logged, but the verifier does not have automatic current protection.
- Log keys are under `TurretVerify/`.
