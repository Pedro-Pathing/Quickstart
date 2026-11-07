# Launcher drive: reversed pulleys and Through Bore encoders

Plan agreed 2026-11-07. Nothing below is built or coded yet. Both launchers (Pollen and Nectar)
will share the same drive train.

## Why

- **The drive now:** an HD Hex motor belted to the flywheel, **40T on the motor, 32T on the
  flywheel**, so the flywheel turns at 0.8 times motor speed. The belt stays because folding the
  motor into the launcher body makes packaging much easier.
- **Measured on the Pollen prototype at full power:** about 5400 RPM at the motor, 4320 RPM at the
  flywheel.
- **Speeds needed:** the launch model (`LaunchModel`, ported from the sim) needs about **2700 to
  3900 flywheel RPM** for both launchers. That covers 0.7 m, the closest distance that scores, to
  2.24 m, the farthest a robot's centre can be from our nearest CELL. These numbers assume the
  sim's `exitSpeedRatio` of 0.406 (ball speed over wheel surface speed), which hasn't been
  measured. Flywheel Characterization and slow-motion video will measure it, and every RPM
  scales with it.
- **Problem with 40:32:** the motor runs at 3375 to 4875 RPM, which is 56 to 81% of its free speed
  of about 6000 RPM. That's past the point where the motor gives its most power (half free speed),
  and the farthest shots leave only about 11% headroom. After a ball, the PIDF saturates at full
  power, so recovery is set by the motor rather than the gains. A low battery makes it worse.
- **Precision needed:** the range of speeds that scores narrows to about 130 RPM wide at 2.24 m, so
  far shots need the wheel within about ±65 RPM (±2%).

## Hardware changes

1. **Swap the pulleys: 32T on the motor, 40T on the flywheel.** The flywheel then turns at 1.25
   times motor speed. The total tooth count is the same, so the belt and centre distance don't
   change; only the pulleys need re-printing. The motor runs at 2160 to 3120 RPM (36 to 52% of
   free speed), right around its peak power.
2. **Fit a REV Through Bore encoder on each flywheel (output) shaft:** 8192 counts per turn, rated
   to 10,000 RPM.
   - Plug it into a high-speed encoder port on the Control Hub (port 0 or 3) if possible.
   - If a spare motor port is free, plug it in there and keep the motor's built-in encoder
     connected to the launcher motor's port. That keeps a backup encoder and allows the check
     below.

## Code changes (TeamCode)

### `flywheel/Flywheel.java`

- **Speed from position for high-count encoders.** The hub reports velocity as a 16-bit number of
  counts per second (`LynxGetBulkInputDataResponse.velocities` is a `short[]`), so it can't go
  above 32,767 counts/s. At 8192 counts per turn that's only 240 RPM. With a Through Bore, work out
  RPM from the change in encoder position each loop:
  `(position - lastPosition) / countsPerRev / dt * 60`. Position is 32-bit and in the same bulk
  read, so it costs no extra loop time. At a 10 ms loop, one count is about 0.7 RPM. Keep
  `getVelocity()` for the motor's built-in encoder, which overflows only above about 70,000 motor
  RPM.
- **Encoder source.** The Through Bore may be on a different port from the motor it measures. Add
  a way to read the encoder from another motor's port, by name: `hardwareMap.get(DcMotorEx.class,
  name)` on that port, reading only `getCurrentPosition()` and never setting its power. Add this
  to the constructor or as a setter, and keep the existing constructor working, so each launcher
  stays on its motor encoder until it's switched over.
- **Encoder direction,** separate from the motor's direction, since the Through Bore may count the
  other way depending on how it's mounted.
- **Counts per flywheel turn** (`countsPerRev`, so that `Flywheel` always reads and controls
  flywheel RPM):
  - Through Bore on the shaft: **8192**
  - Motor encoder, pulleys swapped (32:40): 28 / 1.25 = **22.4**
  - Motor encoder, current pulleys (40:32): 28 × 1.25 = **35**
  - Motor encoder, direct drive: 28 (`DEFAULT_COUNTS_PER_REV` now)

  At the moment every launcher uses 28, so it reads motor RPM while `RpmTable` and `LaunchModel`
  give flywheel RPM. With the current pulleys, shots would come out about 20% slow. Fixing that is
  part of this change.
- **RPM filter.** The cleaner Through Bore signal should allow a shorter
  `DEFAULT_RPM_FILTER_TIME_CONSTANT` than 0.03 s. Choose the new value from `rawRPM` on the
  Panels graph.
- **Fault detection** (`checkEncoder`, `faultMaxRPM`) uses whichever encoder is fitted, so it
  needs no change. Check that `faultMaxRPM` still makes sense in flywheel RPM.

### Where the launchers are created

`new Flywheel(...)` appears in:

- `opmodes/teleop/CompetitionTeleOp.java`
- `opmodes/teleop/MoreThanDrivingTestTeleOp.java`
- `opmodes/teleop/FlywheelCharacterization.java` (`Launcher` enum)
- `opmodes/auto/LoopingAuto.java`
- `opmodes/auto/TestAuto.java`
- `opmodes/teleop/FlywheelTuner.java`, which uses its own `motorName` and `direction` settings
- `opmodes/teleop/FlyMeToTheWheel.java`

Each one repeats the motor names and directions (`"PollenFlywheel"` REVERSE, `"NectarFlywheel"`
FORWARD). The encoder settings would need repeating too, so it's worth moving launcher setup into
one place first: a factory such as `Launchers.pollen(hardwareMap)` and `Launchers.nectar(...)`, or
a small config class. The tuned gains for each launcher would live there as well.

### Other code to check

- **`FlywheelCharacterization`:** `maxRPM` (5400) and the D-pad steps are in whatever RPM
  `Flywheel` reports. Once it reports flywheel RPM, set `maxRPM` to that launcher's measured top
  flywheel speed.
- **`FlywheelTuner`:** the feedforward test fits kV and kS per RPM as `Flywheel` reports it. Re-run
  it after the change, and copy the new gains into each launcher's setup.
- **`LaunchModel.MAX_FLYWHEEL_RPM`** (4500) caps the RPM solver. The 2700 to 3900 range fits, but
  raise it if a measured `exitSpeedRatio` pushes the speeds up.
- **`Flywheel` default gains:** `DEFAULT_KV` and `DEFAULT_KS` were fitted per motor RPM on the old
  setup, so they'll be wrong once `Flywheel` works in flywheel RPM.

## Checks on the robot

1. **Count rate:** at the top of the range the Through Bore produces about 670,000 counts a second
   (more at full power). Spin up to full power with both encoders connected. The Through Bore RPM
   should equal 1.25 times the motor-encoder RPM all the way to top speed. If it falls short or
   jumps about at high speed, the hub input isn't keeping up: try port 0 or 3.
2. **Direction:** both encoders should read positive RPM when the launcher spins forward.
3. **Top speed:** measure the flywheel's top speed at full power with the swapped pulleys. It's
   likely around 6000 RPM, less because of air drag. Use it for `maxRPM` in Flywheel
   Characterization.
4. **Baseline first (optional):** before swapping pulleys, run Flywheel Characterization at about
   2800 and 3800 flywheel RPM and compare `recovery_ms`. Repeat after the change to see the
   improvement.

## The sim (BioBuzzSimV2, separate repo)

- **Gear ratio:** the sim's launcher motors need the same ratio so their built-in encoders give 28
  counts per motor turn.
- **Through Bore:** the sim also needs a Through Bore encoder on the flywheel shaft (8192 counts
  per flywheel turn) on whichever port the robot uses. Otherwise TeamCode will read the wrong speed
  in the sim.
