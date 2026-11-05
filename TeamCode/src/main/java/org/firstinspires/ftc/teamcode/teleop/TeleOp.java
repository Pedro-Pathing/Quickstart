package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;

/**
 * BIOBUZZ TeleOp. Controls are listed in teleop_controls.md; in short:
 *
 * <p>Driver: sticks drive; X toggles facing the upward CELL; right trigger holds the shooting pose
 * for whichever CELL is up; left trigger holds the LOADING ZONE park pose; dpad left holds the
 * side-wall FLOWER pickup pose; dpad right or any stick cancels a hold.
 *
 * <p>Operator: A intake, B stop intake, dpad right (hold) eject, right trigger (hold)
 * aim-and-shoot, Y (hold) turret aim, X (hold) rev, dpad up fixed-power shot, left trigger cancels
 * everything, dpad down (hold) vision relocalize, left bumper records a TIP the camera missed, back
 * undoes one, right bumper toggles POLLEN/NECTAR shot tables, left stick button marks the robot
 * empty.
 */
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "!")
// Commands are assigned by buildCommands(), which TeleOpBase.init() calls.
@SuppressWarnings("NullAway.Init")
public class TeleOp extends TeleOpBase {

  private Command parkCommand;
  private Command shootPoseCommand;
  private Command flowerPoseCommand;
  private Command intakeCommand;
  private Command ejectCommand;
  private Command aimAndShootCommand;
  private Command aimCommand;
  private Command shootManualCommand;
  private Command manualRevCommand;

  /** Latched state of the driver's face-the-upward-CELL heading lock; toggled by driver X. */
  private boolean cellHeadingLockEngaged = false;

  @Override
  protected void buildCommands() {
    parkCommand =
        Command.build().setStart(() -> holdPoseIfClear(profile.parkPose())).requiring(follower);

    // Resolved when the command starts, so it always goes to the CELL that is up right now.
    shootPoseCommand =
        Command.build()
            .setStart(() -> holdPoseIfClear(profile.shootPose(hive.upCell())))
            .requiring(follower);

    flowerPoseCommand =
        Command.build().setStart(() -> holdPoseIfClear(profile.flowerPose())).requiring(follower);

    intakeCommand =
        intake.runIntakeCommand(config.teleop.intake_power, config.teleop.transfer_power);

    ejectCommand = intake.ejectCommand(config.teleop.eject_power);

    // Built by ShotController so autonomous volleys fire the exact same shot.
    aimAndShootCommand = shotController.aimAndShootCommand(follower);

    // Turret only, no flywheel. Aim (operator Y) and rev (operator X) are separate commands over
    // disjoint subsystems so holding both composes into aim-and-spin-up.
    aimCommand =
        Command.build()
            .setStart(() -> turret.setAimMode(Turret.AimMode.AIM_AT_TARGET))
            .setEnd(
                interrupted -> {
                  turret.setHoldAngle(0.0);
                  turret.setAimMode(Turret.AimMode.HOLD);
                })
            .requiring(turret);

    shootManualCommand =
        Command.build()
            .setStart(() -> shotController.startShot(Shooter.constantPower(), false))
            .setEnd(interrupted -> shotController.stopShot())
            .requiring(shooter, intake);

    // Flywheel only: intake ownership stays with intakeCommand.
    manualRevCommand =
        Command.build()
            .setStart(() -> shooter.setTargetPower(config.teleop.manual_rev_power))
            .setEnd(interrupted -> shooter.setTargetPower(0.0))
            .requiring(shooter);
  }

  @Override
  protected boolean hasActiveDriveOverride() {
    return Scheduler.isScheduled(parkCommand)
        || Scheduler.isScheduled(shootPoseCommand)
        || Scheduler.isScheduled(flowerPoseCommand)
        || Scheduler.isScheduled(aimAndShootCommand);
  }

  @Override
  protected void onLoop() {
    handleOperator();
    handleDriver();
  }

  private void handleOperator() {
    if (operatorA.onTrue()) {
      intakeCommand.schedule();
    }
    if (operatorB.onTrue()) {
      intakeCommand.cancel();
    }

    // Hold to spit out a fifth SCORING ELEMENT (G407) or opponent NECTAR (G408).
    if (operatorDpadRight.onTrue()) {
      ejectCommand.schedule();
    }
    if (operatorDpadRight.onFalse()) {
      ejectCommand.cancel();
    }

    if (operatorRightTrigger.onTrue()) {
      aimAndShootCommand.schedule();
    }
    if (operatorRightTrigger.onFalse()) {
      aimAndShootCommand.cancel();
    }

    if (operatorY.onTrue()) {
      aimCommand.schedule();
    }
    if (operatorY.onFalse()) {
      aimCommand.cancel();
    }

    if (operatorX.onTrue()) {
      manualRevCommand.schedule();
    }
    if (operatorX.onFalse()) {
      manualRevCommand.cancel();
    }

    if (operatorDpadUp.onTrue()) {
      shootManualCommand.schedule();
    }

    if (operatorLeftTrigger.onTrue()) {
      aimAndShootCommand.cancel();
      aimCommand.cancel();
      shootManualCommand.cancel();
      manualRevCommand.cancel();
      intakeCommand.cancel();
      ejectCommand.cancel();
    }

    // HIVE bookkeeping the camera may have missed. Each press changes the tracker's epoch, which
    // invalidates any shot solved against the previous upward CELL.
    if (operatorLeftBumper.onTrue()) {
      hive.recordTip();
    }
    if (operatorBack.onTrue()) {
      hive.undoTip();
    }

    if (operatorRightBumper.onTrue()) {
      shotController.setBallType(shotController.getBallType().toggled());
    }

    if (operatorLeftStick.onTrue()) {
      intake.inventory().set(0);
    }
  }

  private void handleDriver() {
    if (driverLeftTrigger.onTrue()) {
      parkCommand.schedule();
    }
    if (driverRightTrigger.onTrue()) {
      shootPoseCommand.schedule();
    }
    if (driverDpadLeft.onTrue()) {
      flowerPoseCommand.schedule();
    }

    // Driver X toggles a heading lock that keeps the chassis facing the upward CELL while the
    // driver translates. Republished every loop so the bearing follows the robot and flips to the
    // other CELL on a TIP. Deliberately the raw bearing, not the lead-compensated azimuth: lead is
    // proportional to robot velocity, so the lock would chase its own motion.
    if (driverX.onTrue()) {
      cellHeadingLockEngaged = !cellHeadingLockEngaged;
    }

    // Steering releases the lock, so a latched mode can never leave the driver unable to turn.
    if (Math.abs(driver.right_stick_x) > Casablanca.headingLockIntentThreshold) {
      cellHeadingLockEngaged = false;
    }

    Pose pose = follower.pose();
    CellTarget target = shotController.currentTarget();
    casablanca.setTargetHeadingLock(
        Turret.alignPose(pose.x(), pose.y(), target.x(), target.y()).heading(),
        cellHeadingLockEngaged);

    boolean driverStickDeflected =
        Math.abs(driver.left_stick_y) > 0.2
            || Math.abs(driver.left_stick_x) > 0.2
            || Math.abs(driver.right_stick_x) > 0.2;

    if (driverDpadRight.onTrue() || driverStickDeflected) {
      parkCommand.cancel();
      shootPoseCommand.cancel();
      flowerPoseCommand.cancel();
    }
  }
}
