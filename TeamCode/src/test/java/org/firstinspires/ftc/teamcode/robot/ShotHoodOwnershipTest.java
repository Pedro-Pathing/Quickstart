package org.firstinspires.ftc.teamcode.robot;

import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.clearInvocations;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.util.concurrent.CountDownLatch;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.BallInventory;
import org.junit.jupiter.api.Test;

/**
 * Who owns the hood. A calibration shot pins its own hood and flywheel; the solver's answer for the
 * current pose must not overwrite that hood, or every calibration row is recorded against a hood
 * position the ball was never fired at.
 */
public class ShotHoodOwnershipTest {
  private static final double SOLVED_HOOD = 0.3;
  private static final double PINNED_HOOD = 0.7;

  private final Shooter shooter = mock(Shooter.class);
  private final CountDownLatch release = new CountDownLatch(1);
  private final ShotController controller;

  public ShotHoodOwnershipTest() {
    Intake intake = mock(Intake.class);
    when(shooter.getTargetPower()).thenReturn(0.5);
    when(shooter.getShooterVelocity()).thenReturn(config.shooter.max_rpm * 0.5);
    when(intake.inventory()).thenReturn(new BallInventory(4, true));
    controller =
        new ShotController(
            shooter,
            mock(Turret.class),
            intake,
            () -> new Pose(58, 27, Math.PI / 2),
            Velocity::zero,
            null,
            new FieldModel(Alliance.RED),
            new HiveTracker(3, 1000, 500, () -> 0),
            null,
            (inputs, flight) -> {
              try {
                release.await();
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
              }
              return ShotSolution.invalid("test solver");
            });
  }

  private static ShotSolution currentSolution() {
    return new ShotSolution(
        SOLVED_HOOD, 1000, 300, 0, 0.7, 0, 30, true, "test", 0, BallType.POLLEN);
  }

  @Test
  public void aFixedCalibrationShotKeepsItsPinnedHood() {
    try {
      // Calibration: pinned power, turret aims, solved rpm off, no valid solution required.
      controller.startShot(0.5, true, false, false);
      controller.setInitialSolution(currentSolution());
      // setInitialSolution pre-positions the hood itself (AUTO's path); judge periodic() alone.
      clearInvocations(shooter);
      for (int i = 0; i < 5; i++) {
        shooter.setTargetHoodPosition(PINNED_HOOD);
        controller.periodic();
      }
      verify(shooter, never()).setTargetHoodPosition(eq(SOLVED_HOOD));
    } finally {
      controller.shutdown();
      release.countDown();
    }
  }

  @Test
  public void aSolvedShotStillSetsTheSolvedHood() {
    try {
      controller.startShot(0.5, true, true, true);
      controller.setInitialSolution(currentSolution());
      clearInvocations(shooter);
      controller.periodic();
      verify(shooter, atLeastOnce()).setTargetHoodPosition(eq(SOLVED_HOOD));
      verify(shooter, atLeastOnce()).setTargetHoodPosition(anyDouble());
    } finally {
      controller.shutdown();
      release.countDown();
    }
  }
}
