package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.BallInventory;
import org.junit.jupiter.api.Test;

public class ShotInputSafetyTest {
  @Test
  public void localizationFaultImmediatelyBlocksAnAlreadyValidShot() {
    Shooter shooter = mock(Shooter.class);
    Intake intake = mock(Intake.class);
    when(shooter.getTargetPower()).thenReturn(0.5);
    when(shooter.getShooterVelocity()).thenReturn(config.shooter.max_rpm * 0.5);
    when(intake.inventory()).thenReturn(new BallInventory(4, true));
    Pose good = new Pose(58, 27, Math.PI / 2);
    AtomicReference<Pose> pose = new AtomicReference<>(good);
    AtomicReference<Velocity> velocity = new AtomicReference<>(Velocity.zero());
    CountDownLatch release = new CountDownLatch(1);
    ShotController controller =
        new ShotController(
            shooter,
            mock(Turret.class),
            intake,
            pose::get,
            velocity::get,
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
    try {
      controller.startShot(0.5, false, false);
      for (double bad :
          new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
        for (boolean badVelocity : new boolean[] {false, true}) {
          for (Pose broken :
              new Pose[] {
                new Pose(bad, good.y(), good.heading()),
                new Pose(good.x(), bad, good.heading()),
                new Pose(good.x(), good.y(), bad),
                null
              }) {
            pose.set(good);
            velocity.set(Velocity.zero());
            controller.setInitialSolution(
                new ShotSolution(0.5, 1000, 300, 0, 0.7, 0, 30, true, "test", 0, BallType.POLLEN));
            controller.periodic();
            assertTrue(controller.isFeedCommanded());
            clearInvocations(shooter, intake);
            if (badVelocity) {
              velocity.set(
                  broken == null ? null : new Velocity(broken.x(), broken.y(), broken.heading()));
            } else {
              pose.set(broken);
            }
            controller.periodic();
            assertFalse(
                controller.isFeedCommanded(),
                "a current epoch is insufficient when localization is invalid");
            verify(intake).stop();
            verify(shooter, never()).setTargetHoodPosition(anyDouble());
            pose.set(good);
            velocity.set(Velocity.zero());
            controller.periodic();
            assertFalse(
                controller.isFeedCommanded(), "wait for a fresh solve after localization recovers");
          }
        }
      }
    } finally {
      controller.shutdown();
      release.countDown();
    }
  }
}
