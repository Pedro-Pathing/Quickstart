package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
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

public class ShotCountingTest {
  @Test
  public void commandedSpeedDropDoesNotCountAPhantomBall() {
    Shooter shooter = mock(Shooter.class);
    Intake intake = mock(Intake.class);
    BallInventory inventory = new BallInventory(4, true);
    inventory.set(4);
    when(intake.inventory()).thenReturn(inventory);
    AtomicReference<Double> power = new AtomicReference<>(0.0);
    doAnswer(
            inv -> {
              power.set(inv.getArgument(0));
              return null;
            })
        .when(shooter)
        .setTargetPower(anyDouble());
    when(shooter.getTargetPower()).thenAnswer(inv -> power.get());
    CountDownLatch release = new CountDownLatch(1);
    ShotController controller =
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
              return ShotSolution.invalid("blocked test solver");
            });
    try {
      controller.startShot(1200 / config.shooter.max_rpm, false, true);
      controller.setInitialSolution(solution(1200));
      when(shooter.getShooterVelocity()).thenReturn(1200.0);
      controller.periodic();
      controller.periodic();
      // A commanded 12% reduction, reached over four 20 ms control loops.
      controller.setInitialSolution(solution(1056));
      for (double speed : new double[] {1164, 1128, 1092, 1056}) {
        when(shooter.getShooterVelocity()).thenReturn(speed);
        controller.periodic();
      }
      assertEquals(0, controller.getBallsFired(), "a setpoint reduction is not a launched ball");
      assertEquals(4, inventory.count());
      when(shooter.getShooterVelocity()).thenReturn(1056.0);
      controller.periodic();
      when(shooter.getShooterVelocity()).thenReturn(900.0);
      controller.periodic();
      assertEquals(1, controller.getBallsFired(), "a real dip at the new speed still counts");
      assertEquals(3, inventory.count());
      controller.setInitialSolution(solution(1200));
      controller.periodic();
      assertEquals(1, controller.getBallsFired(), "re-arming preserves balls already fired");
      when(shooter.getShooterVelocity()).thenReturn(1200.0);
      controller.periodic();
      when(shooter.getShooterVelocity()).thenReturn(1000.0);
      controller.periodic();
      assertEquals(2, controller.getBallsFired());
      assertEquals(2, inventory.count());
    } finally {
      controller.shutdown();
      release.countDown();
    }
  }

  private static ShotSolution solution(double rpm) {
    return new ShotSolution(0.5, rpm, 300, 0, 0.7, 0, 30, true, "test", 0, BallType.POLLEN);
  }
}
