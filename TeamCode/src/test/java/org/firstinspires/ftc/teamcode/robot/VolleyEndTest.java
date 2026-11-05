package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.firstinspires.ftc.teamcode.utilities.BallInventory;
import org.junit.jupiter.api.Test;

/**
 * The AUTO volley counts a ball on the first sample of its flywheel dip, while the ball is still in
 * contact with the wheel. Ending the volley (and braking the flywheel) right there cut the last
 * ball's launch short; the volley must wait for the wheel to recover first.
 */
public class VolleyEndTest {
  @Test
  public void theVolleyEndsOnlyAfterTheLastBallHasLeftTheWheel() {
    Shooter shooter = mock(Shooter.class);
    Intake intake = mock(Intake.class);
    BallInventory inventory = new BallInventory(4, true);
    inventory.set(4);
    when(intake.inventory()).thenReturn(inventory);
    AtomicReference<Double> power = new AtomicReference<>(0.0);
    AtomicReference<Double> wheel = new AtomicReference<>(0.0);
    doAnswer(
            inv -> {
              power.set(inv.getArgument(0));
              return null;
            })
        .when(shooter)
        .setTargetPower(anyDouble());
    when(shooter.getTargetPower()).thenAnswer(inv -> power.get());
    when(shooter.getShooterVelocity()).thenAnswer(inv -> wheel.get());
    Follower follower = mock(Follower.class);
    when(follower.pose()).thenReturn(new Pose(58, 27, Math.PI / 2));
    CountDownLatch release = new CountDownLatch(1);
    ShotController controller =
        new ShotController(
            shooter,
            mock(Turret.class),
            intake,
            follower::pose,
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
      controller.volleyCommand(follower, 1).schedule();
      loop(controller, wheel, 0.0);
      controller.setInitialSolution(
          new ShotSolution(0.5, 1200, 300, 0, 0.7, 0, 30, true, "test", 0, BallType.POLLEN));
      for (int i = 0; i < 4; i++) {
        loop(controller, wheel, 1200.0);
      }
      assertTrue(controller.isActive());

      // The ball hits the wheel: the first dip sample counts it.
      loop(controller, wheel, 1000.0);
      assertEquals(1, controller.getBallsFired());
      // Contact lasts a few 20 ms loops; the volley must hold the flywheel through all of them.
      for (double dip : new double[] {980.0, 975.0, 990.0}) {
        loop(controller, wheel, dip);
        assertTrue(controller.isActive(), "volley ended with the ball still on the wheel");
        assertTrue(power.get() > 0, "flywheel braked under the last ball");
      }

      // The wheel recovers: the ball is gone, now the volley may end.
      for (int i = 0; i < 3; i++) {
        loop(controller, wheel, 1200.0);
      }
      assertFalse(controller.isActive(), "volley never ended after the last ball");
    } finally {
      Scheduler.reset();
      controller.shutdown();
      release.countDown();
    }
  }

  /** One control loop in Robot.update() order: commands, then the shot controller. */
  private static void loop(
      ShotController controller, AtomicReference<Double> wheel, double velocity) {
    wheel.set(velocity);
    Scheduler.execute();
    controller.periodic();
  }
}
