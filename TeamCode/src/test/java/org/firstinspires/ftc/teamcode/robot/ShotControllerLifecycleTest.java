package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.junit.jupiter.api.Test;

public class ShotControllerLifecycleTest {
  @Test
  public void oldSolveCannotPublishAfterRestartOrBallTypeRoundTrip() throws Exception {
    for (boolean restart : new boolean[] {true, false}) {
      CountDownLatch entered = new CountDownLatch(1);
      CountDownLatch release = new CountDownLatch(1);
      CountDownLatch nextSolve = new CountDownLatch(1);
      var calls = new java.util.concurrent.atomic.AtomicInteger();
      ShotController controller =
          new ShotController(
              mock(Shooter.class),
              mock(Turret.class),
              mock(Intake.class),
              () -> new Pose(58, 27, Math.PI / 2),
              Velocity::zero,
              null,
              new FieldModel(Alliance.RED),
              new HiveTracker(3, 1000, 500, () -> 0),
              null,
              (inputs, previousFlight) -> {
                if (calls.getAndIncrement() == 0) {
                  entered.countDown();
                  try {
                    if (!release.await(5, TimeUnit.SECONDS)) {
                      throw new IllegalStateException("test did not release solver");
                    }
                  } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                  }
                  return new ShotSolution(
                      0.5,
                      900,
                      300,
                      0,
                      0.7,
                      0,
                      30,
                      true,
                      "old solve",
                      inputs.target().epoch(),
                      inputs.ballType());
                }
                nextSolve.countDown();
                return ShotSolution.invalid("new solve");
              });
      try {
        controller.startShot(0.5, false);
        controller.periodic();
        assertTrue(entered.await(5, TimeUnit.SECONDS));
        if (restart) {
          controller.stopShot();
          controller.startShot(0.5, false);
        } else {
          controller.setBallType(BallType.NECTAR);
          controller.setBallType(BallType.POLLEN);
        }
        controller.periodic();
        release.countDown();
        // The second solve can only begin after publication of the first has been considered.
        assertTrue(nextSolve.await(5, TimeUnit.SECONDS));
        assertFalse(controller.getLastSolution().isValid());
        controller.stopShot();
        assertFalse(controller.isFeedCommanded());
      } finally {
        release.countDown();
        controller.shutdown();
      }
    }
  }
}
