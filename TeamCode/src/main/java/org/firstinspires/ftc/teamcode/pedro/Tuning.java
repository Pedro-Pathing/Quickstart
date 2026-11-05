package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Robot-hosted Pedro 3 AutoTune procedures for this drivetrain and localizer. Each hardware factory
 * reloads config first, so values sent with pushConfig apply on the next run, not the next app
 * restart.
 */
public final class Tuning {
  private Tuning() {}

  private static <T> T fresh(java.util.function.Supplier<T> build) {
    config.reload();
    return build.get();
  }

  @Tuner
  public static Procedure mecanumTuner() {
    return new MecanumTuner();
  }

  @Tuner
  public static Procedure pinpointTuner() {
    return new PinpointTuner();
  }

  @Tuner
  public static Procedure foresightTuner() {
    return new ForesightTuner(
        h -> fresh(() -> new PinpointLocalizer(h, Constants.localizerConfig())),
        h -> fresh(() -> new Mecanum(h, Constants.drivetrainConfig())));
  }

  @Tuner
  public static Procedure tests() {
    return new Tests(
        h -> fresh(() -> new Mecanum(h, Constants.drivetrainConfig())),
        h -> fresh(() -> new PinpointLocalizer(h, Constants.localizerConfig())),
        () -> fresh(() -> new Foresight(Constants.foresightConfig())));
  }
}
