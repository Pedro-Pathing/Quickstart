package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.Nullable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.records.TiltReading;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/** Reuses Pedro's Pinpoint, falling back to the configured Control Hub IMU on older firmware. */
public final class JumpImu {
  private final GoBildaPinpointDriver pinpoint;
  private final @Nullable IMU hub;
  private final int firmware;

  public JumpImu(HardwareMap hardwareMap) {
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    firmware = pinpoint.getDeviceVersion();
    if (firmware >= 3) {
      hub = null;
    } else {
      var c = config.casablanca.jump_tuner;
      hub = hardwareMap.get(IMU.class, c.imu_name);
      if (!hub.initialize(
          new IMU.Parameters(
              new RevHubOrientationOnRobot(
                  RevHubOrientationOnRobot.LogoFacingDirection.valueOf(c.imu_logo),
                  RevHubOrientationOnRobot.UsbFacingDirection.valueOf(c.imu_usb))))) {
        throw new IllegalStateException("Control Hub IMU failed to initialize");
      }
    }
  }

  /** Call after Robot.update(); never resets or updates Pedro's localizer a second time. */
  public TiltReading read() {
    if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
      throw new IllegalStateException("Pinpoint is not READY: " + pinpoint.getDeviceStatus());
    }
    if (hub != null) {
      var angles = hub.getRobotYawPitchRollAngles();
      return new TiltReading(
          angles.getPitch(AngleUnit.DEGREES),
          angles.getRoll(AngleUnit.DEGREES),
          angles.getAcquisitionTime());
    }
    // Older bulk packets omit these registers; SDK getters explicitly read them when needed.
    // Pinpoint supplies no acquisition timestamp for pitch/roll: timestamp the synchronous reads.
    double pitch = pinpoint.getPitch(AngleUnit.DEGREES);
    double roll = pinpoint.getRoll(AngleUnit.DEGREES);
    return new TiltReading(pitch, roll, System.nanoTime());
  }

  public String source() {
    return hub == null
        ? "Pinpoint (firmware " + firmware + ")"
        : "Control Hub IMU: " + config.casablanca.jump_tuner.imu_name;
  }
}
