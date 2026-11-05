package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.tests.CasablancaJumpAutotuneOpMode;
import org.firstinspires.ftc.teamcode.utilities.JumpImu;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

public class JumpImuTest {
  @AfterEach
  public void restoreConfig() {
    config.reload();
  }

  @Test
  public void newPinpointReadsTiltWithoutInitializingHubOrResettingLocalizer() {
    HardwareMap hardware = mock(HardwareMap.class);
    GoBildaPinpointDriver pinpoint = mock(GoBildaPinpointDriver.class);
    when(hardware.get(GoBildaPinpointDriver.class, "pinpoint")).thenReturn(pinpoint);
    when(pinpoint.getDeviceVersion()).thenReturn(3);
    when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
    when(pinpoint.getPitch(AngleUnit.DEGREES)).thenReturn(2.0);
    when(pinpoint.getRoll(AngleUnit.DEGREES)).thenReturn(-3.0);
    JumpImu imu = new JumpImu(hardware);
    long before = System.nanoTime();
    var sample = imu.read();
    assertEquals(2, sample.pitch(), 0);
    assertEquals(-3, sample.roll(), 0);
    assertTrue(sample.nanos() >= before);
    assertTrue(imu.source().startsWith("Pinpoint"));
    verify(hardware, never()).get(eq(IMU.class), anyString());
    verify(pinpoint, never()).update();
    verify(pinpoint, never()).resetPosAndIMU();
    verify(pinpoint, never()).recalibrateIMU();
    when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ);
    assertThrows(IllegalStateException.class, imu::read);
  }

  @Test
  public void oldPinpointUsesControlHubAndPreservesAcquisitionTimestamp() {
    for (int firmware : new int[] {1, 2}) {
      HardwareMap hardware = mock(HardwareMap.class);
      GoBildaPinpointDriver pinpoint = mock(GoBildaPinpointDriver.class);
      IMU hub = mock(IMU.class);
      when(hardware.get(GoBildaPinpointDriver.class, "pinpoint")).thenReturn(pinpoint);
      when(hardware.get(IMU.class, config.casablanca.jump_tuner.imu_name)).thenReturn(hub);
      when(pinpoint.getDeviceVersion()).thenReturn(firmware);
      when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
      when(hub.initialize(any())).thenReturn(true);
      when(hub.getRobotYawPitchRollAngles())
          .thenReturn(new YawPitchRollAngles(AngleUnit.DEGREES, 90, 2, -3, 123));
      JumpImu imu = new JumpImu(hardware);
      var sample = imu.read();
      assertEquals(2, sample.pitch(), 0);
      assertEquals(-3, sample.roll(), 0);
      assertEquals(123, sample.nanos());
      assertTrue(imu.source().startsWith("Control Hub"));
      verify(pinpoint, never()).getPitch(any());
      verify(pinpoint, never()).getRoll(any());
      verify(hub, never()).resetYaw();
      when(hub.initialize(any())).thenReturn(false);
      assertThrows(IllegalStateException.class, () -> new JumpImu(hardware));
    }
  }

  @Test
  public void rejectsTraversalAbove48AndInconsistentLimitsBeforeMotion() {
    var settings = config.casablanca.jump_tuner;
    CasablancaJumpAutotuneOpMode.validateSettings(settings);
    settings.max_travel_inches = 49;
    assertThrows(
        IllegalArgumentException.class,
        () -> CasablancaJumpAutotuneOpMode.validateSettings(settings));
    settings.max_travel_inches = 48;
    settings.stop_margin_inches = 48;
    assertThrows(
        IllegalArgumentException.class,
        () -> CasablancaJumpAutotuneOpMode.validateSettings(settings));
    settings.stop_margin_inches = 4;
    settings.drive_distance_inches = 44;
    assertThrows(
        IllegalArgumentException.class,
        () -> CasablancaJumpAutotuneOpMode.validateSettings(settings));
    settings.drive_distance_inches = 16;
    settings.reaction_time_sec = 0;
    assertThrows(
        IllegalArgumentException.class,
        () -> CasablancaJumpAutotuneOpMode.validateSettings(settings));
    settings.reaction_time_sec = .001;
    assertThrows(
        IllegalArgumentException.class,
        () -> CasablancaJumpAutotuneOpMode.validateSettings(settings));
    settings.reaction_time_sec = .05;
    settings.trial_power = Double.NaN;
    assertThrows(
        IllegalArgumentException.class,
        () -> CasablancaJumpAutotuneOpMode.validateSettings(settings));
  }
}
