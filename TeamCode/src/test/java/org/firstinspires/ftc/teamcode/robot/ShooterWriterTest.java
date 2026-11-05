package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.*;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ShooterWriterTest {
  private DcMotorEx first;
  private DcMotorEx second;
  private Servo hood;
  private Shooter shooter;
  private boolean savedFtcPid;

  @BeforeEach
  @SuppressWarnings("unchecked")
  public void setUp() {
    savedFtcPid = config.shooter.use_ftc_pid;
    config.shooter.use_ftc_pid = true;
    HardwareMap map = mock(HardwareMap.class);
    first = mock(DcMotorEx.class);
    second = mock(DcMotorEx.class);
    hood = mock(Servo.class);
    when(map.get(DcMotorEx.class, "shooter")).thenReturn(first);
    when(map.get(DcMotorEx.class, "shooter2")).thenReturn(second);
    when(map.get(Servo.class, "hood")).thenReturn(hood);
    map.voltageSensor = mock(HardwareMap.DeviceMapping.class);
    VoltageSensor voltage = mock(VoltageSensor.class);
    when(voltage.getVoltage()).thenReturn(12.0);
    when(map.voltageSensor.iterator()).thenAnswer(inv -> List.of(voltage).iterator());
    for (DcMotorEx motor : new DcMotorEx[] {first, second}) {
      AtomicReference<DcMotor.RunMode> mode = new AtomicReference<>();
      doAnswer(
              inv -> {
                mode.set(inv.getArgument(0));
                return null;
              })
          .when(motor)
          .setMode(any());
      when(motor.getMode()).thenAnswer(inv -> mode.get());
    }
    shooter = new Shooter(map);
  }

  @AfterEach
  public void restoreConfig() {
    config.shooter.use_ftc_pid = savedFtcPid;
  }

  @Test
  public void hoodWaitsForPeriodicAndSettersNeverWriteHardware() {
    verify(hood, never()).setPosition(anyDouble());
    double position = config.shooter.ballistics.max_hood_servo_pos;
    shooter.setTargetHoodPosition(position);
    shooter.setTargetPower(0.5);
    shooter.stop();
    assertEquals(0.0, shooter.getTargetPower(), 0.0);
    verify(hood, never()).setPosition(anyDouble());
    verify(first, never()).setPower(anyDouble());
    verify(first, never()).setVelocity(anyDouble());
    verify(second, never()).setPower(anyDouble());
    verify(second, never()).setVelocity(anyDouble());
    shooter.periodic();
    verify(hood).setPosition(position);
    verify(first).setVelocity(0.0);
  }

  @Test
  public void unchangedVelocityIsWrittenOncePerMotorIncludingIdle() {
    shooter.setTargetPower(0.5);
    for (int i = 0; i < 5; i++) shooter.periodic();
    double velocity = config.shooter.max_rpm * 0.5;
    verify(first).setVelocity(velocity);
    verify(second).setVelocity(velocity);
    shooter.setTargetPower(0.6);
    shooter.periodic();
    verify(first).setVelocity(config.shooter.max_rpm * 0.6);
    verify(second).setVelocity(config.shooter.max_rpm * 0.6);
    shooter.stop();
    for (int i = 0; i < 5; i++) shooter.periodic();
    verify(first).setVelocity(0.0);
    verify(second).setVelocity(0.0);
    shooter.setTargetPower(0.5);
    shooter.periodic();
    verify(first, times(2)).setVelocity(velocity);
    verify(second, times(2)).setVelocity(velocity);
    shooter.stop();
    shooter.periodic();
    verify(first, times(2)).setVelocity(0.0);
    verify(second, times(2)).setVelocity(0.0);
  }

  @Test
  public void modeChangesInvalidateEachMotorsVelocityCache() {
    shooter.setTargetPower(0.5);
    shooter.periodic();
    double velocity = config.shooter.max_rpm * 0.5;
    second.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    shooter.periodic();
    verify(first).setVelocity(velocity);
    verify(second, times(2)).setVelocity(velocity);
    config.shooter.use_ftc_pid = false;
    shooter.periodic();
    config.shooter.use_ftc_pid = true;
    shooter.periodic();
    shooter.periodic();
    verify(first, times(2)).setVelocity(velocity);
    verify(second, times(3)).setVelocity(velocity);
  }
}
