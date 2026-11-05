package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.Test;

public class IntakeCapacityTest {
  @Test
  public void knownFullPreloadCannotBecomeUnknownJustByRequestingCollection() {
    HardwareMap map = mock(HardwareMap.class);
    DcMotorEx front = mock(DcMotorEx.class);
    DcMotorEx mid = mock(DcMotorEx.class);
    when(map.get(DcMotorEx.class, "intakeFront")).thenReturn(front);
    when(map.get(DcMotorEx.class, "intakeMid")).thenReturn(mid);
    Intake intake = new Intake(map);
    intake.inventory().set(config.inventory.capacity_count);
    intake.run(1.0, 0.1);
    verify(front, never()).setPower(anyDouble());
    intake.periodic();
    assertTrue(intake.isBlockedByCapacity());
    assertTrue(intake.inventory().isKnown());
    verify(front).setPower(0.0);
    verify(mid).setPower(0.0);
    intake.inventory().onShot();
    intake.periodic();
    assertFalse(intake.isBlockedByCapacity());
    assertFalse(intake.inventory().isKnown());
    verify(front).setPower(1.0);
  }
}
