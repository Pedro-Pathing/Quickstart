package org.firstinspires.ftc.teamcode.robot;

import static org.mockito.Mockito.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.junit.jupiter.api.Test;

public class LumosWriterTest {
  @Test
  public void constructorAndColorTargetDoNotEnablePwm() {
    HardwareMap map = mock(HardwareMap.class);
    ServoImplEx light = mock(ServoImplEx.class);
    when(map.get(Servo.class, "light")).thenReturn(light);
    Lumos lumos = new Lumos(map);
    lumos.setColor(Lumos.Color.RED);
    verifyNoInteractions(light);
    lumos.periodic();
    lumos.periodic();
    verify(light).setPwmEnable();
    verify(light).setPwmRange(any());
    verify(light, times(2)).setPosition(Lumos.Color.RED.position);
  }
}
