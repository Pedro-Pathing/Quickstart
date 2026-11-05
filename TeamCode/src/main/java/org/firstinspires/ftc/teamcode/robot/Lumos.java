package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Lumos - Control class for goBILDA RGB Indicator Light (3118-0808-0002)
 *
 * <p>The RGB Indicator Light responds to standard Servo PWM signals. As the signal changes, the
 * displayed color smoothly transitions through a gradient. - Signal below 1100µsec (FTC: ~0.277):
 * Light turns off - Signal above 1900µsec (FTC: ~0.722): Light turns solid white
 */
public class Lumos {

  /** Servo position (0.0 - 1.0) for each color the goBILDA RGB light supports. */
  public enum Color {
    OFF(0.0),
    RED(0.277),
    ORANGE(0.333),
    YELLOW(0.388),
    SAGE(0.444),
    GREEN(0.500),
    AZURE(0.555),
    BLUE(0.611),
    INDIGO(0.666),
    VIOLET(0.722),
    WHITE(1.0);

    public final double position;

    Color(double position) {
      this.position = position;
    }
  }

  private final Servo light;
  private Color targetColor = Color.OFF;
  private boolean initialized;

  public Lumos(HardwareMap hardwareMap) {
    light = hardwareMap.get(Servo.class, "light");
  }

  public void setColor(Color color) {
    targetColor = color;
  }

  public void periodic() {
    if (!initialized) {
      if (light instanceof ServoImplEx lightEx) {
        lightEx.setPwmRange(new PwmControl.PwmRange(500, 2500));
        lightEx.setPwmEnable();
      }
      initialized = true;
    }
    light.setPosition(targetColor.position);
  }

  public double getPosition() {
    return light.getPosition();
  }
}
