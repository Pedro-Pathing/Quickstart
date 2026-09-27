package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.units.Units.RotationsPerMinute;

import org.firstinspires.ftc.teamcode.control.Math;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.units.measuretypes.AngularVelocity;

public class PewPew implements Mechanism {
    private final NextMotor Motor = new NextMotor("flywheelMotor");
    private Double targetVelocity = 3500.0;
    public boolean isActive = false;

    public void on() {
        isActive = true;
    }

    public void off() {
        isActive = false;
    }

    public void adjustTargetVelocity(Double change) {
        targetVelocity += change;
    }

    public void setTargetVelocity(Double newVelocity) {
        isActive = true;
        targetVelocity = Math.MaxOf(0.0, newVelocity);
    }

    public AngularVelocity getVelocity() {
        return Motor.getEncoderVelocity();
    }

    @Override
    public void periodic() {
            if (isActive) {
                Motor.setVelocitySetpoint(RotationsPerMinute.of(targetVelocity));
            } else {
                Motor.setVelocitySetpoint(RotationsPerMinute.of(0.0));
            }
            Telemetry.log("shooter Velocity:", this.getVelocity());
    }
}
