package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.units.Units.DegreesPerSecond;
import static dev.nextftc.units.Units.RotationsPerMinute;

import org.firstinspires.ftc.teamcode.control.Math;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.units.measuretypes.AngularVelocity;

public class Flywheel implements Mechanism {
    private final NextMotor motorOne = new NextMotor("flywheelMotorOne");

    private final NextMotor motorTwo = new NextMotor("flywheelMotorTwo");
    private Double targetVelocity = 1400.0;
    public boolean isActive = false;

    public void on() {
        isActive = true; }

    public void off() { isActive = false; }

    public void adjustTarget(Double change) {
        targetVelocity += change;
    }

    public void setTargetVelocity(Double newVelocity) {
        isActive = true;
        //targetVelocity = Math.MaxOf(0.0, newVelocity);
    }

    public Double getVelocity() {
        return motorOne.getEncoderVelocity().getMagnitude();
    }

    public void start(CommandGamepad commandGamepad) {
        motorTwo.follow(motorOne, NextMotor.Direction.REVERSE);
        motorOne.getPositionConstants().withV(0.0075);
        motorOne.getPositionConstants().withP(0.1);
        commandGamepad.dpadUp().onTrue(instant(() -> targetVelocity+= 100.0));
        commandGamepad.dpadDown().onTrue(instant(() -> targetVelocity-= 100.0));
        commandGamepad.dpadLeft().onTrue(instant(() -> targetVelocity-= 20.0));
        commandGamepad.dpadRight().onTrue(instant(() -> targetVelocity+= 20.0));
        commandGamepad.rightBumper().onTrue(instant(this::on));
        commandGamepad.leftBumper().onTrue(instant(this::off));
    }

    double error = 0.0;
    @Override
    public void periodic() {
        error = targetVelocity - this.getVelocity();
        if (isActive) {
            if (error > 30) {
                motorOne.setThrottle(1.0);
            } else if (error < -30) {
                motorOne.setThrottle(0.0);
            } //Set Velocity setpoint doesn't appear to be doing anything
//            else {
//                motorOne.setVelocitySetpoint(RotationsPerMinute.of(targetVelocity));
//            } //had to add because .setVelocitySetpoint was not triggering when velocity too low -- PID issue?
            motorOne.update();
        } else {
            motorOne.setVelocitySetpoint(RotationsPerMinute.of(0));
        }
        Telemetry.log("Flywheel Velocity:", this.getVelocity());
        Telemetry.log("Target Velocity:", this.targetVelocity);
        Telemetry.log("is active?", isActive);
        Telemetry.log("Control type:", motorOne.getControlType());
    }
}















