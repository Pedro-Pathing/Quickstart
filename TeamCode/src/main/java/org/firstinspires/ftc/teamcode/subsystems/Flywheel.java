package org.firstinspires.ftc.teamcode.subsystems;

import static dev.nextftc.units.Units.RotationsPerMinute;

import org.firstinspires.ftc.teamcode.control.Math;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.units.measuretypes.AngularVelocity;

public class Flywheel implements Mechanism {
    private final NextMotor leftMotor = new NextMotor("shooterLeft");

    private final NextMotor rightMotor = new NextMotor("shooterRight");
    private Double targetVelocity = 3500.0;
    public boolean isActive = false;

//    public Flywheel(Robot robot) { //class constructor
//        leftMotor = new NextMotor("leftFlywheelMotor");
//        leftMotor.setDirection(NextMotor.Direction.REVERSE);
//
//        rightMotor = new NextMotor("rightFlywheelMotor");
//
//        telemetry = robot.telemetry;
//    }
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
        return rightMotor.getEncoderVelocity();
    }

    public void start(CommandGamepad commandGamepad) {
        leftMotor.setDirection(NextMotor.Direction.REVERSE);
        commandGamepad.dpadUp().onTrue(instant(() -> targetVelocity+= 100.0));
        commandGamepad.dpadDown().onTrue(instant(() -> targetVelocity-= 100.0));
        commandGamepad.dpadLeft().onTrue(instant(() -> targetVelocity+= 20.0));
        commandGamepad.dpadRight().onTrue(instant(() -> targetVelocity-= 20.0));
        commandGamepad.leftBumper().onTrue(instant(() -> isActive = true));
        commandGamepad.rightBumper().onTrue(instant(() -> isActive = false));
    }

    @Override
    public void periodic() {
        if (isActive) {
            rightMotor.setVelocitySetpoint(RotationsPerMinute.of(targetVelocity));
            leftMotor.setVelocitySetpoint(RotationsPerMinute.of(targetVelocity));
        } else {
            rightMotor.setVelocitySetpoint(RotationsPerMinute.of(0));
            leftMotor.setVelocitySetpoint(RotationsPerMinute.of(0));
        }
        Telemetry.log("Flywheel Velocity:", this.getVelocity());
    }
}















