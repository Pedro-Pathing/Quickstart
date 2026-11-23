package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.controllers.Controller;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Flywheel {

    private static final double TICKS_PER_REV = 145.1;
    private final DcMotorEx motorOne;
    private final DcMotorEx motorTwo;
    private final Telemetry telemetry;

    private Controller psv = Controller.sum(
            Controller.proportionalFeedforward(0.0075),
            Controller.proportional(0.1),
            Controller.staticFeedforward(0.0065));
    private Mode mode = Mode.OFF;
    private double targetVelocity = 1400.0;
    private double targetPosition = 0.0;
    private double home = 0.0;
    public Flywheel(OpMode opMode) {
        motorOne = opMode.hardwareMap.get(DcMotorEx.class, "flywheelMotorOne");
        motorTwo = opMode.hardwareMap.get(DcMotorEx.class, "flywheelMotorTwo");
        motorTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = opMode.telemetry;
    }

    public void setTargetPosition(double target) {
        this.targetPosition = target;
    }

    public Command useInterpolation = instant(() -> mode = Mode.VELOCITY);
    public Command runWithVelocity(double Velocity) {
        return sequential(setTargetVelocity(Velocity), usePosition);
    }
    public Command usePosition = instant(() -> mode = Mode.POSITION);
    public Command off = instant(() -> mode = Mode.OFF);


    public Command toggle() {
        return conditional(() -> mode == Mode.OFF, useInterpolation, off);
    }

    public void adjustTarget(double change) {
        targetVelocity += change;
        setTargetVelocity(targetVelocity);
    }

    public Command setTargetVelocity(double newVelocity) {
        return instant(() -> this.targetVelocity = newVelocity);
    }

    public double getVelocity() {
        return motorOne.getVelocity(AngleUnit.DEGREES)/6.0;
    }

    public void setPower(double power) {
        motorOne.setPower(power);
        motorTwo.setPower(power);
    }


    public Command shootThree(Double velocity) {
        return sequential(
                runWithVelocity(velocity),
                waitMs(200),
                off
        );
    }

    public Command shootThree() {
        return sequential(
                useInterpolation,
                waitMs(200),
                off
        );
    }

    double error = 0.0;

    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case VELOCITY:
                    setPower(psv.calculate(getVelocity(), targetVelocity));
                    return;
                case OFF:
                    setPower(0);
                    return;
                case POSITION:
                    setPower(psv.calculate(motorOne.getCurrentPosition(), targetPosition));
                    break;
            }

        });
    }


    enum Mode {
        OFF,
        VELOCITY,
        POSITION
    }
}















