package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private final DcMotorEx motor;
    private double INTAKE_POWER = 1.0;
    private double OUTTAKE_POWER = 1.0;
    private double intake_power = 0.0;

    private Telemetry telemetry;
    private Gamepad gamepad;
    public boolean isAuto = false;

    public Intake(OpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");

        telemetry = opMode.telemetry;
    }

    public Command intake() {
        return setIntakePower(1.0);
    }
    public Command outtake() {
        return setIntakePower(-1.0);
    }
    public Command off() {
        return setIntakePower(0.0);
    }

    public Command setIntakePower(double power) {
        return instant(()-> intake_power = power);
    }


}
