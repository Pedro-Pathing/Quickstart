package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

public class Intake {

    private final DcMotorEx motor;
    private double INTAKE_POWER = 1.0;
    private double OUTTAKE_POWER = 1.0;
    private double intake_power = 0.0;

    private Telemetry telemetry;

    private boolean activated = false;

    private boolean auto = false;

    private Gamepad gamepad;

    public Intake(Robot robot) {
        motor = robot.hardwareMap.get(DcMotorEx.class, "intakeMotor");

        telemetry = robot.telemetry;

        gamepad = robot.gamepad2;
    }

    public Command intake() {
        return run(INTAKE_POWER);
    }
    public Command outtake() {
        return run(OUTTAKE_POWER);
    }

//    public Command run(Double power) {
//        return Command.build()
//                .setStart(() -> motor.setThrottle(power))
//                .setEnd(endCondition -> motor.setThrottle(0.0))
//                .requiring(this);
//    }
//    public Command run(Double power) {
//        return instant(() -> motor.setPower(power))
//                .setEnd(endCondition -> motor.setPower(0.0))
//                .requiring(motor);
//    }


    public void setIntakePower(float power) {
        intake_power = power;
    }

    public Command periodic() {
        ;
    }


}
