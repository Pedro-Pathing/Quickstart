package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.triggers.CommandGamepad;

public class Intake implements Mechanism {

    private NextMotor motor = new NextMotor("intakeMotor");
    private double INTAKE_POWER = 1.0;
    private double OUTTAKE_POWER = 1.0;
    private double intake_power = 0.0;

    private boolean activated = false;

    private boolean auto = false;

//    public Intake(Robot robot) {
//        motor = robot.hardwareMap.get(DcMotorEx.class, "intakeMotor");
//
//        telemetry = robot.telemetry;
//    }

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
    public Command run(Double power) {
        return instant(() -> motor.setThrottle(power))
                .setEnd(endCondition -> motor.setThrottle(0.0))
                .requiring(motor);
    }

    public void start(CommandGamepad commandGamepad) {
        commandGamepad.leftTrigger().isOver(0.05)
                .whileTrue(instant(() -> this.run(commandGamepad.leftTrigger().invoke())))
                .onFalse(instant(() -> motor.setThrottle(0.0)));
    }

    public void setIntakePower(float power) {
        intake_power = power;
    }

    @Override
    public void periodic() {

    }


}
