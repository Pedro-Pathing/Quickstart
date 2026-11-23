package org.firstinspires.ftc.teamcode.control;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.lang.Math;


@TeleOp
public class psvTuner extends LinearOpMode {

    double kP =0.0, kS=0.0, kV=0.0;
    double[] prev = {kP, kS, kV};
    double scalar = 0.001;
    double target =0.0;

    @Override
    public void runOpMode() {
        Scheduler.reset();

        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        DcMotorEx motorOne = hardwareMap.get(DcMotorEx.class, "flywheelMotorOne");
        DcMotorEx motorTwo = hardwareMap.get(DcMotorEx.class, "flywheelMotorTwo");
        motorTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        Command kStuner = Command.build()
                .setStart(() -> {
                    prev[0] = kP; prev[1] = kS; prev[2] = kV;
                    kS =0.0; kP =0.0; kV =0.0;})
                .setExecute(() -> {
                    if (gamepad1.dpadLeftWasPressed()) {
                        scalar /= 10;
                    }
                    if (gamepad1.dpadRightWasPressed()) {
                        scalar *= 10;
                    }
                    if (gamepad1.dpadUpWasPressed()) {
                        kS += 0.25*scalar;
                    }
                    if (gamepad1.dpadUpWasPressed()) {
                        kS -= 0.25*scalar;
                    }
                    telemetry.addData("kS tuner is running, all constants " +
                            "have been reset", "Press left bumper when motor just" +
                            "\n barely begins to move without" +
                            "\n stalling, increment with arrows");
                    telemetry.addData("kS", kS);
                    telemetry.addData("increment", 0.25*scalar);
                    telemetry.addData("previous constants", prev);
                })
                .setDone(() -> gamepad1.leftBumperWasPressed());



        waitForStart();

        while (opModeIsActive()) {
            motorOne.setPower(kP*(target-motorOne.getVelocity()) + kV*target + kS* Math.signum(target));
            if (gamepad1.aWasPressed() && !Scheduler.isScheduled(kStuner)) {
                Scheduler.schedule(kStuner);
            }
            panelsTelemetry.addData("Flywheel Target Velocity", target);
            panelsTelemetry.addData("Flywheel Velocity", motorOne.getVelocity());
            Scheduler.execute();
            telemetry.update();
        }
    }
}
