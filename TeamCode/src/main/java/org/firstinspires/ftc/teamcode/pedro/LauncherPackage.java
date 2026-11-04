package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.loop;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LauncherPackage {

    private DcMotorEx launcher;
    private Servo hood;
    private Servo turret;
    private CRServo feeder;
    private double velocity = 1700;

    private Gamepad gamepad;

    private Command goLaunch;
    private Telemetry telemetry;

    public LauncherPackage(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        this.launcher = hardwareMap.get(DcMotorEx.class,"launcher");
//        this.hood = hardwareMap.get(Servo.class, "hood");
        this.turret = hardwareMap.get(Servo.class, "turret");
        this.feeder = hardwareMap.get(CRServo.class, "feeder");
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        this.goLaunch = totalLaunch();


    }

    public Command launching() {
        return Command.build()
                .setStart(() -> {
                    // executed on start
                })
                .setExecute(() -> {
                    launcher.setVelocity(velocity);
                })
                .setDone(() -> {
                    // return true to end the command
                    return (Math.abs(launcher.getVelocity() - velocity) < 50);
                });
    }

    public Command rotating() {
        return Command.build()
                .setStart(() -> {
                    // executed on start
                })
                .setExecute(() -> {
//                Aim.hone(); //centers the hive into the middle
                })
                .setDone(() -> {
                    // return true to end the command
                    return true;
//                return Aim.aligned(); //return true when aim is centered
                });
    }

    public Command feeding() {
        return Command.build()
                .setStart(() -> {
                    // executed on start
                })
                .setExecute(() -> {
                    feeder.setPower(1);
                })
                .setDone(() -> {
                    // return timer.
                    return true; //return true after 5s?
                });
    }

    public Command totalLaunch() {
        return sequential(
                rotating(),
                launching(),
                feeding(),
                waitMs(5000)
        );
    }

    public void update() {
        if (gamepad.yWasPressed()) {
            if (goLaunch.isScheduled()) {
                telemetry.addLine("goLaunch has been scheduled");
            } else {
                goLaunch.schedule();
//                Scheduler.schedule(goLaunch);
                telemetry.addLine("schedule goLaunch");
            }

        }

        if (gamepad.dpadDownWasPressed()) {
            velocity -= 10;
            if (velocity < 600) {
                velocity = 600;
            }
        }

        if (gamepad.dpadUpWasPressed()) {
            velocity += 10;
            if (velocity > 2700) {
                velocity = 2700;
            }
        }

        telemetry.addLine("launcher.update");
        telemetry.addData("velocity: ", "%.0f", velocity);
    }
}
