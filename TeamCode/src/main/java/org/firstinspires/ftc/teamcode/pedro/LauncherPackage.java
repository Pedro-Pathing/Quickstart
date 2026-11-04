package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.loop;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class LauncherPackage {

    private DcMotorEx launcher;
    private Servo hood;
    private Servo turret;
    private CRServo feeder;
    private double velocity = 600.0;

    public LauncherPackage(DcMotorEx launcher, Servo hood, Servo turret, CRServo feeder, Aim aim) {
        this.launcher = launcher;
        this.hood = hood;
        this.turret = turret;
        this.feeder = feeder;
        this.launcher = hardwareMap.get(DcMotorEx.class,"launcher");
        this.hood = hardwareMap.get(Servo.class, "hood");
        this.turret = hardwareMap.get(Servo.class, "turret");
        this.feeder = hardwareMap.get(CRServo.class, "feeder");

    }

    public void launch() {
        Scheduler.schedule(loopCommand);
    }

    //private Command launching;
    //private Command rotating;
    //private Command feeding;


    Command launching = Command.build()
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

    Command rotating = Command.build()
            .setStart(() -> {
                // executed on start
            })
            .setExecute(() -> {
                Aim.hone(); //centers the hive into the middle
            })
            .setDone(() -> {
                // return true to end the command
                return Aim.aligned(); //return true when aim is centered
            });

    Command feeding = Command.build()
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
    Command myCommand = sequential(
            rotating,
            launching,
            feeding,
            waitMs(5000)
    );

    Command loopCommand = loop(myCommand);

}
