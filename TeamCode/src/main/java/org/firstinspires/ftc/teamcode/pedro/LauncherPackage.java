package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.ivy.groups.Groups.loop;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class LauncherPackage {

    private DcMotorEx launcher;
    private Servo hood;
    private Servo turret;
    private CRServo feeder;
    private double velocity = 600.0;

    public LauncherPackage(DcMotorEx launcher, Servo hood, Servo turret, Servo gate) {
        this.launcher = launcher;
        this.hood = hood;
        this.turret = turret;
        this.feeder = feeder;
        launcher = hardwareMap.get(DcMotorEx.class,"launcher");
        hood = hardwareMap.get(Servo.class, "hood");
        turret = hardwareMap.get(Servo.class, "turret");
        feeder = hardwareMap.get(CRServo.class, "feeder");

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
                return false;
            });

    Command rotating = Command.build()
            .setStart(() -> {
                // executed on start
            })
            .setExecute(() -> {

            })
            .setDone(() -> {
                // return true to end the command
                return true;
            });

    Command feeding = Command.build()
            .setStart(() -> {
                // executed on start
            })
            .setExecute(() -> {
                feeder.setPower(1);
            })
            .setDone(() -> {
                // return true to end the command
                return true;
            });
    Command myCommand = sequential(
            launching,
            rotating,
            feeding
    );

    Command loopCommand = loop(myCommand);

}
