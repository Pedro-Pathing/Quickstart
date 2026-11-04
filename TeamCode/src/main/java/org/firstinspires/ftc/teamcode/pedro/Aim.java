package org.firstinspires.ftc.teamcode.pedro;




import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.BooleanSupplier;

public class Aim {

    private Limelight3A limelight;
    private Servo turret;
    private Servo angular;
    private boolean aligned;

    public Aim(Limelight3A limelight, Servo turret, Servo angular) {
        this.limelight = limelight;
        this.turret = turret;
        this.angular = angular;
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight"); // or logitech Webcam
        this.turret = hardwareMap.get(Servo.class, "turret");
        this.angular = hardwareMap.get(Servo.class, "angular");
    }

    public boolean aim() {
        Scheduler.schedule(align);
        return aligned;

    }
    Command align = Command.build()
            .setStart(() -> {
                // executed on start
            })
            .setExecute(() -> {
                // if(hive position is left by 10) servo move left; else if(hive position is right by 10) servo move right;
            })
            .setDone(() -> {
                // return true when position has a tolerance of 5 degrees(or more)
            })
            .setEnd(endCondition -> {
                aligned = true;
            })
            .requiring(/* requirements */)
            .setInterruptedBehavior(/* interrupted behavior */)
            .setBlockedBehavior(/* blocked behavior */)
            .setConflictBehavior(/* conflict behavior */)
            .setPriority(/* priority */);
}
