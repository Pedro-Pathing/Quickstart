package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;


public class Tuning {
    // Tuners go here
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }
}
