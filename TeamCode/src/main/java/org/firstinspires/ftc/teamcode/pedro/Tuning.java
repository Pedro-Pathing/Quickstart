package org.firstinspires.ftc.teamcode.pedro;
import com.pedropathing.tuning.autotune.Tuner;
import com.pedropathing.tuning.autotune.Procedure;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;

public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
}
