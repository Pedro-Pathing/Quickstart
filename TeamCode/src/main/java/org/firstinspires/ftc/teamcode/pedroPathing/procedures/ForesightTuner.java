package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Function;

public class ForesightTuner extends Procedure {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    public ForesightTuner(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Foresight Tuner", "A procedure for tuning the Foresight Algorithm.");
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    public void run() throws InterruptedException {
    }
}
