package org.firstinspires.ftc.teamcode.kiwi;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.pedropathing.config.Validator.nonnegative;
import static com.pedropathing.config.Validator.positive;

/**
 * Constants for the 3-wheel Kiwi drivetrain.
 * Motor A drives along the Y axis (left/right), Motor B is front-left and Motor C is front-right.
 */
public class KiwiConfig {
    public final ConfigVar<String> motorAName = ConfigVar.required();
    public final ConfigVar<String> motorBName = ConfigVar.required();
    public final ConfigVar<String> motorCName = ConfigVar.required();
    public final ConfigVar<DcMotorSimple.Direction> motorADirection = ConfigVar.required();
    public final ConfigVar<DcMotorSimple.Direction> motorBDirection = ConfigVar.required();
    public final ConfigVar<DcMotorSimple.Direction> motorCDirection = ConfigVar.required();

    /** Whether ZeroPowerBrake mode is enabled in manual mode. */
    public final ConfigVar<Boolean> manualBrakeMode = ConfigVar.of(true);

    /** Smallest power change that triggers a hardware write. */
    public final ConfigVar<Double> powerThreshold = ConfigVar.of(0.01, nonnegative());

    /** Whether wheel powers are scaled by battery voltage. */
    public final ConfigVar<Boolean> voltageCompensation = ConfigVar.of(false);
    public final ConfigVar<Double> nominalVoltage = ConfigVar.of(12.0, positive());
    public final ConfigVar<Double> staticFrictionCoefficient = ConfigVar.of(0.1, nonnegative());

    public KiwiConfig(Configuration<KiwiConfig> config) {
        config.configure(this);
    }
}
