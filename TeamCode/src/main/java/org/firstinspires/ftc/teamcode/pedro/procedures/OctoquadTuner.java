package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class OctoquadTuner extends Procedure {
    enum PodType {
        SWING_ARM,
        FOUR_BAR,
        CUSTOM
    }

    public static double SWING_ARM = 336.877962768;
    public static double FOUR_BAR = 505.316944406;

    public OctoquadTuner() {
        super("Octoquad Tuner", "A procedure for tuning the Octoquad localizer.");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs inputs = inputs("Setup", "Set Octoquad HardwareMap Name and Odometry Pod Type");
        Inputs.Field<String> octoquadName = inputs.s("HardwareMap Name").withDefault("octoquad");
        Inputs.Field<Integer> xPort =  inputs.i("Forward Pod Port").withDefault(0);
        Inputs.Field<Integer> yPort =  inputs.i("Strafe Pod Port").withDefault(1);
        Inputs.Field<OctoquadTuner.PodType> podType = inputs.e("Odometry Pod Type", OctoquadTuner.PodType.class).withDefault(OctoquadTuner.PodType.FOUR_BAR);
        Inputs.Field<OctoQuad.I2cRecoveryMode> recoveryMode = inputs.e("Recovery Mode", OctoQuad.I2cRecoveryMode.class).withDefault(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        awaitInputs(inputs);

        double customPodScalar = 0;

        Inputs inputsHeadingScalar = inputs("Custom Scalar Identification Turns", "Set the number of times you will turn your robot.");
        Inputs.Field<Integer> turns = inputsHeadingScalar.i("Turns").withDefault(10);
        awaitInputs(inputsHeadingScalar);
        double headingScalar = runOpMode(new OctoquadHeadingScalar(octoquadName.get(), turns.get(), xPort.get(), yPort.get()));

        if (podType.get() == PodType.CUSTOM) {
            Inputs inputsCustom = inputs("Custom Scalar Identification Push Distance", "Set the distance you will push your robot forward in inches");
            Inputs.Field<Double> distance = inputsCustom.d("Distance").withDefault(48.0);
            awaitInputs(inputsCustom);
            customPodScalar = runOpMode(new OctoquadCustomPodScalar(distance.get(), octoquadName.get(), xPort.get(), yPort.get()));
        }

        boolean forwardPodReversed = runOpMode(new OctoquadForwardDirection(octoquadName.get(), podType.get(), customPodScalar, headingScalar, xPort.get(), yPort.get()));
        boolean strafePodReversed = runOpMode(new OctoquadStrafeDirection(octoquadName.get(), podType.get(), customPodScalar, headingScalar, xPort.get(), yPort.get()));

        List<Double> offsets = runOpMode(new OctoquadOffsets(octoquadName.get(), podType.get(), customPodScalar, forwardPodReversed, strafePodReversed, headingScalar, xPort.get(), yPort.get()));

        result("name", octoquadName.get());

        if (podType.get() == PodType.CUSTOM) {
            result("podType", "Custom");
            result("ticksPerUnit", customPodScalar);
        } else {
            result("podType", podType.get() == PodType.SWING_ARM ? SWING_ARM : FOUR_BAR);
        }

        result("xPodDirection", forwardPodReversed ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
        result("yPodDirection", strafePodReversed ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
        result("xPodOffset", offsets.get(0));
        result("yPodOffset", offsets.get(1));

        code(Language.JAVA,"public static OctoquadConfig localizerConfig = new OctoquadConfig(c -> {\n" +
                "    c.name.set(\"" + octoquadName.get() + "\");\n" +
                "    c.xPodPort.set(" + xPort.get() + ");\n" +
                "    c.yPodPort.set(" + yPort.get() + ");\n" +
                (podType.get() == PodType.CUSTOM ? "    c.ticksPerUnit.set(customPodScalar);\n" : "    c..ticksPerUnit.set(" + (podType.get() == OctoquadTuner.PodType.SWING_ARM ? SWING_ARM : FOUR_BAR) + ");\n") +
                "    c.xPodOffset.set(" + offsets.get(0) + ");\n" +
                "    c.yPodOffset.set(" + offsets.get(1) + ");\n" +
                "    c.xPodDirection.set(" + (forwardPodReversed ? "Octoquad.EncoderDirection.REVERSE" : "Octoquad.FORWARD") + ");\n" +
                "    c.yPodDirection.set(" + (strafePodReversed ? "Octoquad.EncoderDirection.REVERSE" : "Octoquad.FORWARD") + ");\n" +
                "    c.globalDistanceUnit.set(DistanceUnit.INCH);\n" +
                "    c.offsetUnits.set(DistanceUnit.INCH);\n" +
                "    c.i2cRecoveryMode.set(" + recoveryMode + ");\n" +
                "    c.headingScalar.set(" + headingScalar + ");\n" +
                "});");
    }
}

class OctoquadHeadingScalar extends TuningOpMode<Double> {
    String name;
    int turns;
    double totalHeading = 0;
    double prevHeading = 0;
    int xPodPort;
    int yPodPort;

    public OctoquadHeadingScalar(String name, int turns, int xPodPort, int yPodPort) {
        super("Heading Scalar Identification",
                "Determines the scalar for the custom pods of the Octoquad localizer. \n"
                        + "Turn your robot " + turns * 360 + " degrees exactly ("+ turns + " times) exactly and then stop the OpMode.",
                true);
        this.name = name;
        this.turns = turns;
        this.xPodPort = xPodPort;
        this.yPodPort = yPodPort;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        OctoQuadConfig config = new OctoQuadConfig(c -> {
            c.name.set(name);
            c.xPodPort.set(xPodPort);
            c.yPodPort.set(yPodPort);
            c.ticksPerUnit.set(1.0);
            c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
        });

        OctoQuadLocalizer localizer = new OctoQuadLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        localizer.update();
        waitForStart();
        while (!isStopRequested()) {
            localizer.update();

            if (localizer.pose().x() != Pose.zero().x() || localizer.pose().y() != Pose.zero().y() || localizer.pose().heading() != Pose.zero().heading()) {
                double currentHeading = localizer.pose().heading();
                totalHeading += Angle.normalizeSigned(currentHeading - prevHeading);
                prevHeading = currentHeading;
            }
        }
        return Math.abs((turns * Math.PI * 2 / totalHeading));
    }
}

class OctoquadCustomPodScalar extends TuningOpMode<Double> {
    String name;
    double distance;
    int xPodPort, yPodPort;

    public OctoquadCustomPodScalar(Double distance, String name, int xPodPort, int yPodPort) {
        super("Custom Scalar Identification",
                "Determines the scalar for the custom pods of the Octoquad localizer. \n"
                        + "Push your robot forward " + distance + " inches exactly and then stop the Opmode",
                true);
        this.name = name;
        this.distance = distance;
        this.xPodPort = xPodPort;
        this.yPodPort = yPodPort;
    }

    @Override
    protected Double runTuningOpMode() {
        OctoQuadConfig config = new OctoQuadConfig(c -> {
            c.name.set(name);
            c.xPodPort.set(xPodPort);
            c.yPodPort.set(yPodPort);
            c.ticksPerUnit.set(1.0);
            c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
        });
        OctoQuadLocalizer localizer = new OctoQuadLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        waitForStart();
        while (!isStopRequested()) {
            localizer.update();
        }
        return Math.abs((distance / (localizer.pose().x())));
    }
}

class OctoquadForwardDirection extends TuningOpMode<Boolean> {
    String name;
    OctoquadTuner.PodType podType;
    double customPodScalar;
    double headingScalar;
    int xPodPort, yPodPort;

    public OctoquadForwardDirection(String name, OctoquadTuner.PodType podType, double customPodScalar, double headingScalar,
                                    int xPodPort, int yPodPort) {
        super("Forward Direction Identification",
                "Determines if your forward pod needs to be reversed. \n"
                        + "Push your robot forward and then stop the Opmode",
                true);
        this.name = name;
        this.podType = podType;
        this.customPodScalar = customPodScalar;
        this.headingScalar = headingScalar;
        this.xPodPort = xPodPort;
        this.yPodPort = yPodPort;
    }

    @Override
    protected Boolean runTuningOpMode() {
        OctoQuadConfig config = new OctoQuadConfig(c -> {
            c.name.set(name);
            c.xPodPort.set(xPodPort);
            c.yPodPort.set(yPodPort);
            c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            c.headingScalar.set(headingScalar);
            if (podType == OctoquadTuner.PodType.CUSTOM) {
                c.ticksPerUnit.set(customPodScalar);
            } else {
                c.ticksPerUnit.set(podType == OctoquadTuner.PodType.SWING_ARM ? OctoquadTuner.SWING_ARM : OctoquadTuner.FOUR_BAR);
            }
        });
        OctoQuadLocalizer localizer = new OctoQuadLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        waitForStart();
        while (!isStopRequested()) {
            localizer.update();
        }

        return localizer.pose().x() < 0;
    }
}

class OctoquadStrafeDirection extends TuningOpMode<Boolean> {
    String name;
    OctoquadTuner.PodType podType;
    double customPodScalar;
    double headingScalar;
    int xPodPort, yPodPort;

    public OctoquadStrafeDirection(String name, OctoquadTuner.PodType podType, double customPodScalar, double headingScalar,
                                   int xPodPort, int yPodPort) {
        super("Strafe Direction Identification",
                "Determines if your strafe pod needs to be reversed. \n"
                        + "Push your robot to the side and then stop the Opmode",
                true);
        this.name = name;
        this.podType = podType;
        this.customPodScalar = customPodScalar;
        this.headingScalar = headingScalar;
        this.xPodPort = xPodPort;
        this.yPodPort = yPodPort;
    }

    @Override
    protected Boolean runTuningOpMode() {
        OctoQuadConfig config = new OctoQuadConfig(c -> {
            c.name.set(name);
            c.xPodPort.set(xPodPort);
            c.yPodPort.set(yPodPort);
            c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.yPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            c.headingScalar.set(headingScalar);

            if (podType == OctoquadTuner.PodType.CUSTOM) {
                c.ticksPerUnit.set(customPodScalar);
            } else {
                c.ticksPerUnit.set(podType == OctoquadTuner.PodType.SWING_ARM ? OctoquadTuner.SWING_ARM : OctoquadTuner.FOUR_BAR);
            }
        });
        OctoQuadLocalizer localizer = new OctoQuadLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        waitForStart();
        while (!isStopRequested()) {
            localizer.update();
        }

        return localizer.pose().y() < 0;
    }
}

class OctoquadOffsets extends TuningOpMode<List<Double>> {
    String name;
    OctoquadTuner.PodType podType;
    double customPodScalar;
    boolean forwardPodReversed, strafePodReversed;
    double headingScalar;
    Pose previous;
    int xPodPort, yPodPort;

    public OctoquadOffsets(String name, OctoquadTuner.PodType podType, double customPodScalar, Boolean forwardPodReversed, Boolean strafePodReversed, double headingScalar,
                           int xPodPort, int yPodPort) {
        super("PinpointOffsets Identification",
                "Automatically identifies the offsets for your Octoquad localizer. \n"
                        + "Spin your robot in place 180 degrees and then stop the Opmode",
                true);
        this.name = name;
        this.podType = podType;
        this.customPodScalar = customPodScalar;
        this.forwardPodReversed = forwardPodReversed;
        this.strafePodReversed = strafePodReversed;
        this.headingScalar = headingScalar;
        this.xPodPort = xPodPort;
        this.yPodPort = yPodPort;
    }

    @Override
    protected List<Double> runTuningOpMode() {
        OctoQuadConfig config = new OctoQuadConfig(c -> {
            c.name.set(name);
            c.xPodPort.set(xPodPort);
            c.yPodPort.set(yPodPort);
            c.xPodDirection.set(forwardPodReversed ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
            c.yPodDirection.set(strafePodReversed ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD);
            if (!podType.equals(OctoquadTuner.PodType.CUSTOM)) {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(customPodScalar);
            } else {
                c.encoderResolutionUnit.set(DistanceUnit.INCH);
                c.ticksPerUnit.set(podType == OctoquadTuner.PodType.SWING_ARM ? OctoquadTuner.SWING_ARM : OctoquadTuner.FOUR_BAR);
            }
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            c.headingScalar.set(headingScalar);
        });
        OctoQuadLocalizer localizer = new OctoQuadLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        localizer.update();

        waitForStart();

        while (!isStopRequested()) {
            previous = localizer.pose();
            localizer.update();
        }

        if (localizer.pose().x() != Pose.zero().x() || localizer.pose().y() != Pose.zero().y()) {
            previous =  localizer.pose();
        }

        return List.of(((-previous.y()) / 2.0), ((-previous.x()) / 2.0));
    }
}
