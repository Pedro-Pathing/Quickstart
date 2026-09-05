package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class OTOSTuner extends Procedure {
    public OTOSTuner() {
        super("OTOS Tuner", "A procedure for tuning the OTOS localizer.");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs inputs = inputs("Setup", "Set OTOS HardwareMap Name");
        Inputs.Field<String> name = inputs.s("HardwareMap Name").withDefault("otos");
        awaitInputs(inputs);

        Inputs scalar = inputs("Scalar Identification", "Set the distance you will push your robot forward in inches and degrees you will turn your robot");
        Inputs.Field<Double> distance = scalar.d("Distance to push robot").withDefault(48.0);
        Inputs.Field<Integer> turns = scalar.i("Degrees to turn robot").withDefault(10);
        awaitInputs(scalar);

        Double linearScalar = runOpMode(new OTOSLinearScalar(name.get(), distance.get()));
        Double angularScalar = runOpMode(new OTOSAngularScalar(name.get(), turns.get() * 2 * Math.PI));

        List<Double> offsets = runOpMode(new OTOSOffsets(name.get(), linearScalar, angularScalar));

        result("name", name.get());
        result("linearScalar", linearScalar);
        result("angularScalar", angularScalar);
        result("xOffset", offsets.get(0));
        result("yOffset", offsets.get(1));

        code(Language.JAVA,"public static OTOSConfig config = new OTOSConfig(c -> {\n" +
                "    c.name.set(\"" + name.get() + "\");\n" +
                "    c.linearScalar.set(" + linearScalar + ");\n" +
                "    c.angularScalar.set(" + angularScalar + ");\n" +
                "    c.xPodOffset.set(" + offsets.get(0) + ");\n" +
                "    c.yPodOffset.set(" + offsets.get(1) + ");\n" +
                "    c.linearUnit.set(DistanceUnit.INCH);\n" +
                "});");
    }
}

class OTOSLinearScalar extends TuningOpMode<Double> {
    String name;
    double distance;

    public OTOSLinearScalar(String name, double distance) {
        super("Custom Scalar Identification",
                "Determines the linear scalar for the OTOS localizer. \n"
                        + "Push your robot forward " + distance + " inches exactly and then stop the Opmode",
                true);
        this.name = name;
        this.distance = distance;
    }

    @Override
    protected Double runTuningOpMode() {
        OTOSConfig config = new OTOSConfig(c -> {
            c.name.set(name);
            c.linearUnit.set(DistanceUnit.INCH);
            c.linearScalar.set(1.0);
            c.offset.set(Pose.zero());
        });
        OTOSLocalizer localizer = new OTOSLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        waitForStart();
        while (!isStopRequested()) {
            localizer.update();
        }
        return (distance / (localizer.pose().x()));
    }
}

class OTOSAngularScalar extends TuningOpMode<Double> {
    String name;
    double rad;
    double totalHeading = 0;
    double prevHeading = 0;

    public OTOSAngularScalar(String name, double rad) {
        super("Custom Scalar Identification",
                "Determines the angular scalar for the OTOS localizer. \n"
                        + "Spin your robot " + rad + " degrees and then stop the Opmode",
                true);
        this.name = name;
        this.rad = rad;
    }

    @Override
    protected Double runTuningOpMode() {
        OTOSConfig config = new OTOSConfig(c -> {
            c.name.set(name);
            c.angularScalar.set(1.0);
            c.offset.set(Pose.zero());
        });
        OTOSLocalizer localizer = new OTOSLocalizer(hardwareMap, config);
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
        return Math.abs((rad / totalHeading));
    }
}

class OTOSOffsets extends TuningOpMode<List<Double>> {
    String name;
    double linearScalar, angularScalar;
    private Pose previous = Pose.zero();

    public OTOSOffsets(String name, double linearScalar, double angularScalar) {
        super("OTOSOffsets Identification",
                "Automatically identifies the offsets for your OTOS localizer. \n"
                        + "Spin your robot in place 180 degrees and then stop the Opmode",
                true);
        this.name = name;
        this.linearScalar = linearScalar;
        this.angularScalar = angularScalar;
    }

    @Override
    protected List<Double> runTuningOpMode() {
        OTOSConfig config = new OTOSConfig(c -> {
            c.name.set(name);
            c.linearScalar.set(linearScalar);
            c.angularScalar.set(angularScalar);
            c.offset.set(Pose.zero());
        });
        OTOSLocalizer localizer = new OTOSLocalizer(hardwareMap, config);
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