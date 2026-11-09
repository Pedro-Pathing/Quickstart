package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.tuning.autotune.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.io.ByteArrayInputStream;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

enum WheelTopMotion {
    @DisplayName("Anticlockwise") ANTICLOCKWISE,
    @DisplayName("Clockwise") CLOCKWISE
}

/**
 * Finds the motor directions for the Kiwi drivetrain.
 * A motor is FORWARD when positive power makes the top of its wheel move anticlockwise
 * around the robot (viewed from above), which is the direction that turns the robot CCW.
 */
public class KiwiTuner extends Procedure {
    private enum Wheel {
        A("Motor A (Rear)"),
        B("Motor B (Front Left)"),
        C("Motor C (Front Right)");

        final String displayName;

        Wheel(String displayName) {
            this.displayName = displayName;
        }
    }

    public KiwiTuner() {
        super("Kiwi Tuner", "A procedure to find the directions of kiwi drive wheels.");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs motorNames = inputs("Kiwi Motor Names", "Enter the names in HardwareMap of your drivetrain motors. Motor A is the rear wheel, which drives left and right.");
        Inputs.Field<String> motorAName = motorNames.s("Motor A (Rear) Name").withDefault("motorA");
        Inputs.Field<String> motorBName = motorNames.s("Motor B (Front Left) Name").withDefault("motorB");
        Inputs.Field<String> motorCName = motorNames.s("Motor C (Front Right) Name").withDefault("motorC");
        awaitInputs(motorNames);

        confirmation("Motor Directions", "Lift the robot so the wheels are clear of the ground. Each drivetrain motor will spin, one at a time. After each one, you will enter which way the top of the wheel moved around the robot, viewed from above. The diagram shows which wheel should be spinning and the correct direction.");

        Direction motorADirection = testMotor(Wheel.A, motorAName.get());
        Direction motorBDirection = testMotor(Wheel.B, motorBName.get());
        Direction motorCDirection = testMotor(Wheel.C, motorCName.get());

        result("motorAName", motorAName.get());
        result("motorBName", motorBName.get());
        result("motorCName", motorCName.get());
        result("motorADirection", motorADirection);
        result("motorBDirection", motorBDirection);
        result("motorCDirection", motorCDirection);

        code(Language.JAVA, "public static KiwiConfig drivetrainConfig = new KiwiConfig(c -> {\n" +
                "    c.motorAName.set(\"" + motorAName.get() + "\");\n" +
                "    c.motorBName.set(\"" + motorBName.get() + "\");\n" +
                "    c.motorCName.set(\"" + motorCName.get() + "\");\n" +
                "    c.motorADirection.set(DcMotorSimple.Direction." + motorADirection + ");\n" +
                "    c.motorBDirection.set(DcMotorSimple.Direction." + motorBDirection + ");\n" +
                "    c.motorCDirection.set(DcMotorSimple.Direction." + motorCDirection + ");\n" +
                "});");
    }

    private Direction testMotor(Wheel wheel, String hardwareName) throws InterruptedException {
        final boolean[] correctMotor = new boolean[1];
        final WheelTopMotion[] motion = new WheelTopMotion[1];

        withDisplay(Display.image(KiwiDiagram.LIGHT[wheel.ordinal()], KiwiDiagram.DARK[wheel.ordinal()]), () -> {
            runOpMode(new KiwiSpinMotor(wheel.displayName, hardwareName));

            Inputs inputs = inputs(wheel.displayName, "Determine the " + wheel.displayName + " direction.");
            Inputs.Field<Boolean> correctMotorField = inputs.b("Did the highlighted wheel spin?").withDefault(true);
            Inputs.Field<WheelTopMotion> motionField = inputs.e("Viewed from above, which way did the top of the wheel move around the robot?", WheelTopMotion.class);
            awaitInputs(inputs);

            correctMotor[0] = correctMotorField.get();
            motion[0] = motionField.get();
        });

        if (!correctMotor[0])
            abort("The wrong motor spun. Check that your motors are plugged into the correct ports, and that they are configured correctly. Then, try again.");

        return motion[0] == WheelTopMotion.ANTICLOCKWISE ? Direction.FORWARD : Direction.REVERSE;
    }
}

class KiwiSpinMotor extends TuningOpMode<Void> {
    private final String name;

    public KiwiSpinMotor(String displayName, String hardwareName) {
        super(displayName, displayName + " will spin. The diagram shows the correct direction. Click stop when you can see which way the top of the wheel is moving.", true);
        this.name = hardwareName;
    }

    @SuppressWarnings("StatementWithEmptyBody")
    @Override
    protected Void runTuningOpMode() {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        waitForStart();
        motor.setPower(0.5);
        while (opModeIsActive()) {
        }
        motor.setPower(0);
        return null;
    }
}

/**
 * Top-down SVG diagrams of the Kiwi drivetrain, one per wheel, with the tested wheel highlighted
 * and an arrow showing the direction the top of the wheel should move.
 */
final class KiwiDiagram {
    // Wheel centres, rolling (anticlockwise tangent) directions and outward normals, in SVG coordinates (y down)
    private static final double[][] CENTER = {{120, 200}, {50.7, 80}, {189.3, 80}};
    private static final double[][] TANGENT = {{1, 0}, {-0.5, 0.866}, {-0.5, -0.866}};
    private static final double[][] NORMAL = {{0, 1}, {-0.866, -0.5}, {0.866, -0.5}};
    private static final String[] LABEL = {"A", "B", "C"};

    static final ImageRegistrar.Handle[] LIGHT = new ImageRegistrar.Handle[3];
    static final ImageRegistrar.Handle[] DARK = new ImageRegistrar.Handle[3];

    static {
        for (int i = 0; i < 3; i++) {
            LIGHT[i] = register(svg(i, "#3c3c3c", "#e6e6e6"));
            DARK[i] = register(svg(i, "#dcdcdc", "#3a3a3a"));
        }
    }

    private KiwiDiagram() {
    }

    private static ImageRegistrar.Handle register(String svg) {
        byte[] bytes = svg.getBytes(StandardCharsets.UTF_8);
        return ImageRegistrar.registerSVG(() -> new ByteArrayInputStream(bytes));
    }

    private static String svg(int highlighted, String ink, String body) {
        String highlight = "#f28c28";
        String arrow = "#2e9e4f";
        StringBuilder s = new StringBuilder();
        s.append("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 240 240' width='240' height='240'>");
        s.append(String.format(Locale.US,
                "<defs><marker id='head' viewBox='0 0 10 10' refX='8' refY='5' markerWidth='6' markerHeight='6' orient='auto-start-reverse'>"
                        + "<path d='M0,0 L10,5 L0,10 z' fill='%s'/></marker>"
                        + "<marker id='front' viewBox='0 0 10 10' refX='8' refY='5' markerWidth='5' markerHeight='5' orient='auto-start-reverse'>"
                        + "<path d='M0,0 L10,5 L0,10 z' fill='%s'/></marker></defs>", arrow, ink));

        // Chassis and front indicator
        s.append(String.format(Locale.US, "<circle cx='120' cy='120' r='62' fill='%s' stroke='%s' stroke-width='2'/>", body, ink));
        s.append(String.format(Locale.US, "<line x1='120' y1='138' x2='120' y2='84' stroke='%s' stroke-width='3' marker-end='url(#front)'/>", ink));
        s.append(String.format(Locale.US, "<text x='120' y='154' font-family='sans-serif' font-size='12' text-anchor='middle' fill='%s'>FRONT</text>", ink));

        for (int i = 0; i < 3; i++) {
            double cx = CENTER[i][0], cy = CENTER[i][1];
            double angle = Math.toDegrees(Math.atan2(TANGENT[i][1], TANGENT[i][0]));
            String fill = i == highlighted ? highlight : body;
            s.append(String.format(Locale.US,
                    "<rect x='%.1f' y='%.1f' width='36' height='14' rx='3' fill='%s' stroke='%s' stroke-width='2' transform='rotate(%.1f %.1f %.1f)'/>",
                    cx - 18, cy - 7, fill, ink, angle, cx, cy));
            s.append(String.format(Locale.US,
                    "<text x='%.1f' y='%.1f' font-family='sans-serif' font-size='16' font-weight='bold' text-anchor='middle' dominant-baseline='central' fill='%s'>%s</text>",
                    cx - 28 * NORMAL[i][0], cy - 28 * NORMAL[i][1], ink, LABEL[i]));
        }

        // Arrow beside the highlighted wheel showing the anticlockwise direction of its top
        double ax = CENTER[highlighted][0] + 24 * NORMAL[highlighted][0];
        double ay = CENTER[highlighted][1] + 24 * NORMAL[highlighted][1];
        double tx = TANGENT[highlighted][0], ty = TANGENT[highlighted][1];
        s.append(String.format(Locale.US,
                "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='%s' stroke-width='4' marker-end='url(#head)'/>",
                ax - 22 * tx, ay - 22 * ty, ax + 22 * tx, ay + 22 * ty, arrow));

        s.append("</svg>");
        return s.toString();
    }
}
