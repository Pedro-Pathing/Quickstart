package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import static com.sun.tools.javac.util.Constants.format;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.field.ResetPose;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOff;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOn;
import org.firstinspires.ftc.teamcode.commands.intake.intakeReverse;
import org.firstinspires.ftc.teamcode.commands.intake.intakeShooting;
import org.firstinspires.ftc.teamcode.commands.intake.intakeToShooter;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.vision.CellLocator;
import org.firstinspires.ftc.teamcode.vision.CellObservation;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.Limelight;

import java.util.Locale;

@Config
@TeleOp(name = "Tuning TeleOp", group = "Tuning")
public class TuningTeleOp extends CommandOpMode {
    public static double START_X_IN = 40;
    public static double START_Y_IN = 8;
    public static double START_HEADING_DEG = 90;
    public static double MERGE_DISTANCE_IN = 3;

    private Follower follower;
    private Turret turret;
    private Shooter shooter;
    private Limelight limelight;
    private TelemetryData telemetryData;
    private ShotTable shotTable;
    Intake intake;

    private Field.Alliance alliance = PosePersistency.getAlliance(Field.Alliance.BLUE);
    private boolean useSavedPose = PosePersistency.hasPose();
    private boolean shooterOn = true;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        TurretHistory history = new TurretHistory();
        limelight = new Limelight(hardwareMap, history);
        turret = new Turret(hardwareMap, limelight, history, follower::pose);
        shooter = new Shooter(hardwareMap, turret::getDistance);
        register(limelight, turret, shooter);
        intake = new Intake(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);
        shotTable = new ShotTable(MERGE_DISTANCE_IN);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(new intakeOn(intake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(new intakeOff(intake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(new intakeToShooter(intake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenInactive(new intakeOff(intake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenActive(new intakeReverse(intake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenInactive(new intakeOff(intake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new ResetPose(follower, () -> alliance));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> shooter.setDistanceControl(!shooter.getDistanceControl()));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> shooterOn = !shooterOn);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            if (!shooter.getDistanceControl()) shotTable.record(turret.getDistance(), ShooterConstants.TUNING_RPM);
        });
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> shotTable.clear());
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.x) alliance = Field.Alliance.BLUE;
        if (gamepad1.b) alliance = Field.Alliance.RED;
        if (gamepad1.y) {
            useSavedPose = false;
            turret.zeroHere();
        }

        turret.readEncoder();
        telemetryData.addData("Controls", "X = BLUE, B = RED, Y = ignore saved pose and zero turret here. Hand-turn the turret now for the encoder checks");
        telemetryData.addData("Alliance", alliance);
        telemetryData.addData("Start pose", useSavedPose ? "saved " + PosePersistency.getPose(null) : "dashboard START_*");
        telemetryData.addData("Turret encoder ticks", turret.getTicks());
        telemetryData.addData("Turret angle deg", Math.toDegrees(turret.getAngle()));
        telemetryData.addData("Turret encoder fault", turret.isEncoderFault());
        telemetryData.update();
    }

    @Override
    public void preRun() {
        Pose start = new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));
        follower.setPose(useSavedPose ? PosePersistency.getPose(start) : start);
        turret.setAlliance(alliance);
        turret.setState(Turret.State.AIM);
        PosePersistency.saveAlliance(alliance);
    }

    @Override
    public void run() {
        follower.manual(ManualDrive.fieldCentric(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading() + (alliance == Field.Alliance.RED ? Math.PI : 0)
        ));
        follower.update();
        shooter.setFlywheel(shooterOn ? ShooterConstants.TUNING_RPM : 0, shooterOn);
        if (TurretConstants.USE_MANUAL_ANGLE) {
            turret.setManualAngle(Math.toRadians(TurretConstants.MANUAL_ANGLE_DEG));
        } else {
            turret.setState(Turret.State.AIM);
        }

        super.run();
        PosePersistency.savePose(follower.pose());

        telemetryData.addData("Controls", "A = shooter AUTO/TUNING, B = shooter OFF/ON, dpad up = record, dpad down = clear");
        telemetryData.addData("Shooter mode", !shooterOn ? "OFF" : shooter.getDistanceControl() ? "AUTO" : "TUNING");
        telemetryData.addData("Manual angle", TurretConstants.USE_MANUAL_ANGLE
                ? TurretConstants.MANUAL_ANGLE_DEG + " deg" : "off (auto aim)");
        telemetryData.addData("Pose", String.format(Locale.US, "%.1f, %.1f, %.1f deg",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading())));
        turret.addTelemetry(telemetryData);
        shooter.addTelemetry(telemetryData);

        StringBuilder ids = new StringBuilder();
        for (CellLocator.TagPoint tag : limelight.getTagPoints()) ids.append(tag.id).append(' ');
        telemetryData.addData("Seen tag IDs", ids.toString().trim());
        telemetryData.addData("Frame age ms", limelight.getFrameAgeMs());
        CellObservation obs = turret.getCell() == null ? null : limelight.getObservation(turret.getCell());
        telemetryData.addData("Cell tags / bearing deg / distance in", obs == null ? "not seen"
                : String.format(Locale.US, "%d / %.1f / %.1f", obs.tagCount(), Math.toDegrees(obs.bearing), obs.distance));

        telemetryData.addData("Recorded points", shotTable.size());
        telemetryData.addData("Paste dist", shotTable.distanceLine());
        telemetryData.addData("Paste RPM", shotTable.velocityLine());


        if (TurretConstants.SHOW_TAGS) {
            for (CellLocator.TagPoint tag : limelight.getTagPoints()) {
                telemetryData.addData("Tag " + tag.id + " fwd/left/up in",
                        String.format(Locale.US, "%.1f / %.1f / %.1f", tag.forward, tag.left, tag.up));
            }
        }
        telemetryData.update();
    }

    @Override
    public void end() {
        PosePersistency.savePose(follower.pose());
        TurretConstants.USE_MANUAL_ANGLE = false;
    }
}
