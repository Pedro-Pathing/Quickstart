package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

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

import org.firstinspires.ftc.teamcode.commands.ShootWhenReady;
import org.firstinspires.ftc.teamcode.commands.field.ResetPose;
import org.firstinspires.ftc.teamcode.commands.turret.SetTurretSide;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.CellLocator;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Config
@TeleOp(name = "Turret TeleOp")
public class TurretTeleOp extends CommandOpMode {
    public static double START_X_IN = 72;
    public static double START_Y_IN = 72;
    public static double START_HEADING_DEG = 90;

    private Follower follower;
    private Turret turret;
    private Shooter shooter;
    private Limelight limelight;
    private GamepadEx driver;
    private GamepadEx operator;
    private TelemetryData telemetryData;

    private Field.Alliance alliance;
    private boolean useSavedPose;
    private boolean shooterOn = false;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        TurretHistory history = new TurretHistory();
        limelight = new Limelight(hardwareMap, history);
        turret = new Turret(hardwareMap, limelight, history, follower::pose);
        shooter = new Shooter(hardwareMap, turret::getDistance);
        register(limelight, turret, shooter);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);

        alliance = PosePersistency.getAlliance(Field.Alliance.BLUE);
        useSavedPose = PosePersistency.hasPose();

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> {
            shooterOn = !shooterOn;
            shooter.setFlywheel(shooterOn ? ShooterConstants.TUNING_RPM : 0, shooterOn);
        });
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ShootWhenReady(() -> turret.isAimed() && shooter.flywheelReady()));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new ResetPose(follower, Field.Alliance.RED));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new ResetPose(follower, Field.Alliance.BLUE));
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
        telemetryData.addData("Controls", "X = BLUE, B = RED, Y = ignore saved pose and zero turret here");
        telemetryData.addData("Alliance", alliance);
        telemetryData.addData("Start pose", useSavedPose ? "saved " + PosePersistency.getPose(null) : "dashboard START_*");
        telemetryData.addData("Turret deg", Math.toDegrees(turret.getAngle()));
        telemetryData.addData("Encoder fault", turret.isEncoderFault());
        telemetryData.update();
    }

    @Override
    public void preRun() {
        Pose start = new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));
        follower.setPose(useSavedPose ? PosePersistency.getPose(start) : start);
        schedule(new SetTurretSide(turret, alliance));
        turret.setState(Turret.State.AIM);
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

        super.run();

        PosePersistency.savePose(follower.pose());

        StringBuilder ids = new StringBuilder();
        for (CellLocator.TagPoint tag : limelight.getTagPoints()) ids.append(tag.id).append(' ');
        telemetryData.addData("Seen tag IDs", ids.toString().trim());
        turret.addTelemetry(telemetryData);
        shooter.addTelemetry(telemetryData);
        telemetryData.update();
    }

    @Override
    public void end() {
        PosePersistency.savePose(follower.pose());
    }
}
