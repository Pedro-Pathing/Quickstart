package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.ShootWhenReady;
import org.firstinspires.ftc.teamcode.commands.field.ResetPose;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOff;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOn;
import org.firstinspires.ftc.teamcode.commands.intake.intakeReverse;
import org.firstinspires.ftc.teamcode.commands.intake.intakeToShooter;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotor;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotorConstants;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.CellLocator;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Config
@TeleOp(name = "Turret Motor TeleOp")
public class TurretMotorTeleOp extends CommandOpMode {
    public static double START_X_IN = 133;
    public static double START_Y_IN = 132;
    public static double START_HEADING_DEG = -90;

    private Follower follower;
    private Intake intake;
    private TurretMotor turret;
    private Shooter shooter;
    private Limelight limelight;
    private GamepadEx driver;
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
        turret = new TurretMotor(hardwareMap, limelight, history, follower::pose);
        shooter = new Shooter(hardwareMap, turret::getDistance);
        register(limelight, turret, shooter);
        intake = new Intake(hardwareMap);

        driver = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);

        alliance = PosePersistency.getAlliance(Field.Alliance.BLUE);
        useSavedPose = PosePersistency.hasPose();

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> {
            shooterOn = !shooterOn;
            shooter.setTargetRPM(ShooterConstants.TUNING_RPM);
            shooter.setState(shooterOn ? Shooter.State.DISTANCE : Shooter.State.OFF);
        });
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ShootWhenReady(() -> turret.isAimed() && shooter.flywheelReady()));
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> turret.setState(
                turret.getRequestedState() == TurretMotor.State.AIM ? TurretMotor.State.IDLE : TurretMotor.State.AIM));
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() ->
                TurretMotorConstants.MOTOR_USE_MANUAL_ANGLE = !TurretMotorConstants.MOTOR_USE_MANUAL_ANGLE);
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> stepManualAngle(1));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> stepManualAngle(-1));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new ResetPose(follower, Field.Alliance.RED));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new ResetPose(follower, Field.Alliance.BLUE));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(new intakeOn(intake));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(new intakeOff(intake));
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(new intakeToShooter(intake));
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenInactive(new intakeOff(intake));
        driver.getGamepadButton(GamepadKeys.Button.Y).whenActive(new intakeReverse(intake));
        driver.getGamepadButton(GamepadKeys.Button.Y).whenInactive(new intakeOff(intake));
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
        telemetryData.addData("Turret ticks", turret.getTicks());
        telemetryData.addData("Encoder fault", turret.isEncoderFault());
        telemetryData.update();
    }

    @Override
    public void preRun() {
        Pose start = new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));
        follower.setPose(useSavedPose ? PosePersistency.getPose(start) : start);
        turret.setAlliance(alliance);
        PosePersistency.saveAlliance(alliance);
        turret.setState(TurretMotor.State.AIM);
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

        if (TurretMotorConstants.MOTOR_USE_MANUAL_ANGLE) {
            turret.setManualAngle(Math.toRadians(TurretMotorConstants.MOTOR_MANUAL_ANGLE_DEG));
        } else if (turret.getRequestedState() == TurretMotor.State.MANUAL) {
            turret.setState(TurretMotor.State.AIM);
        }

        super.run();

        PosePersistency.savePose(follower.pose());

        StringBuilder ids = new StringBuilder();
        for (CellLocator.TagPoint tag : limelight.getTagPoints()) ids.append(tag.id).append(' ');
        telemetryData.addData("Controls", "RB shooter, A shoot, B aim/idle, Y manual angle on/off, DPAD U/D angle +/-, DPAD L/R reset pose");
        telemetryData.addData("Manual angle", TurretMotorConstants.MOTOR_USE_MANUAL_ANGLE
                ? TurretMotorConstants.MOTOR_MANUAL_ANGLE_DEG + " deg" : "off (auto aim)");
        telemetryData.addData("Seen tag IDs", ids.toString().trim());
        turret.addTelemetry(telemetryData);
        shooter.addTelemetry(telemetryData);
        telemetryData.update();
    }

    @Override
    public void end() {
        PosePersistency.savePose(follower.pose());
        TurretMotorConstants.MOTOR_USE_MANUAL_ANGLE = false;
    }

    private void stepManualAngle(int direction) {
        TurretMotorConstants.MOTOR_MANUAL_ANGLE_DEG = Range.clip(
                TurretMotorConstants.MOTOR_MANUAL_ANGLE_DEG + direction * TurretMotorConstants.MOTOR_MANUAL_STEP_DEG,
                TurretMotorConstants.MOTOR_MIN_ANGLE_DEG, TurretMotorConstants.MOTOR_MAX_ANGLE_DEG);
    }
}
