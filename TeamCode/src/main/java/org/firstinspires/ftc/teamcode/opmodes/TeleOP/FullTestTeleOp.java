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

import org.firstinspires.ftc.teamcode.commands.IntakeFlower.IntakeFlowerDown;
import org.firstinspires.ftc.teamcode.commands.IntakeFlower.IntakeFlowerUp;
import org.firstinspires.ftc.teamcode.commands.TransferSequence;
import org.firstinspires.ftc.teamcode.commands.field.WallResetPose;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOff;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOn;
import org.firstinspires.ftc.teamcode.commands.intake.intakeReverse;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;
import org.firstinspires.ftc.teamcode.subsystems.indicator.Indicator;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intakeFlower.IntakeFlower;
import org.firstinspires.ftc.teamcode.subsystems.pollensensor.PollenSensor;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotor;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotorConstants;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Config
@TeleOp(name = "Full Test TeleOp")
public class FullTestTeleOp extends CommandOpMode {
    public static double START_X_IN = 133;
    public static double START_Y_IN = 132;
    public static double START_HEADING_DEG = -90;

    private Follower follower;
    private Intake intake;
    private IntakeFlower flower;
    private Gate gate;
    private PollenSensor pollenSensor;
    private Indicator indicator;
    private Shooter shooter;
    private TurretMotor turret;
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
        limelight = new Limelight(hardwareMap, history,
                () -> TurretMotorConstants.MOTOR_TURRET_FWD, () -> TurretMotorConstants.MOTOR_TURRET_LEFT);
        turret = new TurretMotor(hardwareMap, limelight, history, follower::pose);
        shooter = new Shooter(hardwareMap, turret::getDistance);
        intake = new Intake(hardwareMap);
        flower = new IntakeFlower(hardwareMap);
        gate = new Gate(hardwareMap);
        pollenSensor = new PollenSensor(hardwareMap);
        indicator = new Indicator(hardwareMap, turret::isAimed);
        register(limelight, turret, shooter, intake, flower, gate, pollenSensor, indicator);

        driver = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);

        alliance = PosePersistency.getAlliance(Field.Alliance.BLUE);
        useSavedPose = PosePersistency.hasPose();

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(new intakeOn(intake));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(new intakeOff(intake));
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new TransferSequence(intake, gate, null));
        driver.getGamepadButton(GamepadKeys.Button.Y).whenActive(new intakeReverse(intake));
        driver.getGamepadButton(GamepadKeys.Button.Y).whenInactive(new intakeOff(intake));

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            shooterOn = !shooterOn;
            shooter.setTargetRPM(ShooterConstants.TUNING_RPM);
            shooter.setState(shooterOn ? Shooter.State.DISTANCE : Shooter.State.OFF);
        });

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new IntakeFlowerUp(flower));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new IntakeFlowerDown(flower));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new WallResetPose(follower, WallResetPose.Axis.X));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new WallResetPose(follower, WallResetPose.Axis.Y));
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
        telemetryData.addData("Alliance", alliance + (alliance == PosePersistency.getAlliance(null) ? " (from auto)" : ""));
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

        super.run();

        PosePersistency.savePose(follower.pose());

        telemetryData.addData("Controls", "LB intake, RB transfer, Y reverse, A shooter, DPAD U/D flower up/down, DPAD L = wall reset X (side wall), DPAD R = wall reset Y (top/bottom wall)");
        telemetryData.addData("Alliance", alliance);
        telemetryData.addData("Pose", follower.pose());
        telemetryData.addData("Intake state", intake.getState());
        telemetryData.addData("Flower state", flower.getState());
        pollenSensor.addTelemetry(telemetryData);
        indicator.addTelemetry(telemetryData);
        gate.addTelemetry(telemetryData);
        shooter.addTelemetry(telemetryData);
        turret.addTelemetry(telemetryData);
        telemetryData.update();
    }

    @Override
    public void end() {
        PosePersistency.savePose(follower.pose());
        intake.setState(Intake.State.OFF);
        shooter.setState(Shooter.State.OFF);
    }
}
