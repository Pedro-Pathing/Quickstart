package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretTuning;
import org.firstinspires.ftc.teamcode.vision.Limelight;

import java.util.Locale;

@TeleOp(name = "Turret Tuning", group = "Tuning")
public class TurretTuningTeleOp extends CommandOpMode {
    private Follower follower;
    private Turret turret;
    private Shooter shooter;
    private TelemetryData telemetryData;

    private Field.Alliance alliance = Field.Alliance.BLUE;
    private boolean shooterOn = false;

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        TurretHistory history = new TurretHistory();
        Limelight limelight = new Limelight(hardwareMap, history);
        turret = new Turret(hardwareMap, limelight, history, follower::pose);
        shooter = new Shooter(hardwareMap, turret::getDistance);
        register(limelight, turret, shooter);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> TurretTuning.AUTO_AIM = !TurretTuning.AUTO_AIM);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                () -> TurretTuning.TARGET_DEG = TurretTuning.STEP_DEG);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                () -> TurretTuning.TARGET_DEG = -TurretTuning.STEP_DEG);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> TurretTuning.TARGET_DEG = 0);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> shooterOn = !shooterOn);
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.x) alliance = Field.Alliance.BLUE;
        if (gamepad1.b) alliance = Field.Alliance.RED;

        turret.readEncoder();
        telemetryData.addData("Controls", "X = BLUE, B = RED. Point the turret forward, then press start");
        telemetryData.addData("Alliance", alliance);
        telemetryData.addData("Encoder ticks", turret.getTicks());
        telemetryData.addData("Angle deg", Math.toDegrees(turret.getAngle()));
        telemetryData.addData("Encoder fault", turret.isEncoderFault());
        telemetryData.update();
    }

    @Override
    public void preRun() {
        follower.setPose(new Pose(TurretTuning.START_X_IN, TurretTuning.START_Y_IN,
                Math.toRadians(TurretTuning.START_HEADING_DEG)));
        turret.setAlliance(alliance);
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

        TurretTuning.apply();
        shooter.setFlywheel(shooterOn ? ShooterConstants.TUNING_RPM : 0, shooterOn);
        if (TurretTuning.AUTO_AIM) {
            turret.setState(Turret.State.AIM);
        } else {
            turret.setManualAngle(Math.toRadians(TurretTuning.TARGET_DEG));
        }

        super.run();

        Pose pose = follower.pose();
        telemetryData.addData("Controls", "A = AUTO AIM / MANUAL, B = shooter ON/OFF, LB = left STEP_DEG, RB = right STEP_DEG, Y = center");
        telemetryData.addData("Mode", TurretTuning.AUTO_AIM ? "AUTO AIM" : "MANUAL " + TurretTuning.TARGET_DEG + " deg");
        telemetryData.addData("Pose", String.format(Locale.US, "%.1f, %.1f, %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetryData.addData("Cell", turret.getCell() == null ? "none" : turret.getCell());
        telemetryData.addData("Tags seen", turret.isSeen());
        telemetryData.addData("Aim offset deg", TurretTuning.AIM_OFFSET_DEG);
        telemetryData.addData("Setpoint deg", Math.toDegrees(turret.getSetpoint()));
        telemetryData.addData("Angle deg", Math.toDegrees(turret.getAngle()));
        telemetryData.addData("Error deg", Math.toDegrees(turret.getSetpoint() - turret.getAngle()));
        telemetryData.addData("Velocity deg/s", Math.toDegrees(turret.getVelocity()));
        telemetryData.addData("Power", turret.getPower());
        telemetryData.addData("In range", turret.isInRange());
        telemetryData.addData("Settled", turret.isSettled());
        shooter.addTelemetry(telemetryData);
        telemetryData.addData("Aim distance in", turret.getDistance());
        telemetryData.addData("Turret aimed", turret.isAimed());
        telemetryData.addData("OK to shoot (aimed + shooter ready)", turret.isAimed() && shooter.flywheelReady());
        telemetryData.addData("Encoder fault", turret.isEncoderFault());
        telemetryData.addData("Copy to TurretConstants", String.format(Locale.US,
                "kP %s  kI %s  kD %s  kS %s  AIM_OFFSET_DEG %s",
                TurretTuning.kP, TurretTuning.kI, TurretTuning.kD, TurretTuning.kS, TurretTuning.AIM_OFFSET_DEG));
        telemetryData.update();
    }

    @Override
    public void end() {
        TurretTuning.AUTO_AIM = false;
    }
}
