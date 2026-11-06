package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.intake.intakeShooting;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Config
@Autonomous(name = "testMatch", group = "Autonomous")
public class testMatch extends CommandOpMode {
    public static long AIM_TIMEOUT_MS = 2000;

    private Follower follower;
    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private Field.Alliance alliance;
    private Pose start;
    private Pose startToParkStart;
    private Pose startToPark;
    private Pose startToParkControl1;

    private void buildPoses(Field.Alliance alliance) {
        PoseFactory poseFactory = alliance == Field.Alliance.RED
                ? PoseFactory.degrees().mirrorAroundPoint(Field.CENTER_X, Field.CENTER_Y)
                : PoseFactory.degrees();
        start = poseFactory.of(102, 133, -90);
        startToParkStart = poseFactory.of(102, 132.9127, -90);
        startToPark = poseFactory.of(129.8212, 48.0017, 300);
        startToParkControl1 = poseFactory.of(124.0664, 95.7325, 0);
    }

    public Command autoRoutine() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> turret.isAimed() && shooter.flywheelReady()).withTimeout(AIM_TIMEOUT_MS),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new InstantCommand(() -> shooter.setFlywheel(0, false)),
                new FollowPathCommand(follower, startToPark())
        );
    }

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        TurretHistory history = new TurretHistory();
        Limelight limelight = new Limelight(hardwareMap, history);
        turret = new Turret(hardwareMap, limelight, history, follower::pose);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, turret::getDistance);
        register(limelight, turret, shooter);
        alliance = PosePersistency.getAlliance(Field.Alliance.BLUE);
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.x) alliance = Field.Alliance.BLUE;
        if (gamepad1.b) alliance = Field.Alliance.RED;
        if (gamepad1.y) turret.zeroHere();
        telemetry.addData("Controls", "X = BLUE, B = RED, Y = zero turret here (point it forward)");
        turret.readEncoder();
        telemetry.addData("Turret angle deg (point it forward)", Math.toDegrees(turret.getAngle()));
        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void preRun() {
        buildPoses(alliance);
        follower.setPose(start);
        follower.update();
        PosePersistency.saveAlliance(alliance);
        turret.setAlliance(alliance);
        turret.setState(Turret.State.AIM);
        shooter.setDistanceControl(false);
         shooter.setFlywheel(ShooterConstants.TUNING_RPM, true);
        schedule(autoRoutine());
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        PosePersistency.savePose(follower.pose());

        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading deg", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Turret aimed", turret.isAimed());
        telemetry.addData("Shooter ready", shooter.flywheelReady());
        telemetry.addData("Turret angle deg", Math.toDegrees(turret.getAngle()));
        telemetry.addData("Shooter target / current RPM", shooter.getTargetRPM() + " / " + shooter.getRPM());

        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path number", follower.pathIndex());
        }

        telemetry.update();
    }

    @Override
    public void end() {
        intake.intakeOff();
        PosePersistency.savePose(follower.pose());
    }

    public Path startToPark() {
        return curve(startToParkStart, startToParkControl1, startToPark).linear(startToParkStart, startToPark);
    }
}
