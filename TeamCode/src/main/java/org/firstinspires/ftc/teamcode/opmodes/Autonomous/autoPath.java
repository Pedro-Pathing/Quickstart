package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import static org.firstinspires.ftc.teamcode.opmodes.Autonomous.testMatch.AIM_TIMEOUT_MS;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.intake.intakeOff;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOn;
import org.firstinspires.ftc.teamcode.commands.intake.intakeShooting;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.opmodes.TeleOP.TurretTuningTeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Autonomous(name = "autoPath", group = "Autonomous")
public class autoPath extends CommandOpMode {

    private Follower follower;
    private Intake intake;
    private Turret turret;
    private Shooter shooter;
    private Field.Alliance alliance;
    private Pose start;
    private Pose initGardenStart;
    private Pose initGarden;
    private Pose gardenToShoot1;
    private Pose shoot1ToHive1;
    private Pose hive1ToShoot2Start;
    private Pose hive1ToShoot2;
    private Pose shoot2ToHive2;
    private Pose hive2ToParkingStart;
    private Pose hive2ToParking;

    private void buildPoses(Field.Alliance alliance) {
        PoseFactory poseFactory = alliance == Field.Alliance.RED
                ? PoseFactory.degrees().mirrorAroundPoint(Field.CENTER_X, Field.CENTER_Y)
                : PoseFactory.degrees();
        start = poseFactory.of(86.7, 133.8, 0);
        initGardenStart = poseFactory.of(86.7    , 133.8594, 0);
        initGarden = poseFactory.of(134.4499, 134.1784, 0);
        gardenToShoot1 = poseFactory.of(97.9289, 23.6729, -100);
        shoot1ToHive1 = poseFactory.of(96.9, 13.0, -100.0781);
        hive1ToShoot2Start = poseFactory.of(95.9072, 12.2983, -100);
        hive1ToShoot2 = poseFactory.of(100.3939, 31.8151, 30);
        shoot2ToHive2 = poseFactory.of(127 , 92.6, 10);
        hive2ToParkingStart = poseFactory.of(128.4241, 92.6001, 0);
        hive2ToParking = poseFactory.of(128.988, 51.9602, -90);
    }

    // Autonomous routine
    public Command autoRoutine() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> turret.isAimed() && shooter.flywheelReady()).withTimeout(AIM_TIMEOUT_MS),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeOn(intake),
                new FollowPathCommand(follower, initGarden()),
                new intakeOff(intake),
                new FollowPathCommand(follower, gardenToShoot1()),
                new InstantCommand(() -> shooter.setFlywheel(ShooterConstants.TUNING_RPM, true)),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new InstantCommand(() -> shooter.setFlywheel(0, false)),
                new FollowPathCommand(follower, shoot1ToHive1()),
                // Esperar um intake pra flower
                new FollowPathCommand(follower, hive1ToShoot2()),
                new InstantCommand(() -> shooter.setFlywheel(ShooterConstants.TUNING_RPM, true)),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new InstantCommand(() -> shooter.setFlywheel(0, false)),
                new FollowPathCommand(follower, shoot2ToHive2()),
                // Esperar um intake pra flower
                // Preciso de uma pose pra shootar de novo
                new FollowPathCommand(follower, hive2ToParking())
        );
    }

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        TurretHistory history = new TurretHistory();
        Limelight limelight = new Limelight(hardwareMap, history);
        turret = new Turret(hardwareMap, limelight, history, follower::pose);
        shooter = new Shooter(hardwareMap, turret::getDistance);
        intake = new Intake(hardwareMap);
        register(limelight, turret, shooter);
        alliance = PosePersistency.getAlliance(Field.Alliance.BLUE);
    }

    @Override
    public void initialize_loop() {
        if (gamepad1.x) alliance = Field.Alliance.BLUE;
        if (gamepad1.b) alliance = Field.Alliance.RED;
        if (gamepad1.y) turret.zeroHere();
        telemetry.addData("Controls", "X = BLUE, B = RED, Y = ZERO HERE TURRET (point it forward)");
        telemetry.addData("Turret angle deg (point it forward)", Math.toDegrees(turret.getAngle()));
        telemetry.addData("Alliance", alliance);
        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path number", follower.pathIndex());
        }
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
        telemetry.addData("heading", follower.pose().heading());
        telemetry.addData("Turret aimed", turret.isAimed());
        telemetry.addData("Shooter Ready",shooter.flywheelReady());
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
    public Path initGarden() {
        return Paths.line(initGardenStart, initGarden).linear(initGardenStart, initGarden);
    }

    public Path gardenToShoot1() {
        return Paths.line(initGarden, gardenToShoot1).linear(initGarden, gardenToShoot1);
    }

    public Path shoot1ToHive1() {
        return Paths.line(gardenToShoot1, shoot1ToHive1).tangent();
    }

    public Path hive1ToShoot2() {
        return Paths.line(hive1ToShoot2Start, hive1ToShoot2).linear(hive1ToShoot2Start, hive1ToShoot2);
    }

    public Path shoot2ToHive2() {
        return Paths.line(hive1ToShoot2, shoot2ToHive2).linear(hive1ToShoot2, shoot2ToHive2);
    }

    public Path hive2ToParking() {
        return Paths.line(hive2ToParkingStart, hive2ToParking).linear(hive2ToParkingStart, hive2ToParking);
    }
}