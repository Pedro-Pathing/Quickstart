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
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.IntakeFlower.IntakeFlowerDown;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOff;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOn;
import org.firstinspires.ftc.teamcode.commands.intake.intakeShooting;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intakeFlower.IntakeFlower;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Autonomous(name = "fullSoloAndPark", group = "Autonomous")
public class fullSoloAndPark extends CommandOpMode {

    private Follower follower;
    private Intake intake;
    private IntakeFlower intakeFlower;
    private Shooter shooter;
    private Turret turret;
    private Field.Alliance alliance;

    private  Pose start;
    private  Pose initToGarden;
    private  Pose gardenAdjustAngleStart;
    private  Pose gardenAdjustAngle;
    private  Pose gardenToHive1;
    private  Pose gardenToHive1Control1;
    private  Pose gardenToHive1Control2;
    private  Pose hive1AdjustAngleStart;
    private  Pose hive1AdjustAngle;
    private  Pose hive1ToHive2;
    private  Pose hive1ToHive2Control1;
    private  Pose hive2ToShootStart;
    private  Pose hive2ToShoot;
    private  Pose shootToPark;

    private void buildPoses(Field.Alliance alliance) {
        PoseFactory poseFactory = alliance == Field.Alliance.RED
                ? PoseFactory.degrees().mirrorAroundPoint(Field.CENTER_X, Field.CENTER_Y)
                : PoseFactory.degrees();
        start = poseFactory.of(86.7, 133.8, 0);
        initToGarden = poseFactory.of(132.9873, 133.1039, 0.1951);
        gardenAdjustAngleStart = poseFactory.of(132.9873, 133.1039, 0);
        gardenAdjustAngle = poseFactory.of(125.502, 125.6921, -110);
        gardenToHive1 = poseFactory.of(99.302, 12.0939, -107.1013);
        gardenToHive1Control1 = poseFactory.of(100.3544, 99.0334, 0);
        gardenToHive1Control2 = poseFactory.of(111.3959, 51.4023, 0);
        hive1AdjustAngleStart = poseFactory.of(99.302, 12.0939, -105);
        hive1AdjustAngle = poseFactory.of(104.7053, 23.0347, 90);
        hive1ToHive2 = poseFactory.of(127.9237, 94.8496, -0.8186);
        hive1ToHive2Control1 = poseFactory.of(106.1536, 95.1607, 0);
        hive2ToShootStart = poseFactory.of(127.9237, 94.8496, 0);
        hive2ToShoot = poseFactory.of(109.1074, 108.5861, -75);
        shootToPark = poseFactory.of(127.2001, 49.8667, -72.8749);
    }
    public Command autoRoutine() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> turret.isAimed() && shooter.flywheelReady()).withTimeout(AIM_TIMEOUT_MS),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeOn(intake),
                new FollowPathCommand(follower, initToGarden()),
                new intakeOff(intake),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new FollowPathCommand(follower, gardenAdjustAngle()),
                new IntakeFlowerDown(intakeFlower),
                new FollowPathCommand(follower, gardenToHive1()),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new FollowPathCommand(follower, hive1AdjustAngle()),
                new FollowPathCommand(follower, hive1ToHive2()),
                new FollowPathCommand(follower, hive2ToShoot()),
                new FollowPathCommand(follower, shootToPark())
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
        intakeFlower = new IntakeFlower(hardwareMap);
        register(limelight, turret, shooter, intake, intakeFlower);
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
        telemetry.update();
    }

    @Override
    public void preRun() {
        buildPoses(alliance);
        follower.setPose(start);
        follower.update();
        PosePersistency.saveAlliance(alliance);
        PosePersistency.savePose(start);
        turret.setAlliance(alliance);
        turret.setState(Turret.State.AIM);
        shooter.setTargetRPM(ShooterConstants.TUNING_RPM);
        shooter.setState(Shooter.State.FIXED);
        schedule(autoRoutine());
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        PosePersistency.savePose(follower.pose());

        telemetry.addData("Alliance", alliance);
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        telemetry.addData("Turret aimed", turret.isAimed());
        telemetry.addData("Shooter Ready", shooter.flywheelReady());

        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path number", follower.pathIndex());
        }

        telemetry.update();
    }

    @Override
    public void end() {
        intake.setState(Intake.State.OFF);
        PosePersistency.savePose(follower.pose());
    }

    public Path initToGarden() {
        return Paths.line(start, initToGarden).tangent();
    }

    public Path gardenAdjustAngle() {
        return Paths.line(gardenAdjustAngleStart, gardenAdjustAngle).linear(gardenAdjustAngleStart, gardenAdjustAngle);
    }

    public Path gardenToHive1() {
        return Paths.curve(gardenAdjustAngle, gardenToHive1Control1, gardenToHive1Control2, gardenToHive1).tangent();
    }

    public Path hive1AdjustAngle() {
        return Paths.line(hive1AdjustAngleStart, hive1AdjustAngle).linear(hive1AdjustAngleStart, hive1AdjustAngle);
    }

    public Path hive1ToHive2() {
        return Paths.curve(hive1AdjustAngle, hive1ToHive2Control1, hive1ToHive2).tangent();
    }

    public Path hive2ToShoot() {
        return Paths.line(hive2ToShootStart, hive2ToShoot).linear(hive2ToShootStart, hive2ToShoot);
    }

    public Path shootToPark() {
        return Paths.line(hive2ToShoot, shootToPark).tangent();
    }
}