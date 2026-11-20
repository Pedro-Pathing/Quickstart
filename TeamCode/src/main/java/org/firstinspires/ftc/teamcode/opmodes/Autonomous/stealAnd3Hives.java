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
import org.firstinspires.ftc.teamcode.commands.IntakeFlower.IntakeFlowerUp;
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


@Autonomous(name = "stealAnd3Hives", group = "Autonomous")
public class stealAnd3Hives extends CommandOpMode {


    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private IntakeFlower intakeFlower;
    private Field.Alliance alliance;


    private Pose start;
    private Pose initScoreSteal;
    private Pose initScoreStealControl1;
    private Pose turnToLoseTimeStart;
    private Pose turnToLoseTime;
    private Pose shootHiveDownShoot;
    private Pose toHiveSideStart;
    private Pose toHiveSide;
    private Pose toShootInGardenStart;
    private Pose toShootInGarden;
    private Pose toShootInGardenControl1;
    private Pose takeGarden;
    private Pose toParkStart;
    private Pose toPark;
    private Pose toParkControl1;


    private void buildPoses(Field.Alliance alliance) {
        PoseFactory poseFactory = alliance == Field.Alliance.RED
                ? PoseFactory.degrees().mirrorAroundPoint(Field.CENTER_X, Field.CENTER_Y)
                : PoseFactory.degrees();


        start = poseFactory.of(86, 133, 180);
        initScoreSteal = poseFactory.of(51.9891, 129.6223, 120);
        initScoreStealControl1 = poseFactory.of(68.8671, 121.9965, 0);
        turnToLoseTimeStart = poseFactory.of(51.9891, 129.6223, 120);
        turnToLoseTime = poseFactory.of(58.8171, 124.1608, -80);
        shootHiveDownShoot = poseFactory.of(92.9112, 11.9695, -80);
        toHiveSideStart = poseFactory.of(92.9112, 11.9695, -80);
        toHiveSide = poseFactory.of(129.5306, 95.1629, 0);
        toShootInGardenStart = poseFactory.of(129.5306, 95.1629, 0);
        toShootInGarden = poseFactory.of(133.2043, 126.9234, 90);
        toShootInGardenControl1 = poseFactory.of(123.4235, 107.5156, 0);
        takeGarden = poseFactory.of(133.5599, 133.2044, 90);
        toParkStart = poseFactory.of(133.5599, 133.2044, 90);
        toPark = poseFactory.of(126.4493, 49.7739, -90);
        toParkControl1 = poseFactory.of(123.5602, 104.0386, 0);
    }


    public Command autoRoutine() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> turret.isAimed() && shooter.flywheelReady()).withTimeout(AIM_TIMEOUT_MS),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeOn(intake),
                new IntakeFlowerDown(intakeFlower),
                new FollowPathCommand(follower, initScoreSteal()),
                new FollowPathCommand(follower, turnToLoseTime()),
                new FollowPathCommand(follower, shootHiveDownShoot()),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new FollowPathCommand(follower, toHiveSide()),
                new FollowPathCommand(follower, toShootInGarden()),
                new IntakeFlowerUp(intakeFlower),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new FollowPathCommand(follower, takeGarden()),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new intakeShooting(intake, shooter),
                new WaitCommand(300),
                new FollowPathCommand(follower, toPark())
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
        shooter.setTargetRPM(ShooterConstants.TUNING_RPM);
        shooter.setState(Shooter.State.FIXED);
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
        intake.setState(Intake.State.OFF);
        PosePersistency.savePose(follower.pose());
    }

    public Path initScoreSteal() {
        return Paths.curve(start, initScoreStealControl1, initScoreSteal).linear(start, initScoreSteal);
    }


    public Path turnToLoseTime() {
        return Paths.line(turnToLoseTimeStart, turnToLoseTime).linear(turnToLoseTimeStart, turnToLoseTime);
    }


    public Path shootHiveDownShoot() {
        return Paths.line(turnToLoseTime, shootHiveDownShoot).constant(shootHiveDownShoot);
    }


    public Path toHiveSide() {
        return Paths.line(toHiveSideStart, toHiveSide).linear(toHiveSideStart, toHiveSide);
    }


    public Path toShootInGarden() {
        return Paths.curve(toShootInGardenStart, toShootInGardenControl1, toShootInGarden).linear(toShootInGardenStart, toShootInGarden);
    }


    public Path takeGarden() {
        return Paths.line(toShootInGarden, takeGarden).constant(takeGarden);
    }


    public Path toPark() {
        return Paths.curve(toParkStart, toParkControl1, toPark).linear(toParkStart, toPark);
    }
}