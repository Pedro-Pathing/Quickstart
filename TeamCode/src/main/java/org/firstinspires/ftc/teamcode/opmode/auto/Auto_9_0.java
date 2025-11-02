package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildPath;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildCurve;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactInCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactShootCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Globals;

@Autonomous(name = "Auto_9_0")
@Configurable
public class Auto_9_0 extends LinearOpMode {



    public static double startX = 20.5;
    public static double startY = 123;
    public static double startHeading = 144;


    public static double shoot0X = 45;
    public static double shoot0Y = 99;
    public static double shoot0Heading = 145;


    public static double intake1X = 25;
    public static double intake1Y = 84;
    public static double intake1Heading = 180;


    public static double shoot1X = 45;
    public static double shoot1Y = 99;
    public static double shoot1Heading = 145;


    public static double intake2X = 30;
    public static double intake2Y = 60;
    public static double intake2Heading = 180;


    public static double shoot2X = 45;
    public static double shoot2Y = 99;
    public static double shoot2Heading = 145;


    public static double intake3X = 30;
    public static double intake3Y = 36;
    public static double intake3Heading = 180;

    public static double shoot3X = 45;
    public static double shoot3Y = 99;
    public static double shoot3Heading = 145;


    // control points for intaking
    public static double control1X = 80;
    public static double control1Y = 84;

    public static double control2X = 85;
    public static double control2Y = 57;

    public static double control3X = 77;
    public static double control3Y = 32;


    public static Path shoot0Path;
    public static Path intake1Path;
    public static Path shoot1Path;
    public static Path intake2Path;
    public static Path shoot2Path;
    public static Path intake3Path;
    public static Path shoot3Path;


    public Pose startPose = new Pose(startX, startY, Math.toRadians(startHeading));

    public void buildPaths() {
        Pose shoot0Pose = new Pose(shoot0X, shoot0Y, Math.toRadians(shoot0Heading));
        Pose intake1Pose = new Pose(intake1X, intake1Y, Math.toRadians(intake1Heading));
        Pose shoot1Pose = new Pose(shoot1X, shoot1Y, Math.toRadians(shoot1Heading));
        Pose intake2Pose = new Pose(intake2X, intake2Y, Math.toRadians(intake2Heading));
        Pose shoot2Pose = new Pose(shoot2X, shoot2Y, Math.toRadians(shoot2Heading));
        Pose intake3Pose = new Pose(intake3X, intake3Y, Math.toRadians(intake3Heading));
        Pose shoot3Pose = new Pose(shoot3X, shoot3Y, Math.toRadians(shoot3Heading));

        Pose intakeControl1 = new Pose(control1X, control1Y);
        Pose intakeControl2 = new Pose(control2X, control2Y);
        Pose intakeControl3 = new Pose(control3X, control3Y);

        shoot0Path = buildPath(startPose, shoot0Pose);
        intake1Path = buildCurve(shoot0Pose, intake1Pose, intakeControl1);
        shoot1Path = buildPath(intake1Pose, shoot1Pose);
        intake2Path = buildCurve(shoot1Pose, intake2Pose, intakeControl2);
        shoot2Path = buildPath(intake2Pose, shoot2Pose);
        intake3Path = buildCurve(shoot2Pose, intake3Pose, intakeControl3);
        shoot3Path = buildPath(intake3Pose, shoot3Pose);


    }

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();

        Globals.IS_AUTO = true;

        robot.initialize(hardwareMap, telemetry);
        CommandScheduler.getInstance().reset();

        buildPaths();

        while (!isStarted() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.updateData();
            robot.periodic();
            robot.write();
        }

        robot.follower.setPose(startPose);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new PathCommand(shoot0Path).alongWith(
                                new ArtifactInCommand()
                        ),

                        new WaitCommand(500), // to let the launcher charge up

                        new ArtifactShootCommand(),
                        new WaitCommand(750),
                        new ArtifactShootCommand(),

                        new PathCommand(intake1Path),

                        new PathCommand(shoot1Path),

                        new ArtifactShootCommand(),
                        new WaitCommand(750),
                        new ArtifactShootCommand(),

                        new PathCommand(intake2Path),

                        new PathCommand(shoot2Path),

                        new ArtifactShootCommand(),
                        new WaitCommand(750),
                        new ArtifactShootCommand(),


                        new PathCommand(intake3Path),

                        new PathCommand(shoot3Path),

                        new ArtifactShootCommand(),
                        new WaitCommand(750),
                        new ArtifactShootCommand()

                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.updateData();
            robot.periodic();
            robot.write();
        }
    }
}
