package org.firstinspires.ftc.teamcode.testing_TeaganandJessica;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {
    HardwareMapTest robot = new HardwareMapTest();

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.8226, 22.1944, 90);
    private final Pose point1 = poseFactory.of(56.1149, 36.3431, -91.1836);
    private final Pose point2 = poseFactory.of(18.9617, 36.3466, -0.0054);
    private final Pose point3 = poseFactory.of(18.2169, 102.2249, -89.3522);
    private final Pose point4 = poseFactory.of(4.5466, 102.4055, -0.7569);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
            follow(follower, path1()),
            follow(follower, path2()),
            follow(follower, path3()),
            follow(follower, path4())
        );
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());
        CommandBuilder driving = parallel(
                autoRoutine(),
                robot.shooter( 0.5, 2));

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, point1).reverseTangent();
    }

    public Path path2() {
        return Paths.line(point1, point2).reverseTangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return Paths.line(point3, point4).reverseTangent();
    }
}
