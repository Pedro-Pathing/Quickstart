package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.control.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;


@Autonomous
public class FirstTestAuto extends OpMode {
    private Follower follower;
    private Flywheel flywheel;
    private final PoseFactory p = PoseFactory.degrees();

    ///left and right are relative to the driver

    public final Pose shoot_right = p.of(56, 9, 90); //also start
    public final Pose shoot_left = p.of(9,132, 270);
    public final Pose loading_zone = p.of(9, 104, 90);
    public final Pose garden = p.of(9,9, 270);

    private Path rightToLoad = curve(shoot_right, new Pose(33, 28), loading_zone).linear(shoot_right, garden);
    private Path rightToGarden = curve(shoot_right, new Pose(33, 28), garden).linear(shoot_right, garden);
    private Path gardenToRight = curve(garden, new Pose(33, 28), shoot_right).linear(shoot_right, garden);
    private Path leftToGarden = line(shoot_left, garden);
    private Path gardenToLeft = line(garden, shoot_left);

    private Command autoRoutine() {
        return sequential(
                flywheel.shootThree(2400.0),
                follow(follower, rightToLoad)
        );
    }
    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);


        flywheel = new Flywheel(this);
    }

    @Override
    public void start() {
        follower.setPose(shoot_right);
        Scheduler.schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }
}