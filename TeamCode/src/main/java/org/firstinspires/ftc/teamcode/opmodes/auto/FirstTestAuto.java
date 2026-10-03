package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.control.AutoCommandsKt.*;

import com.pedropathing.config.Modifier;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;
import com.pedropathing.paths.curves.bezier.BezierCurve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Collections;
import java.util.List;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;


@Disabled
@NextAutonomous
public class FirstTestAuto extends NextOpMode {
    public final Robot robot;

    public final Pose startPose = new Pose(0.0, 0.0, 0.0);

    public final Pose intakePose = new Pose(0.0,0.0, 0.0);

    public final Pose shoot1 = new Pose(0.0,0.0, 0.0);

    private Path startToIntake = line(startPose, intakePose)
                    .heading(Interpolator.linear(startPose, intakePose));

    private Path intakeToShoot1 = line(intakePose, shoot1).linear(90.0, 0.0);

    public FirstTestAuto(Robot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void start() {
        super.start();
        //IntakeShoot(robot, startToIntake);
    }
}
