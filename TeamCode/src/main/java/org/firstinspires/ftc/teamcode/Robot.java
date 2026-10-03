package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class Robot implements NextRobot {
    public Flywheel flywheel;
    public Intake intake;

    protected HardwareMap hardwareMap;
    public Drivetrain drivetrain;

    public Follower follower;

    public Robot() {
        flywheel = new Flywheel();
        intake = new Intake();
        drivetrain = new Drivetrain();
//        follower = Constants.create(hardwareMap);
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(
        flywheel
        , intake
        , drivetrain
        );
    }

    public void destroy() {

    }

}
