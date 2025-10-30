package org.firstinspires.ftc.teamcode.commands.drivecommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.*;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods;

import java.util.function.DoubleSupplier;

public class StrafeCommand extends CommandBase {

    private final double inches;

    private final Robot robot = Robot.getInstance();

    private final double speed;

    public StrafeCommand(DoubleSupplier inches) {
        this.inches = inches.getAsDouble();
        this.speed = 1;
    }

    public StrafeCommand(double inches, double speed) {
        this.inches = inches;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        Pose current = robot.follower.getPose();

        Pose target = new Pose(
                current.getX() + inches,
                current.getY(),
                current.getHeading()
        );

        PathChain path = new PathChain(AutonomousMethods.buildPath(current, target));

        robot.follower.setMaxPower(speed);
        robot.follower.followPath(path, true);
    }


    @Override
    public void execute() {
        robot.follower.update();
    }

    @Override
    public boolean isFinished() {
//        double xError = Math.abs(robot.getPose().getX() - path.getPath(0).getLastControlPoint().getX());
//        double yError = Math.abs(robot.getPose().getY() - path.getPath(0).getLastControlPoint().getY());
//        double headingError = Math.abs(robot.getPose().getHeading() - path.getPath(0).getHeadingGoal(1));

//        return xError < Constants.pathEndXTolerance && yError < Constants.pathEndYTolerance && headingError < Constants.pathEndHeadingTolerance;
        return !robot.follower.isBusy();
    }

}
