package org.firstinspires.ftc.teamcode.commands.drivecommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class PathChainCommand extends CommandBase {

    private final PathChain pathChain;

    private final Robot robot = Robot.getInstance();

    private final double speed;

    public PathChainCommand(Path... paths) {
        this.pathChain = new PathChain(paths);
        this.speed = 1;
    }

    public PathChainCommand(double speed, Path... paths) {
        this.pathChain = new PathChain(paths);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        robot.follower.setMaxPower(speed);
        robot.follower.followPath(pathChain, false);
    }


    @Override
    public boolean isFinished() {
        return !robot.follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        robot.follower.setMaxPower(1);
    }
}
