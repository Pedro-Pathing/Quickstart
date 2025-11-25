package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.robot.Robot;

import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class Commands {

    private LoadHardwareClass Robot = null;
    public static Command runPath = null;
    public static Object driveSystem = null;

    public static PathChain[] scheduledPaths;

    public Commands(LoadHardwareClass robot){
        Robot = robot;
    }

    public static Command runPath(LoadHardwareClass Robot, PathChain path, boolean holdEnd) {
        return new LambdaCommand("runPedroPath(" + path + ", " + holdEnd)
                .setInterruptible(false)
                .setStart(() -> Robot.drivetrain.runPath(path, holdEnd))
                .setUpdate(() -> Robot.drivetrain.runPath(path, holdEnd))
                .setIsDone(Robot.drivetrain::pathComplete)
                .requires(driveSystem);
    }
}
