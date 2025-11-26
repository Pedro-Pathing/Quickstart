package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class Commands {

    public static Object driveSystem = null;
    public static Object turretSystem = null;
    public static Object intakeSystem = null;

    public static Command runPath(LoadHardwareClass Robot, PathChain path, boolean holdEnd) {
        return new LambdaCommand("runPedroPath(" + path + ", " + holdEnd + ")")
                .setInterruptible(false)
                .setStart(() -> Robot.drivetrain.runPath(path, holdEnd))
                .setUpdate(() -> Robot.drivetrain.runPath(path, holdEnd))
                .setIsDone(Robot.drivetrain::pathComplete);
    }

    public static Command setFlywheelState(LoadHardwareClass Robot, Turret.flywheelstate state) {
        return new LambdaCommand("shoot()")
                .setInterruptible(false)
                .setUpdate(() -> Robot.turret.setFlywheel(state))
                .setIsDone(() -> Robot.turret.getFlywheelRPM() > 5900)
                .requires(turretSystem);
    }

    public static Command setBeltState(LoadHardwareClass Robot, Intake.Mode state) {
        return new LambdaCommand("shoot()")
                .setInterruptible(false)
                .setUpdate(() -> Robot.intake.setMode(state))
                .requires(intakeSystem);
    }
}
