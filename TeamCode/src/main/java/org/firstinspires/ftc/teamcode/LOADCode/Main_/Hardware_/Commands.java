package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;

public class Commands {

    public static Object driveSystem = null;
    public static Object turretSystem = null;
    public static Object intakeSystem = null;

    public static Command runPath(PathChain path, boolean holdEnd) {
        return new FollowPath(path, holdEnd);
    }
    public static Command runPath(PathChain path, boolean holdEnd, double maxPower) {
        return new FollowPath(path, holdEnd, maxPower);
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
