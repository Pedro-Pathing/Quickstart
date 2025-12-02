package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;

public class Commands {

    public static Object driveSystem = null;
    public static Object turretSystem = null;
    public static Object intakeSystem = null;

    public static Command runPath(PathChain path, boolean holdEnd) {
        return new FollowPath(path, holdEnd).requires(driveSystem);
    }
    public static Command runPath(PathChain path, boolean holdEnd, double maxPower) {
        return new FollowPath(path, holdEnd, maxPower).requires(driveSystem);
    }

    public static Command setFlywheelState(LoadHardwareClass Robot, Turret.flywheelstate state) {
        return new LambdaCommand("setFlywheelState()")
                .setInterruptible(false)
                .setStart(() -> Robot.turret.setFlywheel(state))
                .setIsDone(() -> {
                    if (state == Turret.flywheelstate.ON){
                        return Robot.turret.getFlywheelRPM() > 5900;
                    }else{
                        return Robot.turret.getFlywheelRPM() < 100;
                    }
                })
                .requires(turretSystem);
    }

    public static Command setIntakeMode(LoadHardwareClass Robot, Intake.Mode state) {
        return new InstantCommand(new LambdaCommand("setIntakeMode()")
                .setStart(() -> Robot.intake.setMode(state))
                .requires(intakeSystem));
    }

    public static Command setTransferState(LoadHardwareClass Robot, Intake.transferState state) {
        return new InstantCommand(new LambdaCommand("setIntakeMode()")
                .setStart(() -> Robot.intake.setTransfer(state))
                .requires(intakeSystem));
    }
}
