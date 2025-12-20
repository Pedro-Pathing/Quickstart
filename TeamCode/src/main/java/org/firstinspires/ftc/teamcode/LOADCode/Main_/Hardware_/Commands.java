package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;

public class Commands {

    public static Command runPath(PathChain path, boolean holdEnd) {
        return new FollowPath(path, holdEnd);
    }
    public static Command runPath(PathChain path, boolean holdEnd, double maxPower) {
        return new FollowPath(path, holdEnd, maxPower);
    }

    public static Command setFlywheelState(LoadHardwareClass Robot, Turret.flywheelstate state) {
        return new LambdaCommand("setFlywheelState()")
                .setInterruptible(false)
                .setStart(() -> Robot.turret.setFlywheelState(state))
                .setIsDone(() -> {
                    if (state == Turret.flywheelstate.ON){
                        return Robot.turret.getFlywheelRPM() > Turret.flywheelSpeed;
                    }else{
                        return Robot.turret.getFlywheelRPM() < 100;
                    }
                })
        ;
    }

    public static Command setIntakeMode(LoadHardwareClass Robot, Intake.intakeMode state) {
        return new InstantCommand(new LambdaCommand("setIntakeMode()")
                .setStart(() -> Robot.intake.setMode(state))
                .setIsDone(() -> true)
        );
    }

    public static Command setTransferState(LoadHardwareClass Robot, Intake.transferState state) {
        return new InstantCommand(new LambdaCommand("setIntakeMode()")
                .setStart(() -> Robot.intake.setTransfer(state))
                .setIsDone(() -> true)
        );
    }

}
