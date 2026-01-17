package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_;

import androidx.annotation.NonNull;

import com.pedropathing.paths.PathChain;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;

public class Commands {

    // Robot Object for command access
    public LoadHardwareClass Robot;
    public Commands(@NonNull LoadHardwareClass robot){
        Robot = robot;
    }

    // Delay timer for shooting sequence
    private static final TimerEx shootingTimer = new TimerEx(1);
    private static Command resetShootingTimer() {
        return new LambdaCommand("resetShootingTimer").setStart(shootingTimer::restart);
    }

    public Command runPath(PathChain path, boolean holdEnd, double maxPower) {
        return new FollowPath(path, holdEnd, maxPower);
    }

    public Command setFlywheelState(Turret.flywheelState state) {
        return new LambdaCommand("setFlywheelState()")
                .setInterruptible(false)
                .setStart(() -> Robot.turret.setFlywheelState(state))
                .setIsDone(() -> {
                    if (state == Turret.flywheelState.ON){
                        return Robot.turret.getFlywheelRPM() > Robot.turret.getFlywheelCurrentMaxSpeed() - 100;
                    }else{
                        return true;
                    }
                })
        ;
    }

    private Command setGateState(Turret.gatestate state){
        return new InstantCommand(new LambdaCommand("setGateState")
                .setStart(() -> Robot.turret.setGateState(state))
        );
    }

    public Command setIntakeMode(Intake.intakeMode state) {
        return new InstantCommand(new LambdaCommand("setIntakeMode()")
                .setStart(() -> Robot.intake.setMode(state))
                .setIsDone(() -> true)
        );
    }

    public Command setTransferState(Intake.transferState state) {
        return new InstantCommand(new LambdaCommand("setIntakeMode()")
                .setStart(() -> Robot.intake.setTransfer(state))
                .setIsDone(() -> true)
        );
    }

    public Command shootBalls(){
        return new SequentialGroup(
                // Ensure the flywheel is up to speed, if not, spin up first
                setFlywheelState(Turret.flywheelState.ON),

                // Shoot the first two balls
                setIntakeMode(Intake.intakeMode.INTAKING),
                setGateState(Turret.gatestate.OPEN),
                resetShootingTimer(),
                new WaitUntil(() -> Robot.intake.getTopSensorState() && !Robot.intake.getBottomSensorState() && shootingTimer.isDone()),

                // Shoot the last ball
                setIntakeMode(Intake.intakeMode.SHOOTING),
                setTransferState(Intake.transferState.UP),
                resetShootingTimer(),
                new WaitUntil(shootingTimer::isDone),

                // Reset the systems
                setIntakeMode(Intake.intakeMode.OFF),
                setGateState(Turret.gatestate.CLOSED),
                setTransferState(Intake.transferState.DOWN),
                setFlywheelState(Turret.flywheelState.OFF)
        );
    }

}
