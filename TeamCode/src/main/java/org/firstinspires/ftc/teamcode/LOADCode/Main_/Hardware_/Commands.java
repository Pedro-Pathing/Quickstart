package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_;

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

    private static final TimerEx shootingTimer = new TimerEx(1);

    private static Command resetShootingTimer() {
        return new LambdaCommand("resetShootingTimer");
    }

    public static Command runPath(PathChain path, boolean holdEnd) {
        return runPath(path, holdEnd, 1);
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
                        return Robot.turret.getFlywheelRPM() > Turret.flywheelSpeed-100;
                    }else{
                        return Robot.turret.getFlywheelRPM() < 100;
                    }
                })
        ;
    }

    private static Command setGateState(LoadHardwareClass Robot, Turret.gatestate state){
        return new InstantCommand(new LambdaCommand("setGateState")
                .setStart(() -> Robot.turret.setGateState(state))
        );
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

    public static Command shootBalls(LoadHardwareClass Robot){
        return new SequentialGroup(
                // Ensure the flywheel is up to speed, if not, spin up first
                setFlywheelState(Robot, Turret.flywheelstate.ON),
                new WaitUntil(() -> Robot.turret.getFlywheelRPM() > Turret.flywheelSpeed-100),

                // Shoot the first two balls
                setIntakeMode(Robot, Intake.intakeMode.INTAKING),
                setGateState(Robot, Turret.gatestate.OPEN),
                resetShootingTimer(),
                new WaitUntil(() -> Robot.intake.getTopSensorState() && !Robot.intake.getBottomSensorState() && shootingTimer.isDone()),

                // Shoot the last ball
                setIntakeMode(Robot, Intake.intakeMode.SHOOTING),
                setTransferState(Robot, Intake.transferState.UP),
                resetShootingTimer(),
                new WaitUntil(shootingTimer::isDone),

                // Reset the systems
                setIntakeMode(Robot, Intake.intakeMode.OFF),
                setGateState(Robot, Turret.gatestate.CLOSED),
                setTransferState(Robot, Intake.transferState.DOWN)
        );
    }

}
