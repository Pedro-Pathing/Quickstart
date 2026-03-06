package org.firstinspires.ftc.teamcode.common.command;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

public class Shoot implements Command{
    private int stage = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    @Override
    public void update(Robot robot) {
        if (stage == 0 && ((robot.turret.atTarget() && robot.shooter.atTargetVelocity() && !robot.follower.isBusy()) || elapsedTime.milliseconds()>2500)){

            robot.intake.turnOnIntake();
            robot.lifter.liftDown();
            robot.transfer.turnOnTransfer();
            robot.pins.setPinOpen(1);
            robot.follower.holdPoint(Robot.robotPos);



            elapsedTime.reset();
            stage++;
        } else if (stage == 1 && elapsedTime.milliseconds() > 0) {
            robot.pins.setPinOpen(2);
            elapsedTime.reset();
            stage++;
        } else if (stage == 2 && elapsedTime.milliseconds() > 250) {
            robot.pins.setPinOpen(0);
            elapsedTime.reset();
            stage++;
        } else if (stage == 3 && elapsedTime.milliseconds() > 725) {
            Robot.threeBalls = false;
            robot.lifter.liftUp();
            elapsedTime.reset();
            stage++;
        } else if (stage == 4 && (elapsedTime.milliseconds() > 75 || robot.follower.isTeleopDrive())) {
            if (robot.follower.isTeleopDrive()){
                robot.follower.startTeleOpDrive(false);
            }

            stage++;
        }

    }

    @Override
    public boolean isDone() {
        return stage==5;
    }
}

