package org.firstinspires.ftc.teamcode.common.command;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

public class IntakeBall implements Command{
    private int stage = 0;

    ElapsedTime elapsedTime = new ElapsedTime();
    boolean allPinsDown;
    public IntakeBall(Boolean allPinsDown){
        this.allPinsDown = allPinsDown;
    }
    @Override
    public void update(Robot robot) {
        if (stage == 0 ){

            robot.transfer.turnOnTransfer();
            robot.intake.turnOnIntake();
            robot.lifter.liftBlock();

            stage++;
        } else if (stage == 1) {
            if (allPinsDown){
                robot.pins.closeAllPin();
                stage++;
            } else {
                if (robot.follower.isTeleopDrive()){
                    robot.pins.openAllPin();
                }else {
                    robot.pins.closeMostPins();
                }
                stage = 4;
            }
        } else if (stage == 2 && robot.ballsInIntake()) {
            Robot.threeBalls = true;
            if (robot.follower.isTeleopDrive()){
                robot.follower.startTeleOpDrive(true);
            }
            robot.pins.setPinOpen(2);
            elapsedTime.reset();
            stage++;
        } else if (stage == 3 && elapsedTime.milliseconds()>750) {

            robot.pins.setPinOpen(1);
            robot.pins.closeMostPins();
            stage++;
        }

    }

    @Override
    public boolean isDone() {
        return stage==4;
    }
}

