package org.firstinspires.ftc.teamcode.common.command;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

public class IntakeTransfer implements Command{
    private int stage = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    @Override
    public void update(Robot robot) {
        if (stage == 0){
            Robot.threeBalls = true;
            robot.pins.setPinOpen(2);
            elapsedTime.reset();
            stage++;
        } else if (stage == 1 && elapsedTime.milliseconds()>750) {

            robot.pins.setPinOpen(1);
            robot.pins.closeMostPins();
            stage++;
        }
    }

    @Override
    public boolean isDone() {
        return stage == 2;
    }
}
