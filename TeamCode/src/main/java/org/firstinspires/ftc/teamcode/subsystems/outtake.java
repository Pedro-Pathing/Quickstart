package org.firstinspires.ftc.teamcode.subsystems;



import org.firstinspires.ftc.teamcode.util.Hardware;

public class outtake {
    private Hardware robot;
    private long spinStartTime = 0;
    private long spinUpDuration = 1000;
    private boolean running = false;

    public outtake(Hardware robot) {
        this.robot = robot;
    }
    public void outtakeCharge() {
        if (running) return;
        running = true;
        spinStartTime = System.currentTimeMillis();
        robot.outtake.setPower(0.7);
    }

    public boolean isAtMaxRPM() {
        return running && (System.currentTimeMillis() - spinStartTime >= spinUpDuration);
    }

    public void stopMotor() {
        robot.outtake.setPower(0);
        running = false;
    }
}
