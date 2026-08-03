package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.theLab.SimplePIDController;

public class slideConstants {
    private DcMotor containerMotor;
    private final SimplePIDController pid = new SimplePIDController(0.002, 0, 0.0001);
    private final ElapsedTime timer = new ElapsedTime();
    private int targetPosition = 0;

    static final double TICKS_PER_REV = 537.6;
    static final double ARM_TICKS_PER_REV = 1425.1;
    public static final int POSITION_RETRACTED = 0;
    public static final int POSITION_MIDDLE = -(int)(TICKS_PER_REV * 1.1);
    public static final int POSITION_MAX = -(int)(TICKS_PER_REV * 2.3);
    
    // --- Gravity Feedforward Constants ---
    public static final double kG = -0.1; // Holding power to counteract gravity (negative since extension is negative)
    public static final double SLIDE_POWER_MAX = 0.35 ;

    public slideConstants(HardwareMap hardwareMap) {
        containerMotor = hardwareMap.get(DcMotor.class, "containerMotor");
        containerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        containerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        containerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // We use RUN_WITHOUT_ENCODER to apply manual PID + Gravity Feedforward
        containerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        containerMotor.setPower(0.0);
        timer.reset();
    }

    /** Updates the motor power using PID + Gravity Feedforward. Should be called every loop. */
    public void update() {
        double dt = timer.seconds();
        timer.reset();

        int currentPosition = containerMotor.getCurrentPosition();
        double error = targetPosition - currentPosition;
        
        double pidOutput = pid.calculate(error, dt);
        
        // Only apply gravity feedforward if the slide is not at the very bottom to prevent motor humming
        double feedforward = (currentPosition < -50) ? kG : 0;

        containerMotor.setPower(Range.clip(pidOutput + feedforward, -SLIDE_POWER_MAX, SLIDE_POWER_MAX));
    }

    public void start() {
        setTarget(POSITION_RETRACTED);
    }

    public void setTarget(int ticks) {
        if (ticks > POSITION_RETRACTED) ticks = POSITION_RETRACTED;
        if (ticks < POSITION_MAX) ticks = POSITION_MAX;

        this.targetPosition = ticks;
        // containerMotor.setTargetPosition(ticks); 
        // containerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // containerMotor.setPower(SLIDE_POWER);
    }

    public void extendToHigh() { setTarget(POSITION_MAX); }
    public void extendToMiddle() { setTarget(POSITION_MIDDLE); }
    public void extendToBottom() { setTarget(POSITION_RETRACTED); }

    public int getCurrentPosition() {
        return containerMotor.getCurrentPosition();
    }

    public void resetEncoder() {
        containerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        containerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.targetPosition = POSITION_RETRACTED;
    }
}
