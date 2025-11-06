package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_Classes;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;

public class DcMotorExClass {
    // PID pidCoefficients
    PIDCoefficients pidCoefficients = new PIDCoefficients(0.005, 0, 0);
    // Encoder ticks/rotation
    // 1620rpm Gobilda - 103.8 ticks at the motor shaft
    double ticksPerRotation = 103.8;
    // Motor object
    private DcMotorEx motorObject = null;

    /**
     * Initializes the motor hardware
     * @param opmode Allows this class to access the robot hardware objects.
     * @param motorName The name of the motor in the robot's configuration.
     * @param encoderResolution The resolution of the motor's encoder in ticks/rotation.
     */
    public void init (@NonNull OpMode opmode, String motorName, double encoderResolution){
        // Initialize the motor object
        motorObject  = opmode.hardwareMap.get(DcMotorEx.class, motorName);
        ticksPerRotation = encoderResolution;
    }
    /**
     * Initializes the motor hardware.
     * The resolution of the motor encoder will default to
     *      103.8 ticks/rotation, the value for a 1620RPM Gobilda motor.
     * @param opmode Allows this class to access the robot hardware objects.
     * @param motorName The name of the motor in the robot's configuration.
     */
    public void init (@NonNull OpMode opmode, String motorName){
        // Initialize the motor object
        motorObject  = opmode.hardwareMap.get(DcMotorEx.class, motorName);
    }

    /**
     * Sets the value of the PID coefficients of the motor.
     * @param coefficients The values to set the coefficients to.
     */
    public void setPidCoefficients(PIDCoefficients coefficients) {
        pidCoefficients = coefficients;
    }
    /**
     * @return The current position of the turret motor in encoder ticks. Can be any value.
     */
    public double getEncoderTicks(){
        return motorObject.getCurrentPosition();
    }
    /**
     * @return The resolution of the turret's encoder in ticks/rotation.
     */
    public double getEncoderResolution(){
        return ticksPerRotation;
    }
    /**
     * @param power A value between -1 and 1 that the turret motor's power will be set to.
     */
    public void setPower(double power){
        motorObject.setPower(power);
    }
    /**
     * @return The angle of the turret in degrees. Can be any value.
     */
    public double getAngleAbsolute(){
        return (getEncoderTicks()/ticksPerRotation*360);
    }
    /**
     * @return The angle of the turret in degrees. Can be any value between 0 and 360.
     */
    public double getAngle(){
        return getAngleAbsolute()%360;
    }
    /**
     * @return The velocity of the turret in encoder ticks/second.
     */
    public double getVelocity(){
        return motorObject.getVelocity();
    }
    /**
     * @return The velocity of the turret in degrees/second.
     */
    public double getDegreesPerSecond(){
        return (getVelocity()/ticksPerRotation)*360;
    }
    /**
     * @return The velocity of the turret in RPM.
     */
    public double getRPM(){
        return (getVelocity()/ticksPerRotation)*60;
    }
    /**
     * @return The power that the turret motor has been set to.
     */
    public double getPower(){
        return motorObject.getPower();
    }
    /**
     * Uses a PID controller to move the motor to the desired position.
     * Must be called every loop to function properly.
     * @param angle The angle in degrees to move the motor to. Can be any number.
     */
    public void setAngle(double angle){
        ControlSystem turretPID = ControlSystem.builder().posPid(pidCoefficients).build();
        KineticState currentKineticState = new KineticState(getAngleAbsolute(), getDegreesPerSecond());
        turretPID.setGoal(new KineticState(angle));
        setPower(turretPID.calculate(currentKineticState));
    }
    /**
     * Uses a PID controller to accelerate the motor to the desired RPM
     * Must be called every loop to function properly.
     * @param rpm The RPM to accelerate the motor to. Can be any number
     */
    public void setRPM(double rpm){
        double degreesPerSecond = rpm*6;
        ControlSystem PID = ControlSystem.builder().velPid(pidCoefficients).build();
        KineticState currentKineticState = new KineticState(getAngleAbsolute(), getDegreesPerSecond());
        PID.setGoal(new KineticState(0, rpm));
        setPower(PID.calculate(currentKineticState));
    }
}