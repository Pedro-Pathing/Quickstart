package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_Classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;

public class DcMotorExClass {
    // Turret Constants
    // PID coefficients
    PIDCoefficients turretCoefficients = new PIDCoefficients(0.005, 0, 0);
    // Encoder ticks/rotation
    // 1620rpm - 103.8 ticks at the motor shaft
    double ticksPerRotation = 103.8;

    private DcMotorEx motorObject = null;

    public DcMotorExClass(DcMotorEx motor){
        motorObject = motor;
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
        return (getVelocity()/ticksPerRotation*360);
    }

    /**
     * @return The velocity of the turret in RPM.
     */
    public double getRPM(){
        return ((getVelocity()*60)/ticksPerRotation);
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
     * @param angle The angle to move the motor to.
     */
    public void setAngle(double angle){
        ControlSystem turretPID = ControlSystem.builder().posPid(turretCoefficients).build();
        KineticState currentKineticState = new KineticState(getAngleAbsolute(), getVelocity());
        turretPID.setGoal(new KineticState(angle));
        setPower(turretPID.calculate(currentKineticState));
    }

    /**
     * Uses a PID controller to accelerate the motor to the desired RPM
     * Must be called every loop to function properly.
     * @param rpm The RPM to accelerate the motor to
     */
    public void setTurretAngle(double rpm){
        ControlSystem PID = ControlSystem.builder().velPid(turretCoefficients).build();
        KineticState currentKineticState = new KineticState(getAngleAbsolute(), getVelocity());
        PID.setGoal(new KineticState(0, rpm));
        setPower(PID.calculate(currentKineticState));
    }
}
