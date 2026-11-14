package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.states.LaunchStates;

public class Launcher{

    //making servos/motors the servo/motor class
    DcMotorEx flywheel_a;
    DcMotorEx flywheel_b;

    CRServo feeder;
    ElapsedTime feederTimer = new ElapsedTime();
    Servo hood;

    final double FEED_TIME_SECONDS;
    LaunchStates launchState;
    int targetVel; // Target speed for flywheels
    final double VEL_TOLERANCE; // tolerance for flywheel speed in %
    final double FEED_SPEED; // current position of feeder
    final double STOP_SPEED = 0;
    int minVel;
    int shotsQueued; // Number of balls waiting to be shot

    public Launcher(DcMotorEx flywheel_a, DcMotorEx flywheel_b, CRServo feeder, Servo hood){

        // sets motors and servos to ports
        this.flywheel_a = flywheel_a;
        this.flywheel_b = flywheel_b;
        this.feeder = feeder;
        this.hood = hood;
        hood.scaleRange(0,90); //sets range for hood servo

        launchState = LaunchStates.IDLE;

        //sets flywheels to off
        flywheel_a.setVelocity(0);
        flywheel_b.setVelocity(0);

        // sets static values
        VEL_TOLERANCE = 20;
        FEED_SPEED = 0.6;
        FEED_TIME_SECONDS = 0.4;

        //sets variables on lines 18-20
        targetVel = 1300;
        minVel = (int) (targetVel - VEL_TOLERANCE);
    }

    public void spinUp(){
        launchState = LaunchStates.SPINING_UP;
    }

    public void spinDown(){
        launchState = LaunchStates.IDLE;
    }

    public int speedPer(){ //returns current speed vs target speed as a %

        double current = flywheel_b.getVelocity() + flywheel_a.getVelocity();
        return (int) ((current/(targetVel *2))*100);
    }
    public void launch(boolean shotRequested){ //launches scoring element

        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchStates.SPINING_UP;
                }
                break;
            case SPINING_UP:
                flywheel_a.setVelocity(targetVel);
                flywheel_b.setVelocity(targetVel);
                if (flywheel_a.getVelocity() > minVel && flywheel_b.getVelocity() > minVel) {
                    launchState = LaunchStates.SPUN_UP;
                }
                break;
            case SPUN_UP:
                if (flywheel_a.getVelocity() < minVel || flywheel_b.getVelocity() < minVel) {
                    launchState = LaunchStates.SPINING_UP;
                }
                if (shotRequested) {
                    launchState = LaunchStates.LAUNCHING;
                }
                break;
            case LAUNCH:
                feeder.setPower(FEED_SPEED);
                feederTimer.reset();
                launchState = LaunchStates.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchStates.IDLE;
                    feeder.setPower(STOP_SPEED);
                    if (shotsQueued > 0) {
                        launchState = LaunchStates.SPINING_UP;
                        shotsQueued--;
                    }
                }
                break;
        }

    }

    public void queueShot(){ // adds to queue
        shotsQueued++;
    }

    public void setAngle(int angle){ // sets hood angle to angle
        hood.setPosition(angle);
    }

    public void changeAngle(int angle){ // changes hood angle by angle
        hood.setPosition(hood.getPosition()+angle);
    }

    public void setTargetVel(int targetVel){ //sets target velocity
        this.targetVel = targetVel;}
}