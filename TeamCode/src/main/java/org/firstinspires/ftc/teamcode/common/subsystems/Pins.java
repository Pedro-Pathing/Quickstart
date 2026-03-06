package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pins {
    private final Servo leftPin;
    private final Servo centerPin;
    private final Servo rightPin;

    private final static double leftPinClose = 0.575;
    private final static double centerPinClose = 0.105;
    private final static double rightPinClose = 0.275;

    private final static double leftPinOpen= 0.7;
    private final static double centerPinOpen = 0.4;
    private final static double rightPinOpen = 0.15;

    Servo[] pins;

    Double[] open = {leftPinOpen, centerPinOpen, rightPinOpen};

    public Pins(HardwareMap hardwareMap){
        leftPin = hardwareMap.get(Servo.class,"servo_lp");
        centerPin = hardwareMap.get(Servo.class,"servo_cp");
        rightPin = hardwareMap.get(Servo.class,"servo_rp");
        pins = new Servo[]{leftPin, centerPin, rightPin};
    }

    public void closeAllPin(){
        leftPin.setPosition(leftPinClose);
        rightPin.setPosition(rightPinClose);
        centerPin.setPosition(centerPinClose);
    }

    public void openAllPin(){
        leftPin.setPosition(leftPinOpen);
        rightPin.setPosition(rightPinOpen);
        centerPin.setPosition(0.3375);
    }

    public void closeMostPins(){
        rightPin.setPosition(rightPinClose);
        centerPin.setPosition(centerPinOpen);
        leftPin.setPosition(leftPinClose);
    }
    public void preload(){
        rightPin.setPosition(rightPinOpen);
        centerPin.setPosition(centerPinOpen);
        leftPin.setPosition(leftPinClose);
    }

    public void setPinOpen(int index){
        pins[index].setPosition(open[index]);
    }


}
