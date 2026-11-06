package org.firstinspires.ftc.teamcode.subsystems.intakeFlower;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeFlower extends SubsystemBase {
    Servo IntakeFlower1;
    Servo IntakeFlower2;
     public void intakeFlower(HardwareMap hardwareMap){
         IntakeFlower1 = hardwareMap.get(Servo.class, IntakeFlowerConstants.HMIntakeFlower1);
         IntakeFlower2 = hardwareMap.get(Servo.class, IntakeFlowerConstants.HMIntakeFlower2);
     }
     public void intakeFlowerOn(){
         IntakeFlower1.setPosition(IntakeFlowerConstants.IntakeFlowerOn1);
         IntakeFlower2.setPosition(IntakeFlowerConstants.IntakeFlowerOn2);

     }
     public void intakeFlowerOff(){
         IntakeFlower1.setPosition(IntakeFlowerConstants.IntakeFlowerOff);
         IntakeFlower2.setPosition(IntakeFlowerConstants.IntakeFlowerOff);
     }
     @Override
    public void periodic(){

     }
}
