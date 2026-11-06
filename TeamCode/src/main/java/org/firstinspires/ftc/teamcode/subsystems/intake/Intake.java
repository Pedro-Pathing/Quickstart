package org.firstinspires.ftc.teamcode.subsystems.intake;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Intake extends SubsystemBase {

    DcMotor intake1;
    DcMotor index1;

    public Intake(HardwareMap hardwareMap){
        intake1 = hardwareMap.get(DcMotor.class, IntakeConstants.HMIntake1);
        index1 = hardwareMap.get(DcMotor.class, IntakeConstants.HMIndex1);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        index1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intakeOn(){
        intake1.setPower(IntakeConstants.intakeOn);
        index1.setPower(IntakeConstants.indexOn);
    }

     public void intakeOff(){
        intake1.setPower(IntakeConstants.intakeOff);
        index1.setPower(IntakeConstants.indexOff);
     }

     public void intakeReverse(){
        intake1.setPower(IntakeConstants.intakeReverse);
        index1.setPower(IntakeConstants.indexOn);
     }

    public void intakeToShooter(){
        intake1.setPower(IntakeConstants.intakeOn);
        index1.setPower(IntakeConstants.indexReverse);
    }
     @Override
    public void periodic(){}
}
