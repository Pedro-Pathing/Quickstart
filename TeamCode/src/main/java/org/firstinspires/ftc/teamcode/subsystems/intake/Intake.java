package org.firstinspires.ftc.teamcode.subsystems.intake;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public enum State {
        OFF, INTAKE, REVERSE, FEED
    }

    DcMotor intake1;
    DcMotor index1;
    private State state = State.OFF;

    public Intake(HardwareMap hardwareMap){
        intake1 = hardwareMap.get(DcMotor.class, IntakeConstants.HMIntake1);
        index1 = hardwareMap.get(DcMotor.class, IntakeConstants.HMIndex1);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        index1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setState(State state){
        this.state = state;
        apply();
    }

    public State getState(){
        return state;
    }

    private void apply(){
        switch (state) {
            case INTAKE:
                intake1.setPower(IntakeConstants.intakeOn);
                index1.setPower(IntakeConstants.indexOn);
                break;
            case REVERSE:
                intake1.setPower(IntakeConstants.intakeReverse);
                index1.setPower(IntakeConstants.indexReverse);
                break;
            case FEED:
                intake1.setPower(IntakeConstants.intakeOn);
                index1.setPower(IntakeConstants.indexOn);
                break;
            default:
                intake1.setPower(IntakeConstants.intakeOff);
                index1.setPower(IntakeConstants.indexOff);
        }
    }

    @Override
    public void periodic(){
        apply();
    }
}
