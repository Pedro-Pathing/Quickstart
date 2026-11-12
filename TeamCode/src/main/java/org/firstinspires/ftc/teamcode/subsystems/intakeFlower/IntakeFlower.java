package org.firstinspires.ftc.teamcode.subsystems.intakeFlower;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.TauraServo;

public class IntakeFlower extends SubsystemBase {
    public enum State {
        UP, DOWN
    }

    TauraServo IntakeFlower1;
    private State state = State.UP;

    public IntakeFlower(HardwareMap hardwareMap){
        IntakeFlower1 = new TauraServo(hardwareMap.get(Servo.class, IntakeFlowerConstants.HMIntakeFlower1));
    }

    public void setState(State state){
        this.state = state;
        apply();
    }

    public State getState(){
        return state;
    }

    private void apply(){
        if (state == State.DOWN) {
            IntakeFlower1.setPosition(IntakeFlowerConstants.IntakeFlowerOn1);
        } else {
            IntakeFlower1.setPosition(IntakeFlowerConstants.IntakeFlowerOff);
        }
    }

    @Override
    public void periodic(){
        apply();
    }
}
