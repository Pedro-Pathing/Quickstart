package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class IntakeSubsystem extends RE_SubsystemBase {

    private final DcMotorEx intakeMotor;

    public IntakeState intakeState;

    public enum IntakeState {
        IN,
        STOP,
        OUT
    }

    public IntakeSubsystem(HardwareMap hardwareMap, String intakeMotorName) {
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
        this.intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intakeState = IntakeState.STOP;

        Robot.getInstance().subsystems.add(this);
    }

    @Override
    public void updateData() {
        Robot.getInstance().data.intakeState = intakeState;
    }

    public void updateIntakeState(IntakeState state) {
        this.intakeState = state;
    }

    @Override
    public void periodic() {
        switch (intakeState) {
            case IN:
                intakeMotor.setPower(Constants.intakeInPower);
                break;
            case OUT:
                intakeMotor.setPower(Constants.intakeOutPower);
                break;
            case STOP:
                intakeMotor.setPower(0);
                break;
        }
    }
}
