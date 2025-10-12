package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class ShooterSubsystem extends RE_SubsystemBase {

    private final DcMotorEx shootMotor;

    private final DcMotorEx stopMotor;

    public ShootState shootState;

    public StopState stopState;

    public enum ShootState {
        SHOOT,
        STOP
    }

    public enum StopState {
        READY,
        STOP
    }

    public ShooterSubsystem(HardwareMap hardwareMap, String shootroller, String stoproller) {
        this.shootMotor = hardwareMap.get(DcMotorEx.class, shootroller);
        this.shootMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.stopMotor = hardwareMap.get(DcMotorEx.class, stoproller);
        this.stopMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);

        shootState = ShootState.STOP;

        Robot.getInstance().subsystems.add(this);
    }

    @Override
    public void updateData() {
        Robot.getInstance().data.shootState = shootState;

        Robot.getInstance().data.stopState = stopState;
    }



    public void updateShootState(ShootState state) {
        this.shootState = state;
    }

    public void updateStopState(StopState state) {
        this.stopState = state;
    }

    @Override
    public void periodic() {
        switch (shootState) {
            case SHOOT:
                shootMotor.setPower(Constants.shootPower);
                break;
            case STOP:
                shootMotor.setPower(0);
                break;
        }

        switch(stopState) {
            case READY:
                stopMotor.setPower(Constants.readyPower);
                break;
            case STOP:
                stopMotor.setPower(0);
                break;
        }
    }
}
