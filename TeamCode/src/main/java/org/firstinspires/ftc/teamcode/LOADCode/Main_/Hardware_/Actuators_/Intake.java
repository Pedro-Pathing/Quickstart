package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Intake {
    // RESET THESE TO PRIVATE AFTER DECEMBER 6TH!
    public final Devices.DcMotorExClass intake = new Devices.DcMotorExClass();
    public final Devices.CRServoClass belt = new Devices.CRServoClass();
    public final Devices.ServoClass transfer = new Devices.ServoClass();

    public enum intakeMode {
        INTAKING,
        SHOOTING,
        REVERSING,
        OFF
    }

    public enum transferState {
        UP,
        DOWN
    }

    public void init(OpMode opmode){
        intake.init(opmode, "intake");
        belt.init(opmode, "belt");
        transfer.init(opmode, "transfer");

        intake.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        belt.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * @param direction
     * Takes the following inputs
     * <ul>
     *     <li><code>intakeMode.INTAKING</code></li>
     *     <li><code>intakeMode.SHOOTING</code></li>
     *     <li><code>intakeMode.REVERSING</code></li>
     *     <li><code>intakeMode.OFF</code></li>
     * </ul>
     */
    public void setMode(intakeMode direction){
        if (direction == intakeMode.INTAKING){
            intake.setPower(1);
            belt.setPower(1);
        }else if (direction == intakeMode.SHOOTING){
            intake.setPower(0);
            belt.setPower(1);
        }else if (direction == intakeMode.REVERSING){
            intake.setPower(-1);
            belt.setPower(-1);
        }else{
            intake.setPower(0);
            belt.setPower(0);
        }
    }

    /**
     * Outputs one of the following modes
     * <ul>
     *     <li><code>intakeMode.INTAKING</code></li>
     *     <li><code>intakeMode.SHOOTING</code></li>
     *     <li><code>intakeMode.REVERSING</code></li>
     *     <li><code>intakeMode.OFF</code></li>
     * </ul>
     */
    public intakeMode getMode(){
        double intakePower = intake.getPower();
        double beltPower = belt.getPower();
        if (intakePower == 1 && beltPower == 1){
            return intakeMode.INTAKING;
        }else if (intakePower == 0 && beltPower == 1){
            return intakeMode.SHOOTING;
        }else if (intakePower == -1 && beltPower == -1){
            return intakeMode.REVERSING;
        }else{
            return intakeMode.OFF;
        }
    }

    public void setTransfer(transferState state) {
        switch (state){
            case UP:
                transfer.setAngle(0);
                return;
            case DOWN:
                transfer.setAngle(.05);
        }
    }
}
