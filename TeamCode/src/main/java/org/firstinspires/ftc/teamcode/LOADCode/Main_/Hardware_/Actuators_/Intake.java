package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Intake {
    // RESET THESE TO PRIVATE AFTER DECEMBER 6TH!
    private final Devices.DcMotorExClass intake = new Devices.DcMotorExClass();
    private final Devices.CRServoClass belt = new Devices.CRServoClass();
    private final Devices.ServoClass transfer = new Devices.ServoClass();
    public final Devices.DualProximitySensorClass topSensor = new Devices.DualProximitySensorClass();
    public final Devices.DualProximitySensorClass bottomSensor = new Devices.DualProximitySensorClass();

    public enum intakeMode {
        INTAKING,
        SHOOTING,
        NO_BELT,
        REVERSING,
        OFF
    }

    public enum transferState {
        UP,
        DOWN
    }

    public static double proximitySensorThreshold = 20;

    public void init(OpMode opmode){
        intake.init(opmode, "intake");
        belt.init(opmode, "belt");
        transfer.init(opmode, "transfer");
        topSensor.init(opmode, "color1", "color2");
        bottomSensor.init(opmode, "color3", "color4");


        intake.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        belt.setDirection(DcMotorSimple.Direction.REVERSE);

        topSensor.setGain(2);
        topSensor.units = DistanceUnit.MM;
        topSensor.threshold = proximitySensorThreshold;
        bottomSensor.setGain(2);
        bottomSensor.units = DistanceUnit.MM;
        bottomSensor.threshold = proximitySensorThreshold;
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
            //if (!(getTopSensorState() && getBottomSensorState())){
                belt.setPower(1);
            //}
            intake.setPower(1);
        }else if (direction == intakeMode.SHOOTING){
            intake.setPower(0);
            belt.setPower(1);
        }else if (direction == intakeMode.REVERSING){
            intake.setPower(-1);
            belt.setPower(-1);
        }else if (direction == intakeMode.NO_BELT){
            intake.setPower(1);
            belt.setPower(0);
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

    public boolean getTopSensorState(){
        topSensor.threshold = proximitySensorThreshold;
        return topSensor.objectDetected();
    }
    public boolean getBottomSensorState(){
        bottomSensor.threshold = proximitySensorThreshold;
        return bottomSensor.objectDetected();
    }
}
