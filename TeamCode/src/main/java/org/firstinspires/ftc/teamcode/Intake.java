package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    public DcMotorEx intakeMotor;
    public DcMotorEx transferMotor;
    private final double INTAKE_POWER = 0.75;

    private final double SHOOTING_POWER = 0.5;

    private final double TRANSFER_INTAKE_POWER = -0.4;

    private final double TRANFER_SHOOTING_POWER = -0.75;

    private final double TRANSFER_OFF = 0.0;



    public Intake (HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void intakeArtifacts() {
        intakeMotor.setPower(INTAKE_POWER);
        transferMotor.setPower(TRANSFER_INTAKE_POWER);
    }

    public void intakeArtifactsOnlyIntake() {
        intakeMotor.setPower(INTAKE_POWER);
        transferMotor.setPower(TRANSFER_OFF);
    }

    public void shootArtifacts() {
        intakeMotor.setPower(SHOOTING_POWER);
        transferMotor.setPower(TRANFER_SHOOTING_POWER);
    }
    public void intakeStop(){
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
    }

    public void transferOnly(){
        intakeMotor.setPower(0);
        transferMotor.setPower(0.75);
    }


}
