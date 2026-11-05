package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.nectarShooter;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class NectarShooterConstants {
    public String frontMotorName = "FrontNectarShooter";
    public String rearMotorName = "RearNectarShooter";
    public String motorConfigurationType = "clone";
    public int ticksPerRev = 28; // gobilda ticks 6000 rpm
    public double achievableMaxRPMFraction = 1.0;
    public double gearing = 1;
    public DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public DcMotor.RunMode resetMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
    public DcMotorSimple.Direction frontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorSimple.Direction rearMotorDirection = DcMotorSimple.Direction.FORWARD;
    public DcMotor.ZeroPowerBehavior frontMotorZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
    public DcMotor.ZeroPowerBehavior rearMotorZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
    public double startPosition = 0;
    public double targetPosition = 0;
    public double tolerableError = 30; //rpm
}