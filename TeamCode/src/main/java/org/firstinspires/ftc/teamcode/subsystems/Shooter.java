package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter extends SubsystemBase {
    private Motor leftShooter;
    private Motor rightShooter;
    private double rpmTarget;
    private Telemetry telemetry;
    /**
     * Creates a new Shooter.
     */
    public Shooter(HardwareMap hardwareMap, String leftMotorName, String rightMotorName, Telemetry telemetry) {
        this.leftShooter = new Motor(hardwareMap, leftMotorName);
        this.rightShooter = new Motor(hardwareMap, rightMotorName);
//        this.leftShooter.setRunMode(Motor.RunMode.VelocityControl);
//        this.rightShooter.setRunMode(Motor.RunMode.VelocityControl);
//        this.leftShooter.setVeloCoefficients(0.05, 0, 0);
//        this.rightShooter.setVeloCoefficients(0.05, 0, 0);
//        this.rpmTarget = 0;
        this.telemetry = telemetry;
    }

    public void setVelocity(double rpm) {
        this.rpmTarget = rpm;
    }

    public double getSetpointVelocity() {
        return this.rpmTarget;
    }

    @Override
    public void periodic() {
        leftShooter.set(-rpmTarget);
        rightShooter.set(-rpmTarget);
        telemetry.addData("ShooterCPR", leftShooter.getCorrectedVelocity());
    }
}