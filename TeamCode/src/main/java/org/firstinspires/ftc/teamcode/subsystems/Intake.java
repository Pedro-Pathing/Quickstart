package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {
    private CRServo lpull;
    private CRServo  rpull;
    private double lp;
    private double rp;

    /**
     * Creates a new pull.
     */
    public Intake(HardwareMap hardwareMap, String lsn, String rsn, Telemetry telemetry) {
        this.lpull = new CRServo(hardwareMap, lsn);
        this.rpull = new CRServo(hardwareMap, rsn);
        this.lp = 0;
        this.rp = 0;
    }
    public void servospin(double yer) {
        this.lp = yer;
        this.rp = yer;
    }

    @Override
    public void periodic() {
        lpull.set(lp);
        rpull.set(rp);
    }
}