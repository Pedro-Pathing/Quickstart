package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();

    private DcMotorEx flywheel;

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {

    }

}
