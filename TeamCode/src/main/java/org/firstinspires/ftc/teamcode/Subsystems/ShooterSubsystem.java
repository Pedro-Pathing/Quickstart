package org.firstinspires.ftc.teamcode.Subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class ShooterSubsystem {

    private final DcMotorEx flywheel1;
    private final DcMotorEx flywheel2;
    private final DcMotorEx turret;
    private final Servo hood;
    private final Limelight3A limelight;

    // Turret PIDF
    public static double turretKp = 1.5;
    public static double turretKi = 0.0;
    public static double turretKd = 0.03;
    public static double turretKf = 0.03;

    // Distance to RPM (from ta)
    public static double C = -2000;
    public static double D = 6000;

    // Distance to the hood
    public static double A = 19.67;
    public static double B = 6;

    // Motion compensation (RPM only)
    public static double RPM_PER_MPS = 0;

    // RPM limits
    public static double MIN_RPM = 3000;
    public static double MAX_RPM = 4700;
    private double turretIntegral = 0;
    private double lastError = 0;

    private double filteredTa = 0;
    private static final double TA_ALPHA = 0.15;

    private static final double TICKS_TO_RAD = 2.0 * Math.PI / 8192.0;

    public ShooterSubsystem(HardwareMap hw) {

        flywheel1 = hw.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hw.get(DcMotorEx.class, "flywheel2");
        turret = hw.get(DcMotorEx.class, "turret");
        hood = hw.get(Servo.class, "hood");
        limelight = hw.get(Limelight3A.class, "limelight");

        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        hood.setPosition(0);
    }

    public void update(Pose pose, Vector velocity, double dt, Telemetry telemetry) {

        double tps1 = flywheel1.getVelocity();
        double tps2 = flywheel2.getVelocity();
        double rpm1 = (tps1 / 28) * 60;
        double rpm2 = (tps2 / 28) * 60;

        double avgRPM = (rpm1 + rpm2) / 2;


        telemetry.addData("Flywheel RPM", avgRPM);
        telemetry.addData("Hood Position", hood.getPosition());
        telemetry.addData("Limelight tx", limelight.getLatestResult().getTx());
        telemetry.addData("Limelight ta", limelight.getLatestResult().getTa());
        telemetry.update();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            filteredTa = TA_ALPHA * result.getTa() + (1 - TA_ALPHA) * filteredTa;
        } // else keep previous filteredTa

        // --- TURRET TRACKING ---
        if (result != null && result.isValid()) {

            double txRad = Math.toRadians(result.getTx());

            // Normalize to -π..π for shortest rotation
            txRad = ((txRad + Math.PI) % (2 * Math.PI)) - Math.PI;

            // PIDF calculations
            turretIntegral += txRad * dt;
            double derivative = (txRad - lastError) / dt;

            double output =
                    turretKp * txRad +
                            turretKi * turretIntegral +
                            turretKd * derivative +
                            turretKf * Math.signum(txRad);

            turret.setPower(output);
            lastError = txRad;

        } else {
            // Target lost → stop turret
            turret.setPower(0);
        }

        // robot motion
        double turretAngle =
                turret.getCurrentPosition() * TICKS_TO_RAD;

        double vForward =
                velocity.getXComponent() * Math.cos(turretAngle)
                        + velocity.getYComponent() * Math.sin(turretAngle);

        // flywheel RPM
        double targetRPM =
                C * Math.sqrt(filteredTa) + D;

        // Backward compensation only
        targetRPM += (-vForward) * RPM_PER_MPS;
        targetRPM = clamp(targetRPM, MIN_RPM, MAX_RPM);

        double TPS = targetRPM * 28 / 60;


        flywheel1.setVelocity(TPS);
        flywheel2.setVelocity(TPS);

        // hood angle
        double hoodAngle = A * Math.sqrt(filteredTa) + B;  // hood angle in degrees

        hoodAngle = clamp(hoodAngle, 23, 58);

// Convert to servo position
        hood.setPosition(angleToServo(hoodAngle));
    }

    private double angleToServo(double angleDeg){
        // Map hood angle (degrees) to servo position (0-1)
        // 1 = 58° (fully down), 0 = 23° (fully up)
        double servoPos = (58.0 - angleDeg) / (58.0 - 23.0);
        return clamp(servoPos, 0, 1);
    }



    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public DcMotorEx getFlywheel1() {
        return flywheel1;
    }

    public DcMotorEx getFlywheel2() {
        return flywheel2;
    }
    public Servo getHood() {
        return hood;
    }

}
