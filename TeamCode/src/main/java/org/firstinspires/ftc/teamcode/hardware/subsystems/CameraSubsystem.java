package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.util.Globals.ALLIANCE;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

import java.util.List;

public class CameraSubsystem extends RE_SubsystemBase {

    private final Limelight3A limelight;

    private LLResult limelightResult;
    private Obelisk obeliskResult;
    private ShootDistance shootDistance;
    private CameraState cameraState;

    // tune these por favor

    private final double minRange = 1.0;
    private final double maxRange = 3.0;
    private double CAMERA_HEIGHT_M = 0.30; // the actual lens heigh in meters
    private double TAG_HEIGHT_M    = 1.22;     // center of april tag heigh make sure to do ceneter
    private double CAMERA_PITCH_RAD = toRadians(0.0); //keep perpendicular to the ground

    private double cameraYawOffsetDeg = 0.0;

    private LLResultTypes.FiducialResult bestBasketTarget = null;


    private double distanceM = Double.NaN;

    public enum CameraState {
        ON,
        OFF }
    public enum ShootDistance {
        INRANGE,
        OUTOFRANGE }
    public enum Obelisk {
        PPG,
        PGP,
        GPP,
        PPP }

    public CameraSubsystem(HardwareMap hardwareMap, String limelightName) {
        this.limelight = hardwareMap.get(Limelight3A.class, limelightName);
        obeliskResult = Obelisk.PPP;
        this.shootDistance = ShootDistance.OUTOFRANGE;

        startCamera();
        Robot.getInstance().subsystems.add(this);
    }

    private void startCamera() {
        cameraState = CameraState.ON;
        limelight.setPollRateHz(100);
        limelight.start();
    }

    @SuppressWarnings("unused")

    private void stopCamera() {
        cameraState = CameraState.OFF;
        limelight.stop();
    }


    /** true if we see an alliance basket tag this frame */
    public boolean hasBasket() { return bestBasketTarget != null; }


    public double getBasketYawDeg() {
        if (bestBasketTarget != null) {
            return bestBasketTarget.getTargetXDegrees() + cameraYawOffsetDeg;
        }
        if (limelightResult != null && limelightResult.isValid()) {
            return limelightResult.getTx() + cameraYawOffsetDeg; //how much needed to turn
        }
        return Double.NaN;
    }

    //distance estimate in meters from ty + geometry
    public double getBasketDistanceM() {
        return distanceM;
    }

    public ShootDistance getShootDistance() { return shootDistance; }
    public Obelisk getObelisk() { return obeliskResult; }
    public void setCameraYawOffsetDeg(double offset) { this.cameraYawOffsetDeg = offset; }

    @Override
    public void updateData() {
        Robot.getInstance().data.obelisk = obeliskResult;
    }

    @Override
    public void periodic() {
        if (cameraState == CameraState.ON) {
            limelightResult = limelight.getLatestResult();
        }

        bestBasketTarget = null;
        distanceM = Double.NaN;

        if (limelightResult == null || !limelightResult.isValid()) return;


        List<LLResultTypes.FiducialResult> fiducials = limelightResult.getFiducialResults();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult target : fiducials) {

                switch (target.getFiducialId()) {
                    case 21:
                        obeliskResult = Obelisk.GPP;
                        break;
                    case 22:
                        obeliskResult = Obelisk.PGP;
                        break;
                    case 23:
                        obeliskResult = Obelisk.PPG;
                        break;
                    default:
                        break;
                }

                boolean isAllianceBasket =
                        (ALLIANCE == Globals.COLORS.BLUE && target.getFiducialId() == 20) ||
                                (ALLIANCE == Globals.COLORS.RED  && target.getFiducialId() == 24);

                if (isAllianceBasket) {
                    if (bestBasketTarget == null) {
                        bestBasketTarget = target;
                    } else {
                        if (abs(target.getTargetXDegrees()) < abs(bestBasketTarget.getTargetXDegrees())) {
                            bestBasketTarget = target;
                        }
                    }
                }
            }
        }

        // --- Distance estimate from camera geometry using ty ---
        // Use the best basket target’s vertical angle if we have one; otherwise fallback to result ty.
        Double tyDeg = null;
        if (bestBasketTarget != null) {
            tyDeg = bestBasketTarget.getTargetYDegrees();
        } else if (limelightResult.isValid()) {
            tyDeg = limelightResult.getTy();
        }

        if (tyDeg != null) {
            double tyRad = toRadians(tyDeg);
            double denom = tan(CAMERA_PITCH_RAD + tyRad);
            if (abs(denom) > 1e-6) {

                distanceM = (TAG_HEIGHT_M - CAMERA_HEIGHT_M) / denom;
            } else {
                distanceM = Double.NaN;
            }
        }

        if (!Double.isNaN(distanceM) && distanceM >= minRange && distanceM <= maxRange) {
            shootDistance = ShootDistance.INRANGE;
        } else {
            shootDistance = ShootDistance.OUTOFRANGE;
        }

        Robot.getInstance().data.obelisk = obeliskResult;
    }
}