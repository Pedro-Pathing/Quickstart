package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static org.firstinspires.ftc.teamcode.util.Globals.ALLIANCE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;


public class CameraSubsystem extends RE_SubsystemBase {

    private Limelight3A limelight;

    private LLResult limelightResult;
    private Obelisk obeliskResult;
    private ShootDistance shootDistance;

    private CameraState cameraState;

    final double minRange = 1.0; //we shd change these values theyre in meters
    final double maxRange = 3.0;

    double zDistance;
    double yDistance;
    double xDistance;
    double distance;



    public enum CameraState {
        ON,
        OFF
    }

    public enum ShootDistance {
        INRANGE,
        OUTOFRANGE
    }

    public enum Obelisk {
        PPG,
        PGP,
        GPP,
        PPP
    }

    public CameraSubsystem(HardwareMap hardwareMap, String limelight) {

        this.limelight = hardwareMap.get(Limelight3A.class, limelight);

        obeliskResult = Obelisk.PPP;
        ShootDistance shootDistance = ShootDistance.OUTOFRANGE;


        startCamera();

        Robot.getInstance().subsystems.add(this);
    }

    private void startCamera() {
        cameraState = CameraState.ON;
        limelight.start();
    }

    private void stopCamera() {
        cameraState = CameraState.OFF;
        limelight.stop();
    }

    public Obelisk getObelisk() {
        return obeliskResult;
    }

    @Override
    public void updateData() {
        Robot.getInstance().data.obelisk = obeliskResult;
    }

    @Override
    public void periodic() {

        if (cameraState == CameraState.ON) {
            limelightResult = limelight.getLatestResult();
        }

        for (LLResultTypes.FiducialResult target: limelightResult.getFiducialResults()) {
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
            if((ALLIANCE == Globals.COLORS.BLUE) && target.getFiducialId() == 20) {
                    zDistance = target.getTargetPoseCameraSpace().getPosition().z;
                    yDistance = target.getTargetPoseCameraSpace().getPosition().y;
                    xDistance = target.getTargetPoseCameraSpace().getPosition().x;
                    distance = Math.sqrt(xDistance * xDistance + yDistance * yDistance + zDistance * zDistance);

                    if (distance >= minRange && distance <= maxRange) {
                        shootDistance = ShootDistance.INRANGE;
                    } else {
                        shootDistance = ShootDistance.OUTOFRANGE;
                    }
            }
            else if((ALLIANCE == Globals.COLORS.RED) && target.getFiducialId() == 24) {
                    zDistance = target.getTargetPoseCameraSpace().getPosition().z;
                    yDistance = target.getTargetPoseCameraSpace().getPosition().y;
                    xDistance = target.getTargetPoseCameraSpace().getPosition().x;
                    distance = Math.sqrt(xDistance * xDistance + yDistance * yDistance + zDistance * zDistance);

                    if (distance >= minRange && distance <= maxRange) {
                        shootDistance = ShootDistance.INRANGE;
                    } else {
                        shootDistance = ShootDistance.OUTOFRANGE;
                    }
            }
        }



    }

}