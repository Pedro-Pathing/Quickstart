package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;


public class CameraSubsystem extends RE_SubsystemBase {

    private Limelight3A limelight;

    private LLResult limelightResult;
    private Obelisk obeliskResult;

    private CameraState cameraState;


    public enum CameraState {
        ON,
        OFF
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
        }

    }

}