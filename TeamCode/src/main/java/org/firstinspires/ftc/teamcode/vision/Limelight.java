package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.LimelightConstants.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

public class Limelight extends SubsystemBase {
    private final Limelight3A limelight;
    private final TurretHistory history;

    private List<LLResultTypes.FiducialResult> visibleTags = Collections.emptyList();
    private boolean resultValid = false;

    private double lastFrameTimestamp = Double.NaN;
    private boolean newFrame = false;
    private double frameAgeMs = Double.NaN;
    private final List<CellLocator.TagPoint> tagPoints = new ArrayList<>();
    private final Map<HiveCells.Cell, CellObservation> observations = new EnumMap<>(HiveCells.Cell.class);

    public Limelight(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Limelight(HardwareMap hardwareMap, TurretHistory history) {
        this.history = history;
        limelight = hardwareMap.get(Limelight3A.class, DEVICE_NAME);
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.setPollRateHz(POLL_RATE_HZ);
        limelight.start();
    }

    @Override
    public void periodic() {
        newFrame = false;
        if (!limelight.isConnected()) {
            resultValid = false;
            visibleTags = Collections.emptyList();
            clearObservations();
            return;
        }

        LLResult result = limelight.getLatestResult();
        resultValid = result != null && result.isValid();
        visibleTags = resultValid ? result.getFiducialResults() : Collections.emptyList();

        if (history == null) return;
        if (!resultValid) {
            clearObservations();
            return;
        }

        if (result.getTimestamp() == lastFrameTimestamp) return;
        lastFrameTimestamp = result.getTimestamp();

        frameAgeMs = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
        if (frameAgeMs > MAX_AGE_MS) {
            clearObservations();
            return;
        }
        TurretHistory.Sample atCapture = history.at(history.now() - frameAgeMs / 1000);
        if (atCapture == null) {
            clearObservations();
            return;
        }

        CellLocator.CameraMount mount = new CellLocator.CameraMount(CAM_FWD, CAM_LEFT, CAM_UP,
                Math.toRadians(CAM_PITCH_DEG), Math.toRadians(CAM_YAW_DEG), Math.toRadians(CAM_ROLL_DEG));
        tagPoints.clear();
        Map<HiveCells.Cell, List<CellLocator.TagPoint>> byCell = new EnumMap<>(HiveCells.Cell.class);
        for (LLResultTypes.FiducialResult tag : visibleTags) {
            HiveCells.Cell cell = HiveCells.forTag(tag.getFiducialId());
            if (cell == null) continue;
            Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
            double[] t = CellLocator.cameraToTurret(p.x, p.y, p.z, mount);
            CellLocator.TagPoint point = new CellLocator.TagPoint(
                    tag.getFiducialId(), cell.tagIndex(tag.getFiducialId()), t[0], t[1], t[2]);
            tagPoints.add(point);
            List<CellLocator.TagPoint> list = byCell.get(cell);
            if (list == null) {
                list = new ArrayList<>();
                byCell.put(cell, list);
            }
            list.add(point);
        }

        observations.clear();
        for (Map.Entry<HiveCells.Cell, List<CellLocator.TagPoint>> entry : byCell.entrySet()) {
            observations.put(entry.getKey(), CellLocator.locate(entry.getKey(), entry.getValue(), atCapture,
                    TurretConstants.TURRET_FWD, TurretConstants.TURRET_LEFT));
        }
        newFrame = true;
    }

    private void clearObservations() {
        observations.clear();
        tagPoints.clear();
    }

    public boolean hasNewFrame() {
        return newFrame;
    }

    public CellObservation getObservation(HiveCells.Cell cell) {
        return observations.get(cell);
    }

    public Map<HiveCells.Cell, CellObservation> getObservations() {
        return observations;
    }

    public List<CellLocator.TagPoint> getTagPoints() {
        return tagPoints;
    }

    public double getFrameAgeMs() {
        return frameAgeMs;
    }

    public boolean isConnected() {
        return limelight.isConnected();
    }

    public boolean hasValidResult() {
        return resultValid;
    }

    public boolean isTagVisible(int id) {
        return getFiducial(id) != null;
    }

    public LLResultTypes.FiducialResult getFiducial(int id) {
        for (LLResultTypes.FiducialResult tag : visibleTags) {
            if (tag.getFiducialId() == id) return tag;
        }
        return null;
    }

    public LLResultTypes.FiducialResult getFirstVisible(int[] ids) {
        for (LLResultTypes.FiducialResult tag : visibleTags) {
            for (int id : ids) {
                if (tag.getFiducialId() == id) return tag;
            }
        }
        return null;
    }

    public List<LLResultTypes.FiducialResult> getVisibleTags() {
        return visibleTags;
    }
}
