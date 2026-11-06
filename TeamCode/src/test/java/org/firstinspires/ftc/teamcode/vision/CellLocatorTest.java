package org.firstinspires.ftc.teamcode.vision;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CellLocatorTest {
    private static final double EPS = 0.0001;
    private static final CellLocator.CameraMount LEVEL = new CellLocator.CameraMount(0, 0, 0, 0, 0);

    @Test
    public void levelCameraMapsRightDownForward() {
        assertArrayEquals(new double[]{10, -1, -2}, CellLocator.cameraToTurret(1, 2, 10, LEVEL), EPS);
    }

    @Test
    public void pitchedCameraLooksUp() {
        CellLocator.CameraMount mount = new CellLocator.CameraMount(0, 0, 0, Math.toRadians(30), 0);
        double[] p = CellLocator.cameraToTurret(0, 0, 10, mount);
        assertArrayEquals(new double[]{10 * Math.cos(Math.toRadians(30)), 0, 5}, p, EPS);
    }

    @Test
    public void yawedCameraLooksLeftAndOffsetIsAdded() {
        CellLocator.CameraMount mount = new CellLocator.CameraMount(3, 1, 2, 0, Math.toRadians(90));
        assertArrayEquals(new double[]{3, 11, 2}, CellLocator.cameraToTurret(0, 0, 10, mount), EPS);
    }

    @Test
    public void rolledCameraTurnsImageRightIntoUp() {
        CellLocator.CameraMount mount = new CellLocator.CameraMount(0, 0, 0, 0, 0, Math.toRadians(90));
        assertArrayEquals(new double[]{10, 0, 1}, CellLocator.cameraToTurret(1, 0, 10, mount), EPS);
        assertArrayEquals(new double[]{10, -2, 0}, CellLocator.cameraToTurret(0, 2, 10, mount), EPS);
    }

    private static List<CellLocator.TagPoint> row(double f, double l, double yaw, int... indices) {
        List<CellLocator.TagPoint> tags = new ArrayList<>();
        for (int i : indices) {
            double o = HiveCells.TAG_OFFSETS_IN[i];
            tags.add(new CellLocator.TagPoint(42 + i, i, f + o * Math.cos(yaw), l + o * Math.sin(yaw), 30));
        }
        return tags;
    }

    @Test
    public void fourTagsGiveRowDirectionAndMiddle() {
        double yaw = Math.toRadians(25);
        List<CellLocator.TagPoint> tags = row(40, -5, yaw, 0, 1, 2, 3);
        assertEquals(yaw, CellLocator.measureRowYaw(tags), EPS);
        assertArrayEquals(new double[]{40, -5}, CellLocator.rowCenter(tags, yaw), EPS);
    }

    @Test
    public void twoUnevenTagsStillFindTheMiddle() {
        double yaw = Math.toRadians(-70);
        List<CellLocator.TagPoint> tags = row(30, 12, yaw, 1, 3);
        double measured = CellLocator.measureRowYaw(tags);
        assertEquals(yaw, measured, EPS);
        assertArrayEquals(new double[]{30, 12}, CellLocator.rowCenter(tags, measured), EPS);
    }

    @Test
    public void oneTagHasNoMeasuredDirection() {
        assertTrue(Double.isNaN(CellLocator.measureRowYaw(row(30, 0, 0, 2))));
    }

    @Test
    public void turretPointGoesToFieldWithPivotOffset() {
        TurretHistory.Sample at = new TurretHistory.Sample(0, 0, 10, 20, Math.toRadians(90));
        assertArrayEquals(new double[]{10, 27}, CellLocator.turretToField(5, 0, at, 2, 0), EPS);

        at = new TurretHistory.Sample(0, Math.toRadians(90), 10, 20, Math.toRadians(90));
        assertArrayEquals(new double[]{5, 22}, CellLocator.turretToField(5, 0, at, 2, 0), EPS);
    }

    @Test
    public void observationReportsBearingDistanceAndTagIds() {
        TurretHistory.Sample at = new TurretHistory.Sample(1.23, 0, 0, 0, 0);
        CellObservation obs = CellLocator.locate(HiveCells.Cell.BLUE_FAR, row(30, 30, 0.3, 0, 1, 2, 3), at, 0, 0);
        assertEquals(Math.toRadians(45), obs.bearing, EPS);
        assertEquals(Math.hypot(30, 30), obs.distance, EPS);
        assertEquals(4, obs.tagCount());
        assertEquals(Arrays.toString(new int[]{42, 43, 44, 45}), Arrays.toString(obs.tagIds));
        assertEquals(1.23, obs.captureTime, EPS);
    }

    @Test
    public void oneTagUsesCellHeadingForRowDirection() {
        TurretHistory.Sample at = new TurretHistory.Sample(0, 0, 0, 0, 0);
        double yaw = HiveCells.Cell.BLUE_FAR.pose.heading();
        CellObservation obs = CellLocator.locate(HiveCells.Cell.BLUE_FAR, row(30, 5, yaw, 3), at, 0, 0);
        assertEquals(30, obs.forward, EPS);
        assertEquals(5, obs.left, EPS);
    }
}
