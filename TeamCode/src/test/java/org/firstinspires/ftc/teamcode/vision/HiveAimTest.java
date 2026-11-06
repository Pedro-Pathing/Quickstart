package org.firstinspires.ftc.teamcode.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Test;

public class HiveAimTest {
    private static final long MS = 1_000_000L;
    private static final double IN = 0.0254;

    private final HiveAim aim = new HiveAim();

    private static HiveTarget target(double x, double y, long frame, long captureNanos) {
        return new HiveTarget(HiveTarget.Cell.SCORING, x, y, 1.48, 2, 1, 0, frame, captureNanos);
    }

    @Test
    public void usesOdometryBeforeTheCameraHasSeenTags() {
        double h = aim.update(Alliance.RED, null, new Pose(30, 20, 0), 0);
        assertEquals(HiveCells.headingToNearestCell(Alliance.RED, 30, 20), h, 1e-12);
        assertFalse(aim.isFromVision());
    }

    @Test
    public void odometryFollowsTheRobotWhileNoTagsHaveBeenSeen() {
        aim.update(Alliance.RED, null, new Pose(30, 20, 0), 0);
        double h = aim.update(Alliance.RED, null, new Pose(40, 50, 0), 20 * MS);
        assertEquals(HiveCells.headingToNearestCell(Alliance.RED, 40, 50), h, 1e-12);
    }

    @Test
    public void aTargetStraightAheadAimsAlongTheHeading() {
        double h = aim.update(Alliance.RED, target(1.0, 0, 1, 0), new Pose(48, 72, 0), 0);
        assertEquals(0.0, h, 1e-9);
        assertTrue(aim.isFromVision());
    }

    @Test
    public void targetIsConvertedToAFieldPointUsingTheFullPose() {
        // Robot at (48, 72) facing +Y; the target is 1 m ahead and 0.5 m to its left, which is
        // 1 m further +Y and 0.5 m towards -X
        aim.update(Alliance.RED, target(1.0, 0.5, 1, 0), new Pose(48, 72, Math.PI / 2), 0);
        double px = 48 - 0.5 / IN, py = 72 + 1.0 / IN;
        double h = aim.update(Alliance.RED, null, new Pose(20, 40, 0), 20 * MS);
        assertEquals(Math.atan2(py - 40, px - 20), h, 1e-9);
    }

    @Test
    public void pointStaysFixedInTheFieldAsTheRobotTranslatesAfterTagsDrop() {
        double h0 = aim.update(Alliance.RED, target(1.0, 0, 1, 0), new Pose(48, 72, 0), 0);
        assertEquals(0.0, h0, 1e-9);
        double px = 48 + 1.0 / IN, py = 72;

        // Robot drives towards -Y with the tags lost: the aim swings left to keep pointing at the point
        double h1 = aim.update(Alliance.RED, null, new Pose(48, 40, 0), 20 * MS);
        assertEquals(Math.atan2(py - 40, px - 48), h1, 1e-9);
        assertTrue(h1 > 0);

        // Further along, and turned: the heading is still the direction to the same point
        double h2 = aim.update(Alliance.RED, null, new Pose(60, 40, 1.0), 40 * MS);
        assertEquals(Math.atan2(py - 40, px - 60), h2, 1e-9);
        assertTrue(aim.isFromVision());
    }

    @Test
    public void turningInPlaceDoesNotMoveTheAim() {
        aim.update(Alliance.RED, target(1.0, 0.3, 1, 0), new Pose(48, 72, 0), 0);
        double before = aim.update(Alliance.RED, null, new Pose(48, 72, 0), 20 * MS);
        double after = aim.update(Alliance.RED, null, new Pose(48, 72, 2.0), 40 * MS);
        assertEquals(before, after, 1e-12);
    }

    @Test
    public void aNewFrameUpdatesThePoint() {
        aim.update(Alliance.RED, target(1.0, 0, 1, 0), new Pose(48, 72, 0), 0);
        double h = aim.update(Alliance.RED, target(1.0, 1.0, 2, 20 * MS), new Pose(48, 72, 0), 20 * MS);
        assertEquals(Math.PI / 4, h, 1e-9);
    }

    @Test
    public void theSameFrameIsNotConvertedTwice() {
        HiveTarget t = target(1.0, 0, 1, 0);
        aim.update(Alliance.RED, t, new Pose(48, 72, 0), 0);
        // The robot moves and the same frame is still reported. Converting it again with the new
        // pose would carry the point along with the robot and the heading would stay 0
        double h = aim.update(Alliance.RED, t, new Pose(48, 40, 0), 20 * MS);
        assertEquals(Math.atan2(32, 1.0 / IN), h, 1e-9);
    }

    @Test
    public void usesTheHeadingAtCaptureTimeNotNow() {
        aim.update(Alliance.RED, null, new Pose(48, 72, 0), 0);
        aim.update(Alliance.RED, null, new Pose(48, 72, Math.PI / 2), 100 * MS);
        // A frame captured at t = 0 with the target dead ahead of the robot as it was facing then
        double h = aim.update(Alliance.RED, target(1.0, 0, 1, 0), new Pose(48, 72, Math.PI / 2), 120 * MS);
        assertEquals(0.0, h, 1e-9);
    }

    @Test
    public void usesThePositionAtCaptureTimeToo() {
        aim.update(Alliance.RED, null, new Pose(40, 72, 0), 0);
        aim.update(Alliance.RED, null, new Pose(60, 72, 0), 100 * MS);
        // Captured halfway, when the robot was at x = 50
        aim.update(Alliance.RED, target(1.0, 0, 1, 50 * MS), new Pose(60, 72, 0), 100 * MS);
        double h = aim.update(Alliance.RED, null, new Pose(60, 50, 0), 120 * MS);
        assertEquals(Math.atan2(72 - 50, 50 + 1.0 / IN - 60), h, 1e-9);
    }

    @Test
    public void resetForgetsThePointAndGoesBackToOdometry() {
        aim.update(Alliance.RED, target(1.0, 0, 1, 0), new Pose(48, 72, 0), 0);
        assertTrue(aim.isFromVision());
        aim.reset();
        assertFalse(aim.isFromVision());
        double h = aim.update(Alliance.RED, null, new Pose(30, 20, 0), 20 * MS);
        assertEquals(HiveCells.headingToNearestCell(Alliance.RED, 30, 20), h, 1e-12);
    }

    @Test
    public void afterResetTheSameFrameNumberIsAcceptedAgain() {
        aim.update(Alliance.RED, target(1.0, 0, 1, 0), new Pose(48, 72, 0), 0);
        aim.reset();
        double h = aim.update(Alliance.RED, target(1.0, 1.0, 1, 20 * MS), new Pose(48, 72, 0), 20 * MS);
        assertEquals(Math.PI / 4, h, 1e-9);
    }

    @Test
    public void veryCloseToThePointHoldsTheLastHeading() {
        aim.update(Alliance.RED, target(1.0, 0, 1, 0), new Pose(48, 72, 0), 0);
        double px = 48 + 1.0 / IN;
        double far = aim.update(Alliance.RED, null, new Pose(px - 20, 60, 0), 20 * MS);
        // Almost on top of the point the direction is meaningless, so the heading is held
        double near = aim.update(Alliance.RED, null, new Pose(px - 1, 72.5, 0), 40 * MS);
        assertEquals(far, near, 1e-12);
        // Driving away again, the aim recovers and points at the same point
        double back = aim.update(Alliance.RED, null, new Pose(px - 20, 60, 0), 60 * MS);
        assertEquals(far, back, 1e-12);
    }

    @Test
    public void isLockedNeedsSmallErrorAndSlowTurn() {
        assertTrue(HiveAim.isLocked(0.0, 0.0));
        assertFalse(HiveAim.isLocked(Math.toRadians(5), 0.0));
        assertFalse(HiveAim.isLocked(0.0, 1.0));
    }
}
