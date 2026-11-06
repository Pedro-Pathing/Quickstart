package org.firstinspires.ftc.teamcode.drive;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;

import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.vision.Alliance;
import org.firstinspires.ftc.teamcode.vision.FloorBall;
import org.firstinspires.ftc.teamcode.vision.LimelightCamera;
import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * CollectAssist's states and timeouts, on a fake clock. The robot sits at the origin facing +X
 * (Pedro inches), still, so a ball straight ahead needs no turn and one off to the side never
 * gets aimed at.
 */
public class CollectAssistTest {
    private Follower follower;
    private LimelightCamera camera;
    private Intake intake;
    private CollectAssist assist;
    private final double[] now = {0.0};
    private long frame = 0;

    @Before
    public void setUp() {
        CollectAssist.searchTimeoutSeconds = 1.5;
        CollectAssist.collectTimeoutSeconds = 4.0;
        CollectAssist.intakeAfterSeconds = 0.5;
        follower = mock(Follower.class);
        when(follower.pose()).thenReturn(new Pose(0, 0, 0));
        when(follower.twist()).thenReturn(Twist.zero());
        camera = mock(LimelightCamera.class);
        when(camera.isFloorAvailable()).thenReturn(true);
        intake = mock(Intake.class);
        assist = new CollectAssist(follower, camera, intake, () -> now[0]);
    }

    private FloorBall ball(FloorBall.Kind kind, double x, double y) {
        return new FloorBall(kind, x, y, false, frame, 0L);
    }

    /** A fresh camera frame holding balls of these kinds at these robot-frame positions {x, y}. */
    private List<FloorBall> newFrame(Object... kindsAndPositions) {
        frame++;
        FloorBall[] balls = new FloorBall[kindsAndPositions.length / 2];
        for (int i = 0; i < balls.length; i++) {
            double[] p = (double[]) kindsAndPositions[2 * i + 1];
            balls[i] = ball((FloorBall.Kind) kindsAndPositions[2 * i], p[0], p[1]);
        }
        return Arrays.asList(balls);
    }

    private void step(double seconds, List<FloorBall> balls) {
        now[0] += seconds;
        assist.update(balls, Alliance.RED);
    }

    private boolean collecting() {
        CollectAssist.State s = assist.getState();
        return s == CollectAssist.State.SEARCHING || s == CollectAssist.State.TURNING
                || s == CollectAssist.State.DRIVING;
    }

    @Test
    public void doesNothingWithoutAFloorCamera() {
        when(camera.isFloorAvailable()).thenReturn(false);
        assist.start();
        assertEquals(CollectAssist.State.IDLE, assist.getState());
    }

    @Test
    public void givesUpAfterTheSearchTimeoutWhenNoBallsAreSeen() {
        assist.start();
        assertEquals(CollectAssist.State.SEARCHING, assist.getState());
        step(1.0, Collections.emptyList());
        assertEquals(CollectAssist.State.SEARCHING, assist.getState());
        step(0.6, Collections.emptyList());
        assertEquals(CollectAssist.State.IDLE, assist.getState());
        verify(camera).setMode(LimelightCamera.Mode.TAGS);
    }

    @Test
    public void ignoresTheOtherAlliancesNectar() {
        assist.start();
        step(0.1, newFrame(FloorBall.Kind.BLUE_NECTAR, new double[] {1.0, 0.0}));
        assertEquals(CollectAssist.State.SEARCHING, assist.getState());
    }

    @Test
    public void aReachableBallGoesSearchingTurningDrivingAndFinishesOnPickup() {
        assist.start();
        step(0.1, newFrame(FloorBall.Kind.POLLEN, new double[] {1.0, 0.0}));
        assertEquals(CollectAssist.State.TURNING, assist.getState());
        // Already facing it and still, so it's lined up on the next loop
        step(0.1, null);
        assertEquals(CollectAssist.State.DRIVING, assist.getState());
        verify(intake).run();
        when(intake.hasPickedUp()).thenReturn(true);
        step(0.1, null);
        assertEquals(CollectAssist.State.FINISHING, assist.getState());
        assertFalse(assist.isActive());
        step(0.6, null);
        assertEquals(CollectAssist.State.IDLE, assist.getState());
        verify(intake).stop();
    }

    @Test
    public void turningTimesOutOverallWhenTheBallCannotBeFacedAndNoFramesArrive() {
        assist.start();
        // 90 degrees to the left: the robot is held still, so it never gets lined up
        step(0.1, newFrame(FloorBall.Kind.POLLEN, new double[] {0.0, 1.0}));
        assertEquals(CollectAssist.State.TURNING, assist.getState());
        for (int i = 0; i < 7; i++) step(0.5, null);
        assertEquals(CollectAssist.State.TURNING, assist.getState()); // 3.6 s
        step(0.5, null);                                              // 4.1 s
        assertFalse(collecting());
    }

    @Test
    public void aBallThatKeepsBeingBlockedNeverRunsPastTheOverallTimeout() {
        assist.start();
        double[] pollen = {0.0, 1.0};
        double[] blocker = {0.0, 0.5};
        // Alternate a clear frame (-> TURNING) with a blocked one (-> back to SEARCHING), as
        // happens when their Nectar keeps rolling in the way. Each state change used to restart
        // the timer, so this never ended.
        boolean blocked = false;
        double elapsed = 0.0;
        while (elapsed < 3.8) {
            step(0.3, blocked ? newFrame(FloorBall.Kind.POLLEN, pollen, FloorBall.Kind.BLUE_NECTAR, blocker)
                    : newFrame(FloorBall.Kind.POLLEN, pollen));
            elapsed += 0.3;
            blocked = !blocked;
            assertEquals("still collecting at " + elapsed, true, collecting());
        }
        for (int i = 0; i < 3; i++) {
            step(0.3, blocked ? newFrame(FloorBall.Kind.POLLEN, pollen, FloorBall.Kind.BLUE_NECTAR, blocker)
                    : newFrame(FloorBall.Kind.POLLEN, pollen));
            blocked = !blocked;
        }
        assertFalse(collecting());
    }

    @Test
    public void aSecondCollectGetsAFreshOverallTimer() {
        assist.start();
        step(2.0, Collections.emptyList());
        assertEquals(CollectAssist.State.IDLE, assist.getState());
        step(10.0, null);
        assist.start();
        step(1.0, Collections.emptyList());
        assertEquals(CollectAssist.State.SEARCHING, assist.getState());
    }
}
