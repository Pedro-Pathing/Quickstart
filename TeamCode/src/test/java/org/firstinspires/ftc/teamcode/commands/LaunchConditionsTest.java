package org.firstinspires.ftc.teamcode.commands;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.firstinspires.ftc.teamcode.vision.LimelightCamera;
import org.junit.Before;
import org.junit.Test;

/** The launch rule: no collect, a ready camera, and (new) the aim locked if auto-aim is on. */
public class LaunchConditionsTest {
    private LimelightCamera camera;
    private ShotReadiness readiness;

    @Before
    public void setUp() {
        camera = mock(LimelightCamera.class);
        readiness = mock(ShotReadiness.class);
        when(readiness.isReady()).thenReturn(true);
    }

    @Test
    public void aimStillTurningBlocksALaunch() {
        LaunchCommands.Conditions c = new LaunchCommands.Conditions(camera, readiness, () -> false, () -> false);
        assertFalse(c.ready());
    }

    @Test
    public void aimLockedOrOffAllowsALaunch() {
        LaunchCommands.Conditions c = new LaunchCommands.Conditions(camera, readiness, () -> false, () -> true);
        assertTrue(c.ready());
    }

    @Test
    public void aimCheckIsReadEveryTime() {
        boolean[] locked = {false};
        LaunchCommands.Conditions c = new LaunchCommands.Conditions(camera, readiness, () -> false, () -> locked[0]);
        assertFalse(c.ready());
        locked[0] = true;
        assertTrue(c.ready());
    }

    @Test
    public void oldConstructorHasNoAimCheck() {
        LaunchCommands.Conditions c = new LaunchCommands.Conditions(camera, readiness, () -> false);
        assertTrue(c.ready());
    }

    @Test
    public void collectingStillBlocksEvenWhenAimIsReady() {
        LaunchCommands.Conditions c = new LaunchCommands.Conditions(camera, readiness, () -> true, () -> true);
        assertFalse(c.ready());
    }

    @Test
    public void cameraNotReadyStillBlocks() {
        when(readiness.isReady()).thenReturn(false);
        LaunchCommands.Conditions c = new LaunchCommands.Conditions(camera, readiness, () -> false, () -> true);
        assertFalse(c.ready());
    }
}
