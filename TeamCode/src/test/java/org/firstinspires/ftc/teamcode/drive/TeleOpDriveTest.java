package org.firstinspires.ftc.teamcode.drive;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.junit.After;
import org.junit.Test;

/** Which heading lock TeleOpDrive uses, depending on whether Constants marks Foresight as tuned. */
public class TeleOpDriveTest {
    @After
    public void restore() {
        Constants.foresightTuned = false;
        TeleOpDrive.useForesight = true;
    }

    @Test
    public void foresightIsUsedOnceItIsMarkedTuned() {
        Constants.foresightTuned = true;  // as the sim's sim-overrides.txt sets it
        TeleOpDrive.useForesight = true;
        assertTrue(new TeleOpDrive(mock(Follower.class)).usingForesight());
    }

    @Test
    public void foresightIsNotUsedWhileUntuned() {
        Constants.foresightTuned = false;
        TeleOpDrive.useForesight = true;
        assertFalse(new TeleOpDrive(mock(Follower.class)).usingForesight());
    }

    @Test
    public void foresightStaysOffWhenSwitchedOffEvenIfTuned() {
        Constants.foresightTuned = true;
        TeleOpDrive.useForesight = false;
        assertFalse(new TeleOpDrive(mock(Follower.class)).usingForesight());
    }
}
