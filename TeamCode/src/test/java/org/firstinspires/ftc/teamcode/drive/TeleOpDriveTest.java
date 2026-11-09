package org.firstinspires.ftc.teamcode.drive;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.junit.After;
import org.junit.Test;

/** Which heading lock TeleOpDrive uses for each constants set. */
public class TeleOpDriveTest {
    @After
    public void restore() {
        Constants.target = Constants.Target.ROBOT;
        TeleOpDrive.useForesight = true;
    }

    @Test
    public void foresightIsUsedWithTheTunedSimSet() {
        Constants.target = Constants.Target.SIM;
        TeleOpDrive.useForesight = true;
        assertTrue(new TeleOpDrive(mock(Follower.class)).usingForesight());
    }

    @Test
    public void foresightIsNotUsedWithTheUntunedRobotSet() {
        Constants.target = Constants.Target.ROBOT;
        TeleOpDrive.useForesight = true;
        assertFalse(new TeleOpDrive(mock(Follower.class)).usingForesight());
    }

    @Test
    public void foresightStaysOffWhenSwitchedOffEvenIfTuned() {
        Constants.target = Constants.Target.SIM;
        TeleOpDrive.useForesight = false;
        assertFalse(new TeleOpDrive(mock(Follower.class)).usingForesight());
    }
}
