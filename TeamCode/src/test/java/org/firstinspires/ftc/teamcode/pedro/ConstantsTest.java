package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Test;

/** The SIM/ROBOT switch in Constants. */
public class ConstantsTest {
    @After
    public void restoreDefault() {
        Constants.target = Constants.Target.ROBOT;
    }

    @Test
    public void defaultsToTheRobot() {
        assertEquals(Constants.Target.ROBOT, Constants.target);
    }

    @Test
    public void simAndRobotSelectDifferentSets() {
        Constants.target = Constants.Target.SIM;
        Object simForesight = Constants.foresightConfig();
        Object simLocalizer = Constants.localizerConfig();
        Object simDrivetrain = Constants.drivetrainConfig();

        Constants.target = Constants.Target.ROBOT;
        assertNotSame(simForesight, Constants.foresightConfig());
        assertNotSame(simLocalizer, Constants.localizerConfig());
        assertNotSame(simDrivetrain, Constants.drivetrainConfig());

        Constants.target = Constants.Target.SIM;
        assertSame(simForesight, Constants.foresightConfig());
    }

    @Test
    public void bothSetsHaveTheSameDrivetrain() {
        Constants.target = Constants.Target.SIM;
        String sim = Constants.drivetrainConfig().motorAName.get() + Constants.drivetrainConfig().motorADirection.get();
        Constants.target = Constants.Target.ROBOT;
        String robot = Constants.drivetrainConfig().motorAName.get() + Constants.drivetrainConfig().motorADirection.get();
        assertEquals(sim, robot);
    }

    @Test
    public void simForesightIsTunedAndRobotIsNotYet() {
        Constants.target = Constants.Target.SIM;
        assertTrue(Constants.foresightTuned());
        Constants.target = Constants.Target.ROBOT;
        assertFalse(Constants.foresightTuned());
    }
}
