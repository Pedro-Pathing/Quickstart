package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

/**
 * Constants as the robot and the BIOBUZZ sim need it. The sim's vhub/sim-overrides.txt sets these
 * public static fields by name through reflection, so renaming or hiding one breaks the sim
 * without a compile error anywhere: these tests catch that instead.
 */
public class ConstantsTest {
    @Test
    public void theFieldsTheSimOverridesArePublicStaticAndNotFinal() throws Exception {
        for (String name : new String[] {"drivetrainConfig", "localizerConfig", "foresightConfig", "foresightTuned"}) {
            Field field = Constants.class.getField(name);
            int modifiers = field.getModifiers();
            assertEquals(name + " must be static", true, Modifier.isStatic(modifiers));
            assertFalse(name + " must not be final, so the sim can set it", Modifier.isFinal(modifiers));
            assertNotNull(name, field.get(null));
        }
    }

    @Test
    public void foresightIsNotMarkedTunedForTheRobotYet() {
        assertFalse(Constants.foresightTuned);
    }

    @Test
    public void thePinpointIsNamedSoTheFollowerCanBeBuilt() {
        // Pedro has no default for the name and throws when it's read unset, which disables Drive
        assertEquals("pinpoint", Constants.localizerConfig.name.get());
    }

    @Test
    public void theDrivetrainUsesTheRobotsMotorNames() {
        assertEquals("MotorA", Constants.drivetrainConfig.motorAName.get());
        assertEquals("MotorB", Constants.drivetrainConfig.motorBName.get());
        assertEquals("MotorC", Constants.drivetrainConfig.motorCName.get());
    }
}
