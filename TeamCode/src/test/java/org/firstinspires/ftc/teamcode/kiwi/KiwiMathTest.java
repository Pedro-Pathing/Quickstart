package org.firstinspires.ftc.teamcode.kiwi;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.drivetrain.DrivePowers;

import org.junit.Test;

/** The Kiwi wheel-power maths, which needs no hardware. */
public class KiwiMathTest {
    private static final double EPS = 1e-9;
    private static final double SIN_60 = Math.sqrt(3) / 2.0;

    @Test
    public void pureForwardDrivesBAndCOppositely() {
        double[] w = Kiwi.computeWheelPowersUnnormalized(new DrivePowers(1, 0, 0));
        assertArrayEquals(new double[] {0, -SIN_60, SIN_60}, w, EPS);
    }

    @Test
    public void pureStrafeDrivesAAgainstBAndC() {
        double[] w = Kiwi.computeWheelPowersUnnormalized(new DrivePowers(0, 1, 0));
        assertArrayEquals(new double[] {-1, 0.5, 0.5}, w, EPS);
    }

    @Test
    public void pureTurnDrivesAllWheelsTheSameWay() {
        double[] w = Kiwi.computeWheelPowersUnnormalized(new DrivePowers(0, 0, 1));
        assertArrayEquals(new double[] {1, 1, 1}, w, EPS);
    }

    @Test
    public void maxScalingIsOneWhenTheStepStaysInRange() {
        assertEquals(1.0, Kiwi.scalingFor(DrivePowers.zero(), new DrivePowers(0.5, 0, 0)), EPS);
    }

    @Test
    public void maxScalingStopsAtTheFirstWheelReachingFullPower() {
        // Forward 2 would put wheel C at 2 * sin 60, so it reaches 1 a fraction 1 / (2 sin 60) of the way
        double lambda = Kiwi.scalingFor(DrivePowers.zero(), new DrivePowers(2, 0, 0));
        assertEquals(1.0 / (2 * SIN_60), lambda, EPS);
    }

    @Test
    public void maxScalingAccountsForTheCurrentPowers() {
        // Turning at 0.5 already, so a turn step of 1 gets to full power half way
        double lambda = Kiwi.scalingFor(new DrivePowers(0, 0, 0.5), new DrivePowers(0, 0, 1));
        assertEquals(0.5, lambda, EPS);
    }

    @Test
    public void pathFollowingScalesAllWheelsTogetherKeepingTheDirection() {
        double[] out = new double[3];
        double[] raw = Kiwi.computeWheelPowersUnnormalized(new DrivePowers(1, 1, 1));
        double scale = Kiwi.scaleWheelPowers(new DrivePowers(1, 1, 1), 1.0, false, out);
        double max = Math.max(Math.abs(raw[0]), Math.max(Math.abs(raw[1]), Math.abs(raw[2])));
        assertTrue(max > 1);
        assertEquals(1.0 / max, scale, EPS);
        for (int i = 0; i < 3; i++) {
            assertEquals(raw[i] * scale, out[i], EPS);
            assertTrue(Math.abs(out[i]) <= 1.0 + EPS);
        }
    }

    @Test
    public void powersInRangeAreNotScaled() {
        double[] out = new double[3];
        double scale = Kiwi.scaleWheelPowers(new DrivePowers(0.3, 0, 0.2), 1.0, false, out);
        assertEquals(1.0, scale, EPS);
        assertArrayEquals(Kiwi.computeWheelPowersUnnormalized(new DrivePowers(0.3, 0, 0.2)), out, EPS);
    }

    @Test
    public void turnFirstKeepsTheTurnAndScalesOnlyTheTranslation() {
        double[] out = new double[3];
        double scale = Kiwi.scaleWheelPowers(new DrivePowers(1, 0, 0.5), 1.0, true, out);
        // Wheel C is the limit: SIN_60 * forward * scale + 0.5 = 1
        assertEquals(0.5 / SIN_60, scale, EPS);
        assertEquals(1.0, out[2], EPS);
        // Wheel A has no translation here, so it carries exactly the turn
        assertEquals(0.5, out[0], EPS);
        assertEquals(-SIN_60 * scale + 0.5, out[1], EPS);
    }

    @Test
    public void turnFirstWithFullTurnDropsTheTranslationButKeepsTheTurn() {
        double[] out = new double[3];
        double scale = Kiwi.scaleWheelPowers(new DrivePowers(1, 0.5, 1), 1.0, true, out);
        assertTrue(scale < 1.0);
        for (double w : out) {
            assertTrue(Math.abs(w) <= 1.0 + EPS);
        }
    }

    @Test
    public void turnFirstStaysWithinFullPowerForAnyInput() {
        double[] out = new double[3];
        for (double f = -1; f <= 1; f += 0.25) {
            for (double s = -1; s <= 1; s += 0.25) {
                for (double t = -1; t <= 1; t += 0.25) {
                    Kiwi.scaleWheelPowers(new DrivePowers(f, s, t), 1.0, true, out);
                    for (double w : out) {
                        assertTrue("f=" + f + " s=" + s + " t=" + t, Math.abs(w) <= 1.0 + EPS);
                    }
                }
            }
        }
    }

    @Test
    public void turnFirstAppliesVoltageCompensationBeforeScaling() {
        double[] out = new double[3];
        Kiwi.scaleWheelPowers(new DrivePowers(0, 0, 0.5), 1.5, true, out);
        assertArrayEquals(new double[] {0.75, 0.75, 0.75}, out, EPS);
    }
}
