package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Unit tests for the Betaflight-style "Actual Rates" curve.
 *
 * These run on the local JVM -- no phone, no emulator, no Robolectric -- because
 * Rates imports nothing but java.lang.Math. Run them with:
 *
 *     ./gradlew :TeamCode:testDebugUnitTest
 *
 * The curve maps a stick position in [-1, 1] to a motor command in [-1, 1] and is
 * defined by three properties, which the first group of tests pins down:
 *
 *   1. center stick produces no output
 *   2. full stick produces exactly maxRate
 *   3. the slope of the curve at center stick is exactly centerRate
 *
 * Property 3 is what makes the three knobs independent of each other, which is the
 * whole reason to prefer Actual Rates over the older Betaflight rate system.
 */
public class RatesTest {

    /** Tolerance for exact algebraic identities. */
    private static final double EPS = 1e-9;

    /** A representative profile used by most tests. */
    private static Rates profile() {
        return new Rates(0.35, 1.0, 0.6, 0.0);
    }

    // ---------------------------------------------------------------- //
    // The three defining properties                                     //
    // ---------------------------------------------------------------- //

    @Test
    public void centerStickProducesNoOutput() {
        assertEquals(0.0, profile().apply(0.0), EPS);
    }

    @Test
    public void fullForwardStickIsExactlyMaxRate() {
        assertEquals(1.0, profile().apply(1.0), EPS);
    }

    @Test
    public void fullReverseStickIsExactlyNegativeMaxRate() {
        assertEquals(-1.0, profile().apply(-1.0), EPS);
    }

    @Test
    public void slopeAtCenterEqualsCenterRate() {
        Rates r = profile();
        double h = 1e-6;
        double slope = (r.apply(h) - r.apply(0.0)) / h;
        assertEquals(0.35, slope, 1e-4);
    }

    // ---------------------------------------------------------------- //
    // Sign handling                                                     //
    // ---------------------------------------------------------------- //

    @Test
    public void reverseStickProducesReverseOutput() {
        Rates r = profile();
        for (double x = -1.0; x <= -0.02; x += 0.01) {
            assertTrue("stick " + x + " produced non-negative output " + r.apply(x),
                    r.apply(x) < 0.0);
        }
    }

    @Test
    public void curveIsOddSymmetric() {
        Rates r = profile();
        for (double x = 0.0; x <= 1.0; x += 0.01) {
            assertEquals("asymmetry at stick " + x,
                    -r.apply(x), r.apply(-x), EPS);
        }
    }

    // ---------------------------------------------------------------- //
    // Shape invariants across the whole stick range                     //
    // ---------------------------------------------------------------- //

    @Test
    public void curveIsMonotonic() {
        Rates r = profile();
        double previous = r.apply(-1.0);
        for (double x = -1.0; x <= 1.0; x += 0.001) {
            double out = r.apply(x);
            assertTrue("output decreased at stick " + x, out >= previous - EPS);
            previous = out;
        }
    }

    @Test
    public void outputNeverExceedsMaxRate() {
        Rates r = profile();
        for (double x = -1.0; x <= 1.0; x += 0.001) {
            assertTrue("stick " + x + " commanded " + r.apply(x),
                    Math.abs(r.apply(x)) <= 1.0 + EPS);
        }
    }

    @Test
    public void degeneratesToIdentityWhenCenterEqualsMaxAndNoExpo() {
        // With no gap between center and max, and no expo, the curve is a straight
        // line. This is the one case that can be checked entirely by hand.
        Rates r = new Rates(1.0, 1.0, 0.0, 0.0);
        for (double x = -1.0; x <= 1.0; x += 0.05) {
            assertEquals("not linear at stick " + x, x, r.apply(x), EPS);
        }
    }

    // ---------------------------------------------------------------- //
    // What the two tuning knobs actually do                             //
    // ---------------------------------------------------------------- //

    @Test
    public void moreExpoSoftensMidStick() {
        double soft = new Rates(0.35, 1.0, 0.8, 0.0).apply(0.5);
        double firm = new Rates(0.35, 1.0, 0.2, 0.0).apply(0.5);
        assertTrue("more expo must mean less output at mid stick", soft < firm);
    }

    @Test
    public void moreCenterRateSharpensSmallMovements() {
        double gentle = new Rates(0.20, 1.0, 0.6, 0.0).apply(0.1);
        double eager = new Rates(0.50, 1.0, 0.6, 0.0).apply(0.1);
        assertTrue("more center rate must mean more output near center", eager > gentle);
    }

    @Test
    public void expoDoesNotChangeFullStickOutput() {
        // Expo reshapes the middle of the curve only; the endpoint stays pinned.
        for (double expo = 0.0; expo <= 1.0; expo += 0.1) {
            assertEquals("expo " + expo + " moved the endpoint",
                    1.0, new Rates(0.35, 1.0, expo, 0.0).apply(1.0), EPS);
        }
    }

    // ---------------------------------------------------------------- //
    // Deadzone                                                          //
    // ---------------------------------------------------------------- //

    @Test
    public void insideDeadzoneProducesNoOutput() {
        Rates r = new Rates(0.35, 1.0, 0.6, 0.05);
        assertEquals(0.0, r.apply(0.049), EPS);
        assertEquals(0.0, r.apply(-0.049), EPS);
    }

    @Test
    public void leavingDeadzoneIsContinuous() {
        // A deadzone that zeroes below the threshold without rescaling above it
        // produces a step change the driver feels as a jolt.
        Rates r = new Rates(0.35, 1.0, 0.6, 0.05);
        assertTrue("output jumped on leaving the deadzone",
                Math.abs(r.apply(0.051)) < 0.01);
        assertTrue("output jumped on leaving the deadzone (reverse)",
                Math.abs(r.apply(-0.051)) < 0.01);
    }

    @Test
    public void deadzoneDoesNotReduceFullStickOutput() {
        // The rescale divisor is what keeps full stick reaching full output.
        Rates r = new Rates(0.35, 1.0, 0.6, 0.05);
        assertEquals(1.0, r.apply(1.0), EPS);
        assertEquals(-1.0, r.apply(-1.0), EPS);
    }

    // ---------------------------------------------------------------- //
    // Input and parameter guards                                        //
    // ---------------------------------------------------------------- //

    @Test(expected = IllegalArgumentException.class)
    public void rejectsCenterRateAboveMaxRate() {
        // Otherwise the curve overshoots maxRate at full stick.
        new Rates(1.5, 1.0, 0.6, 0.0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsOutOfRangeStickInput() {
        // Nothing should be able to command more than maxRate, even if a caller
        // hands us a badly scaled value.
        profile().apply(1.4);
    }

    // ---------------------------------------------------------------- //
    // Characterization of the profiles MainTeleOp actually drives with  //
    // ---------------------------------------------------------------- //

    /**
     * Golden values for the tuned profiles. These are not derived from the
     * implementation -- they are the reference Actual Rates curve. If someone
     * changes the formula these fail; if someone deliberately retunes the
     * profile in MainTeleOp, update these to match.
     *
     * This table is also directly comparable to the curve drawn by the
     * Betaflight configurator for the same three values.
     */
    @Test
    public void moveRateProfileMatchesReferenceCurve() {
        Rates moveRate = new Rates(0.7, 1.0, 0.4, 0.01);
        double[] expected = {
                0.0000000000, 0.0651240347, 0.1409793665, 0.2205716833,
                0.3041399552, 0.3923243418, 0.4864782117, 0.5890719320,
                0.7041884285, 0.8381105161, 1.0000000000,
        };
        for (int i = 0; i < expected.length; i++) {
            double stick = i / 10.0;
            assertEquals("moveRate at stick " + stick,
                    expected[i], moveRate.apply(stick), 1e-9);
        }
    }

    @Test
    public void turnRateProfileMatchesReferenceCurve() {
        Rates turnRate = new Rates(0.375, 1.0, 0.4, 0.01);
        double[] expected = {
                0.0000000000, 0.0371902238, 0.0857945556, 0.1421842728,
                0.2068572298, 0.2811470926, 0.3678733788, 0.4721826866,
                0.6025811115, 0.7721578513, 1.0000000000,
        };
        for (int i = 0; i < expected.length; i++) {
            double stick = i / 10.0;
            assertEquals("turnRate at stick " + stick,
                    expected[i], turnRate.apply(stick), 1e-9);
        }
    }

    @Test
    public void turnRateIsGentlerThanMoveRateNearCenter() {
        // The intent documented in MainTeleOp: turning should be less twitchy
        // than translating for the same small stick movement.
        Rates moveRate = new Rates(0.7, 1.0, 0.4, 0.01);
        Rates turnRate = new Rates(0.375, 1.0, 0.4, 0.01);
        assertTrue(turnRate.apply(0.2) < moveRate.apply(0.2));
    }
}
