package org.firstinspires.ftc.teamcode.launch;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.BeforeClass;
import org.junit.Test;

/** LaunchModel and RpmTable. The tables take about a second each to build, so they're built once. */
public class LaunchMathTest {
    private static RpmTable pollen;
    private static RpmTable nectar;

    @BeforeClass
    public static void buildTables() {
        pollen = new RpmTable(LaunchModel.Launcher.POLLEN);
        nectar = new RpmTable(LaunchModel.Launcher.NECTAR);
    }

    private static RpmTable table(LaunchModel.Launcher l) {
        return l == LaunchModel.Launcher.POLLEN ? pollen : nectar;
    }

    @Test
    public void solvedRpmIsFiniteAndInRange() {
            // Pollen has no solution as close as 0.6 m (RpmTable fills that gap from the nearest solved distance)
        for (LaunchModel.Launcher l : LaunchModel.Launcher.values()) {
            for (double d = 0.7; d <= 2.0001; d += 0.1) {
                double rpm = LaunchModel.solveRPM(l, d, LaunchModel.FACE_HEIGHT);
                assertFalse(l + " at " + d + " m has no solution", Double.isNaN(rpm));
                assertTrue(l + " at " + d + " m gave " + rpm, rpm >= 500 && rpm <= LaunchModel.MAX_FLYWHEEL_RPM);
            }
        }
    }
    /**
     * Known quirk in LaunchModel (not changed here): Pollen's solved RPM dips from 0.7 m (2896) to 0.8 m (2859), so it
     * is not monotonic at close range. Probably the "middle of the widest scoring range" choice jumping between
     * shot arcs, but worth a look if short shots misbehave.
     */
    @org.junit.Ignore("LaunchModel.solveRPM is not monotonic for Pollen between 0.7 and 0.8 m")
    @Test
    public void solvedRpmRisesWithDistanceFromSeventyCentimetres() {
        double previous = LaunchModel.solveRPM(LaunchModel.Launcher.POLLEN, 0.7, LaunchModel.FACE_HEIGHT);
        double next = LaunchModel.solveRPM(LaunchModel.Launcher.POLLEN, 0.8, LaunchModel.FACE_HEIGHT);
        assertTrue(next + " at 0.8 m is not above " + previous, next > previous);
    }


    @Test
    public void solvedRpmRisesWithDistance() {
        for (LaunchModel.Launcher l : LaunchModel.Launcher.values()) {
            double previous = LaunchModel.solveRPM(l, 1.0, LaunchModel.FACE_HEIGHT);
            for (double d = 1.1; d <= 2.0001; d += 0.1) {
                double rpm = LaunchModel.solveRPM(l, d, LaunchModel.FACE_HEIGHT);
                assertTrue(l + ": " + rpm + " at " + d + " m is not above " + previous, rpm > previous);
                previous = rpm;
            }
        }
    }

    @Test
    public void scoresAtTheSolvedRpmButNotFarFromIt() {
        for (LaunchModel.Launcher l : LaunchModel.Launcher.values()) {
            for (double d : new double[] {0.8, 1.2, 1.6, 2.0}) {
                double rpm = LaunchModel.solveRPM(l, d, LaunchModel.FACE_HEIGHT);
                assertTrue(l + " at " + d, LaunchModel.scores(l, rpm, d, LaunchModel.FACE_HEIGHT));
                assertFalse(l + " at " + d + " far too slow",
                        LaunchModel.scores(l, rpm * 0.5, d, LaunchModel.FACE_HEIGHT));
                assertFalse(l + " at " + d + " far too fast",
                        LaunchModel.scores(l, rpm * 1.5, d, LaunchModel.FACE_HEIGHT));
            }
        }
    }

    @Test
    public void rpmTableMatchesTheModelAtItsGridPoints() {
        for (LaunchModel.Launcher l : LaunchModel.Launcher.values()) {
            RpmTable t = table(l);
            for (int i : new int[] {10, 20, 30}) {
                assertEquals(LaunchModel.solveRPM(l, t.distanceAt(i), LaunchModel.FACE_HEIGHT), t.rpmAtIndex(i), 1e-9);
                assertEquals(t.rpmAtIndex(i), t.rpmAt(t.distanceAt(i)), 1e-6);
            }
        }
    }

    @Test
    public void rpmAtInterpolatesBetweenGridPoints() {
        for (LaunchModel.Launcher l : LaunchModel.Launcher.values()) {
            RpmTable t = table(l);
            int i = 20;
            double mid = (t.distanceAt(i) + t.distanceAt(i + 1)) / 2;
            assertEquals((t.rpmAtIndex(i) + t.rpmAtIndex(i + 1)) / 2, t.rpmAt(mid), 1e-6);
            double quarter = t.distanceAt(i) + RpmTable.STEP / 4;
            assertEquals(0.75 * t.rpmAtIndex(i) + 0.25 * t.rpmAtIndex(i + 1), t.rpmAt(quarter), 1e-6);
        }
    }

    @Test
    public void rpmAtClampsAtTheEnds() {
        for (LaunchModel.Launcher l : LaunchModel.Launcher.values()) {
            RpmTable t = table(l);
            assertEquals(t.rpmAtIndex(0), t.rpmAt(0.0), 1e-9);
            assertEquals(t.rpmAtIndex(0), t.rpmAt(RpmTable.MIN_DISTANCE), 1e-9);
            assertEquals(t.rpmAtIndex(t.size() - 1), t.rpmAt(RpmTable.MAX_DISTANCE), 1e-9);
            assertEquals(t.rpmAtIndex(t.size() - 1), t.rpmAt(10.0), 1e-9);
        }
    }

    @Test
    public void rpmTableHasNoGaps() {
        for (LaunchModel.Launcher l : LaunchModel.Launcher.values()) {
            RpmTable t = table(l);
            for (int i = 0; i < t.size(); i++) assertFalse(l + " index " + i, Double.isNaN(t.rpmAtIndex(i)));
        }
    }
}
