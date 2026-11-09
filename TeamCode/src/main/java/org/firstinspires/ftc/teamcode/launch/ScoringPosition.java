package org.firstinspires.ftc.teamcode.launch;

import org.firstinspires.ftc.teamcode.vision.Alliance;

/**
 * Is the robot's position a scoring one? From where the robot's centre is, ignoring which way it faces (auto-aim
 * turns it), the chance that a Pollen and a Nectar shot both enter our alliance's CELL on this half of the field,
 * assuming the HIVE is tipped so that CELL faces up. Graded as ShotQuality: LIKELY over likelyChance, MARGINAL over
 * marginalChance, otherwise NONE.
 *
 * The chances come from tables the BIOBUZZ sim works out offline (ScoreTableData): for each point of a 3 in grid,
 * many shots at the RPM RpmTable picks, each with a random speed, launch angle and direction error like the real
 * launchers', flown with the sim's full flight model (including shots that clip the HIVE). Here it's a table lookup,
 * interpolated between grid points, so it's cheap enough for every loop. The two launchers' errors are independent,
 * so the chance both score is the product of each one's.
 *
 * Positions are Pedro's field frame, inches: (72, 72) is the field centre, +Y towards the scoring side, so y below
 * 72 is the audience half. The tables are for red; blue's field is red's turned 180 degrees, so a blue position is
 * turned round before the lookup.
 *
 * Call update() once per loop for a grade that doesn't flicker at the boundaries: it has to get hysteresis past a
 * threshold to move up a grade, and the same below it to move down.
 */
public class ScoringPosition {
    public static double likelyChance = 0.90;
    public static double marginalChance = 0.50;
    /** Margin either side of a threshold before update() changes grade. */
    public static double hysteresis = 0.03;

    public enum Half { AUDIENCE, SCORING }

    private static final double FIELD = 144.0;
    private static final double MID = 72.0;
    private static final int UNREACHABLE = 0xff;

    private static final int[] AUDIENCE_POLLEN = decode(ScoreTableData.RED_AUDIENCE_POLLEN);
    private static final int[] AUDIENCE_NECTAR = decode(ScoreTableData.RED_AUDIENCE_NECTAR);
    private static final int[] SCORING_POLLEN = decode(ScoreTableData.RED_SCORING_POLLEN);
    private static final int[] SCORING_NECTAR = decode(ScoreTableData.RED_SCORING_NECTAR);

    private ShotQuality quality = ShotQuality.NONE;
    private double chance;

    /** Which half of the field the robot's centre is in. */
    public static Half halfOf(double y) {
        return y < MID ? Half.AUDIENCE : Half.SCORING;
    }

    /** Chance (0..1) a shot from this launcher enters our CELL on this half, from the robot centred at (x, y). */
    public static double chance(Alliance alliance, LaunchModel.Launcher launcher, double x, double y) {
        if (alliance == Alliance.BLUE) {
            x = FIELD - x;
            y = FIELD - y;
        }
        boolean audience = halfOf(y) == Half.AUDIENCE;
        int[] table;
        if (launcher == LaunchModel.Launcher.POLLEN) {
            table = audience ? AUDIENCE_POLLEN : SCORING_POLLEN;
        } else {
            table = audience ? AUDIENCE_NECTAR : SCORING_NECTAR;
        }
        return lookup(table, x, audience ? y : y - MID) / 100.0;
    }

    /** Chance (0..1) both a Pollen and a Nectar shot score from the robot centred at (x, y). */
    public static double chanceBoth(Alliance alliance, double x, double y) {
        return chance(alliance, LaunchModel.Launcher.POLLEN, x, y) * chance(alliance, LaunchModel.Launcher.NECTAR, x, y);
    }

    /** The grade for a chance, with no hysteresis. */
    public static ShotQuality grade(double chanceBoth) {
        if (chanceBoth > likelyChance) return ShotQuality.LIKELY;
        if (chanceBoth > marginalChance) return ShotQuality.MARGINAL;
        return ShotQuality.NONE;
    }

    /** Call once per loop with the robot's pose (Pedro, inches). Returns the grade, with hysteresis. */
    public ShotQuality update(Alliance alliance, double x, double y) {
        chance = chanceBoth(alliance, x, y);
        ShotQuality raw = grade(chance);
        if (raw.ordinal() < quality.ordinal()) {
            // Better: only once it's clear of the threshold it crossed
            if (grade(chance - hysteresis).ordinal() < quality.ordinal()) quality = grade(chance - hysteresis);
        } else if (raw.ordinal() > quality.ordinal()) {
            // Worse: only once it's clearly below the threshold
            if (grade(chance + hysteresis).ordinal() > quality.ordinal()) quality = grade(chance + hysteresis);
        }
        return quality;
    }

    /** The grade from the last update(). */
    public ShotQuality quality() {
        return quality;
    }

    /** The chance both score from the last update(). */
    public double lastChance() {
        return chance;
    }

    /**
     * Percent from a table, interpolated between the four grid points round (x, yInHalf), using only those where
     * the robot can be (so a robot hard against the wall, between a row it can't reach and one it can, gets the
     * reachable row's chance). 0 if it can't be at any of them. Clamped at the grid's edges.
     */
    private static double lookup(int[] table, double x, double yInHalf) {
        double gx = clamp(x / ScoreTableData.STEP_IN, 0, ScoreTableData.COLUMNS - 1);
        double gy = clamp(yInHalf / ScoreTableData.STEP_IN, 0, ScoreTableData.ROWS - 1);
        int c = Math.min((int) gx, ScoreTableData.COLUMNS - 2);
        int r = Math.min((int) gy, ScoreTableData.ROWS - 2);
        double fx = gx - c, fy = gy - r;
        double sum = 0, weight = 0;
        for (int k = 0; k < 4; k++) {
            int dc = k & 1, dr = k >> 1;  // corners (c, r), (c + 1, r), (c, r + 1), (c + 1, r + 1)
            int v = table[(r + dr) * ScoreTableData.COLUMNS + c + dc];
            if (v == UNREACHABLE) continue;
            double w = (dc == 1 ? fx : 1 - fx) * (dr == 1 ? fy : 1 - fy);
            sum += w * v;
            weight += w;
        }
        return weight > 1e-9 ? sum / weight : 0.0;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static int[] decode(String hex) {
        int[] out = new int[hex.length() / 2];
        for (int i = 0; i < out.length; i++) out[i] = Integer.parseInt(hex.substring(2 * i, 2 * i + 2), 16);
        return out;
    }
}
