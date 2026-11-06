package org.firstinspires.ftc.teamcode.launch;

import org.firstinspires.ftc.teamcode.leds.RobotState;

/** How good the robot's position is for scoring both a Pollen and a Nectar in our CELL (ScoringPosition). */
public enum ShotQuality {
    /** Over ScoringPosition.likelyChance (90%) that both score. */
    LIKELY(RobotState.GOOD_SHOT),
    /** Over ScoringPosition.marginalChance (50%), up to the likely chance. */
    MARGINAL(RobotState.MARGINAL_SHOT),
    /** Up to the marginal chance, or a position the robot can't be in. */
    NONE(RobotState.NO_SHOT);

    /** What the LEDs show for it. */
    public final RobotState ledState;

    ShotQuality(RobotState ledState) {
        this.ledState = ledState;
    }
}
