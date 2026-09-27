package org.firstinspires.ftc.teamcode.util;

public class Rates {

    private double centerRate;
    private double maxRate;
    private double expo;
    private double deadzone;

    public Rates(double centerRate, double maxRate, double expo, double deadzone) {
        if (centerRate > maxRate) {
            throw new IllegalArgumentException("Center rate must be less than max rate.");
        }

        this.centerRate = centerRate;
        this.maxRate = maxRate;
        this.expo = expo;
        this.deadzone = deadzone;
    }

    /**
     * apply Deadzone prevents robot twitches while the joystick is moving without intent.
     * @param stickPosition
     * @return
     */
    private double applyDeadzone(double stickPosition) {
        double magnitude = Math.abs(stickPosition);
        if (magnitude < deadzone) {
            return 0.0;
        }

        return Math.signum(stickPosition) * (magnitude - deadzone) / (1 - deadzone);
    }

    /**
     *
     * @param stickPosition A parameter read from a controller that should be in the domain {0..1}
     * @return A power value representative of what we want to hand to a Drive train / motors
     */
    public double apply(double stickPosition) {
        if (Math.abs(stickPosition) > 1) {
            throw new IllegalArgumentException("Stick Position cannot be more than 1 or less than -1");
        }

        stickPosition = applyDeadzone(stickPosition);
        double actualRatesExpoFactor =
                Math.abs(stickPosition) * (Math.pow(stickPosition, 5) * expo + stickPosition * (1-expo));

        return (centerRate * stickPosition) +
                ((maxRate - centerRate) * actualRatesExpoFactor);
    }
}
