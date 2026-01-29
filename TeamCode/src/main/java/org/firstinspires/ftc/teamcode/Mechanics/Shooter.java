package org.firstinspires.ftc.teamcode.Mechanics;

import static org.firstinspires.ftc.teamcode.Mechanics.Robot.flywheel;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.hood;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    private static double velocity;


    //manual
    public static double getVel() {
        velocity = -flywheel.getVelocity(AngleUnit.RADIANS);
        return velocity;
    }

    public static void setPower(double power) {
        flywheel.setPower(power);
    }

    public static void setHood(double pos) {
        hood.setPosition(pos);
    }

    public static double getHood() {
        return hood.getPosition();
    }


    //Automation
    public static double autoShotHood(double x, double y, double theta, boolean red) {
        velocity = -flywheel.getVelocity(AngleUnit.RADIANS);
        double dist = Math.sqrt(Math.pow(x + 2.95 * Math.cos(theta), 2) + Math.pow(y + 2.95 * Math.sin(theta), 2));
        double hoodP = hoodVal(dist, velocity);
        hood.setPosition(hoodP);
        return hoodP;
    }

    private static double hoodVal(double dist, double velocity) {
        double ret = 0;
        if (dist < 52)
            return 1;
        else if (dist < 60) {
            double slope = .0876578 * dist - 4.38447;
            double y_inter = -0.380435 * dist + 19.68859;
            return slope * velocity + y_inter;
        }
        else if (dist < 75.85) {
            double slope = -.0251794 * dist + 2.01077;
            double y_inter = .0965159 * dist - 7.39096;
            return slope * velocity + y_inter;
        }
        else if (dist < 81.3) {
            double slope = .0099411 * dist - .653127;
            double y_inter = -.0192563 * dist + 1.39036;
            return slope * velocity + y_inter;
        }
        else if (dist < 88.9) {
            double slope = -.005055 * dist + .566056;
            double y_inter = .00353105 * dist - .462244;
            return slope * velocity + y_inter;
        }
        else if (dist < 102.12) {
            double slope = .00466059 * dist - .297659;
            double y_inter = -.0308132 * dist + 2.59096;
            return slope * velocity + y_inter;
        }
        else if (dist < 160) {
            double slope = -.000200639 * dist + .198769;
            double y_inter = -.00468182 * dist - .0775751;
            return slope * velocity + y_inter;
        }
        return 0;
    }
}
