package org.firstinspires.ftc.teamcode.pedroPathing;

import com.arcrobotics.ftclib.controller.PIDFController;

public class Values {
    public Modes mode = Modes.INTAKING;
    public static  double HOOD_FAR = 0;
    public static  double HOOD_CLOSE = 0;

    public static class flywheel_Values{
        public static PIDFController flywheelPIDController = new PIDFController(0, 0, 0, 0);
        public static double fp = 0;
        public static double fi = 0;
        public static double fd = 0;

        public static double ff = 0;
        public double flyWheelVelocity = 1500;
    }

    public static final double TURRET_RIGHT = 0;

    public enum Modes {
        INTAKING,
        SHOOTING,
        REST
    }
}
