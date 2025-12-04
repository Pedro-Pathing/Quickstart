package org.firstinspires.ftc.teamcode.pedroPathing;

public class Methods {
    public Modes mode = Modes.INTAKING;
    public static final double HOOD_FAR = 0;
    public static final double HOOD_CLOSE = 0;

    public static class FlyWheel_Values{
        static double fv = 0;
        static double fd = 0;
        static double fp = 0;
    }

    public static class Intake_Values{
        static double iv = 0;
        static double id = 0;
        static double ip = 0;
    }

    public static class Hood_Values{
        static double hv = 0;
        static double hd = 0;
        static double hp = 0;
    }
    public static final double TURRET_RIGHT = 0;

    public enum Modes {
        INTAKING,
        SHOOTING,
        REST
    }
}
/// intake: PID,
/// Hood: PID,
/// flywheel: PID
/// transfer: PID, autoaim turret
/// ll: relocalization