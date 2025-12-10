package org.firstinspires.ftc.teamcode.pedroPathing.teleop;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

public class Values {
    public Modes mode = Modes.INTAKING;
    public static  double HOOD_FAR = 0;
    public static  double HOOD_CLOSE = 0;

    public static class flywheel_Values {
        public static PIDFController flywheelPIDController = new PIDFController(0, 0, 0, 0);
        public static double fP = 0;
        public static double fI = 0;
        public static double fD = 0;

        public static double fF = 0;
        public double flyWheelVelocity = 1500;
    }
    public static class transfer_Values {
        public static PIDFController transferPIDController = new PIDFController(0, 0, 0, 0);

        public static double trP = 0;
        public static double trI = 0;
        public static double trD = 0;
        public static double trF = 0;

        public static double transferTarget = 0;
    }
    public static class hood_Values {
        public static PIDFController hoodPIDController = new PIDFController(0, 0, 0, 0);
        public static double hP = 0;
        public static double hI = 0;
        public static double hD = 0;
        public static double hF = 0;

        public static double hoodTarget = 0;
    }

    public static class intake_Values {
        public static PIDFController intakePIDController = new PIDFController(0, 0, 0, 0);
        public static double iP = 0;
        public static double iI = 0;
        public static double iD = 0;

        public static double iK = 0;

        public static double iV = 0;

        public static double iA = 0;
        public static double intakeTarget = 0;
    }
    public static class turret_Values {
        public static ProfiledPIDController turretPIDController =  new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0,0));
        public static double tP = 0;
        public static double tI = 0;
        public static double tD = 0;
        public static double tA = 0;
        public static double tV = 0;
        public static double turretTarget = 0;
    }


    public static final double TURRET_RIGHT = 0;

    public enum Modes {
        INTAKING,
        SHOOTING,
        REST
    }
}
