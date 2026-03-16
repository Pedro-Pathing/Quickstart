package org.firstinspires.ftc.teamcode.Global;



public class Constants {

    /*
     * Hardware Constants Bellow
     */

    public final static class HardwareNames {
        public final static String FLYWHEEL_TOP = "smTop";
        public final static String FLYWHEEL_BOTTOM = "smBottom";
        public final static String TURRET = "turret";
        public final static String[] DRIVE_MOTORS = {"fl", "fr", "bl", "br"};
        public final static String INTAKE = "test";

        public final static String[] INTAKE_SERVOS = {"intakeR","intakeL"};
        public final static String INDEXER_SERVO = "indexer";

        public final static String LIMELIGHT = "LimeLight";

        public final static String IMU = "imu";

        public final static String LED = "led";
    }

    public final static class HardwareInitialization {
        public final static String IMUParametersJsonFileName = "BNO055IMUCalibration.json";

    }

    public final static class GamePad {
        public final static double TRIGGER_THRESHOLD = 0.5;
    }

    public enum Alliance {
        RED(0),
        BLUE(1);

        private Alliance(int value) {}
    }




}
