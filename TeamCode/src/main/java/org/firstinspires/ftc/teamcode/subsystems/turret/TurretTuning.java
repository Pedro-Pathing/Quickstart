package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.acmerobotics.dashboard.config.Config;

@Config("Turret Tuning")
public class TurretTuning {
    public static boolean AUTO_AIM = false;
    public static double TARGET_DEG = 0;
    public static double STEP_DEG = 45;

    public static double kP = TurretConstants.kP;
    public static double kI = TurretConstants.kI;
    public static double kD = TurretConstants.kD;
    public static double kS = TurretConstants.kS;
    public static double AIM_OFFSET_DEG = TurretConstants.AIM_OFFSET_DEG;

    public static double START_X_IN = 133;
    public static double START_Y_IN = 132;
    public static double START_HEADING_DEG = -90;

    public static void apply() {
        TurretConstants.kP = kP;
        TurretConstants.kI = kI;
        TurretConstants.kD = kD;
        TurretConstants.kS = kS;
        TurretConstants.AIM_OFFSET_DEG = AIM_OFFSET_DEG;
    }
}
