package org.firstinspires.ftc.teamcode.driveclasses;

public class MecanumKinematics {

    public static double[] mecanumKinematics(double forward, double strafe, double turn) {
        return new double[]{
                (forward - strafe - turn),
                (forward + strafe + turn),
                (forward + strafe - turn),
                (forward - strafe + turn)
        };
    }
}