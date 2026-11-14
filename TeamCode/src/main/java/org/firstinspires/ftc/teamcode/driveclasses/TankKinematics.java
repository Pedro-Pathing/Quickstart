package org.firstinspires.ftc.teamcode.driveclasses;

public class TankKinematics {

    public static double[] tankKinematics(double forward, double turn){
        return new double[]{
                Math.min(Math.max(forward - turn,-1),1), //right drive motor power
                Math.min(Math.max(forward + turn,-1),1) //left drive motor power
        };
    }
}