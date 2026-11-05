package org.firstinspires.ftc.teamcode.hardwareConfig.actuators.nectarShooter;


public class NectarShooterTBHControllerConstants {
    public double FRONT_Kp = 0.00005;  //to be tuned for the PID controller
    public double REAR_Kp = 0.00005;  //to be tuned for the PID controller
    public double FRONT_Kf_a = 0.08; // a,b,c represent the values in
    public double FRONT_Kf_b = 1.15E-04; // a + bx + cx^2
    public double FRONT_Kf_c = 1.03E-08; // in the feedforward equation
    public double REAR_Kf_a = 0.075;
    public double REAR_Kf_b = 1.28E-04;
    public double REAR_Kf_c = 8.51E-09;
}