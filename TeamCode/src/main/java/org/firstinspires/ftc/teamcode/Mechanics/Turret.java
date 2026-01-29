package org.firstinspires.ftc.teamcode.Mechanics;

import static org.firstinspires.ftc.teamcode.Mechanics.Robot.turret;
import static org.firstinspires.ftc.teamcode.Testing.TurretPID.D;
import static org.firstinspires.ftc.teamcode.Testing.TurretPID.F;
import static org.firstinspires.ftc.teamcode.Testing.TurretPID.I;
import static org.firstinspires.ftc.teamcode.Testing.TurretPID.P;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.CPR1150;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class Turret {
    public static PIDFController awesome = new PIDFController(new PIDFCoefficients(P,I,D,F));
    public static double faceGoal(double x, double y, double theta, boolean red, double offset) {
        //x and y are in inches bc pedropathing is dumb
        double dx = (144) * (red ? 1 : 0) - x;
        double dy = 144 - y;

        double dtheta = Math.atan2(dy,dx) + offset;
        dtheta -= theta;
        dtheta = dtheta % (2 * Math.PI);

        if (dtheta < -Math.toRadians(190)) {
            dtheta += 2 * Math.PI;
        }
        else if (dtheta > Math.PI) {
            dtheta -= 2 * Math.PI;
        }

        int target = (int)Math.round((dtheta / (2 * Math.PI) * 5 * CPR1150));

        goTo(target);


        return dtheta;
    }
    public static double faceGoal(double x, double y, double theta, boolean red, double offset, boolean justAngle) {
        //x and y are in inches bc pedropathing is dumb
        double dx = (144) * (red ? 1 : 0) - x;
        double dy = 144 - y;

        double dtheta = Math.atan2(dy,dx) + offset;
        dtheta -= theta;
        dtheta = dtheta % (2 * Math.PI);

        if (dtheta < -Math.toRadians(190)) {
            dtheta += 2 * Math.PI;
        }
        else if (dtheta > Math.PI) {
            dtheta -= 2 * Math.PI;
        }


        return dtheta;
    }
    private static void goTo(double target) {
        awesome.updatePosition(turret.getCurrentPosition());
        awesome.setTargetPosition(target);
        turret.setPower(awesome.run());
        if (Math.abs(turret.getCurrentPosition() - target) < 10)
            turret.setPower(0);
    }
    public static void turretToPosition(double target){
        goTo(target);
    }
    public static double faceMotif(double x, double y,double theta){
        double dx = 72-x;
        double dy = 144-y;

        double dtheta = Math.atan2(dy,dx);
        dtheta -= theta;
        dtheta = dtheta % (2 * Math.PI);

        if (dtheta < -Math.toRadians(190)) {
            dtheta += 2 * Math.PI;
        }
        else if (dtheta > Math.PI) {
            dtheta -= 2 * Math.PI;
        }
        int target = (int)Math.round((dtheta / (2 * Math.PI) * 5 * CPR1150));

        turret.setTargetPosition(target);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
        return dtheta;
    }
}
