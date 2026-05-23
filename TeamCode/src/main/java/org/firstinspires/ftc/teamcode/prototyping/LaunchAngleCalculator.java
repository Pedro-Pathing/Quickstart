package org.firstinspires.ftc.teamcode.prototyping;

import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class LaunchAngleCalculator {
    public static double hoodOffset = -1.5; //offsets hood angle because zeroing servo is hard
    InterpLUT launchVelocityLUT = new InterpLUT();

    InterpLUT angleLUT = new InterpLUT();
    //height is in meters
    static double height = 3 * 0.0254;
    static double minHoodAngle = angleChange(30);
    static double maxHoodAngle = angleChange(60);

    static double closeDistance = 55;
//    private void createAngleLUT() {
//        //for mapping angles (in degrees) to servo positions on the hood
//        angleLUT.add(0,0);
//        angleLUT.add(25, 60);
//    }

    private void createSpeedLUT() {
        launchVelocityLUT.add(1000,4.68);
        launchVelocityLUT.add(1025,4.82);
        launchVelocityLUT.add(1100,5.04);
        launchVelocityLUT.add(1175,5.26);
        launchVelocityLUT.add(1212,5.50);
        launchVelocityLUT.add(1250,5.72);
        launchVelocityLUT.add(1300,5.96);
        launchVelocityLUT.add(1400,6.40);
        launchVelocityLUT.add(1500,6.87);
        launchVelocityLUT.add(1600,7.16);
    }

    private static double estimateSpeed(double velocity) {
        return 0.0043 * velocity +0.35;
    }

    public void init(){
        //createAngleLUT();
        createSpeedLUT();
    }

    public double calcBestAngle(double velocity, double distance, TelemetryManager telemetryM) {

        boolean isClose = distance < closeDistance;


        //converts input values (inches) into meters.
        double d = distance * 0.0254;
        //double v = launchVelocityLUT.get(velocity);
        double v = estimateSpeed(velocity) * 0.85;

        //didn't change much here, just made it easier to read (It does work). distance is meters and angles in radians
        double sqrt = Math.sqrt(Math.pow(v,4) - (9.8 * ((9.8 * Math.pow(d, 2)) + (2 * height * Math.pow(v,2)))));
        double vel_1 = Math.pow(v,2);
        double divider = 9.8 * d;
        
        double lowAngle = Math.atan((vel_1 - sqrt)/divider);
        double highAngle = Math.atan((vel_1 + sqrt)/divider);

        //converts hood angle into degrees and then into servo positions
        lowAngle = Math.toDegrees(lowAngle);
        highAngle = Math.toDegrees(highAngle);

        telemetryM.addData("Low",lowAngle);
        telemetryM.addData("High",highAngle);
        telemetryM.addData("Speed",v);
        /*if (lowAngle > minHoodAngle && !isClose) {
            //return hoodToServoAngle(lowAngle);
            return angleChange(lowAngle);
        }
        else if (highAngle < maxHoodAngle && isClose) {
            //return hoodToServoAngle(highAngle);
            return angleChange(highAngle);
        }
        else */ if (lowAngle < minHoodAngle && highAngle > maxHoodAngle) {
            if (distance > 70) return  angleChange(maxHoodAngle);
            else return angleChange(minHoodAngle);
        } else if (highAngle > maxHoodAngle) {
            //return hoodToServoAngle(maxHoodAngle);
            return angleChange(maxHoodAngle);
        }
        else {
            //return hoodToServoAngle(40);
            return angleChange(lowAngle);
        }

    }
    public static double hoodToServoAngle(double hoodAngle) {
        return (hoodAngle-minHoodAngle - hoodOffset) * 121 / 13;
    }

    public static double angleChange(double angle) {
        return (90-angle);
    }
}
