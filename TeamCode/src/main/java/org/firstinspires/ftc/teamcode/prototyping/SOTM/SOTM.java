package org.firstinspires.ftc.teamcode.prototyping.SOTM;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.prototyping.LaunchAngleCalculator;

@Configurable
public class SOTM {
    public LaunchAngleCalculator angleCalculator = new LaunchAngleCalculator();
    InterpLUT timeTable = new InterpLUT();
    public static double accelCompensation = 0.2;
    public double dist, airtime, virtualGoalX, virtualGoalY, newdist, angle;
    public void init() {
        initLUT();
    }


    public void calculateSOTM(Pose botPose, Translation2d goalPose, Vector2d botVel, Vector2d botAccel,
                              double shooterVel, TelemetryManager telemetryM) {
        Translation2d botPose2d = new Translation2d(botPose.getX(),botPose.getY());
        Translation2d botToGoal = goalPose.minus(botPose2d);
        dist = botToGoal.getDistance(new Translation2d(0,0));
        airtime = timeTable.get(dist);
        virtualGoalX = goalPose.getX()-airtime*(botVel.getX()+botAccel.getX()*accelCompensation);
        virtualGoalY = goalPose.getY()-airtime*(botVel.getY()+botAccel.getY()*accelCompensation);
        Translation2d movingGoalLocation = new Translation2d(virtualGoalX,virtualGoalY);
        newdist = movingGoalLocation.getDistance(botPose2d);

        angle = angleCalculator.calcBestAngle(shooterVel, newdist, telemetryM);

    }

    public void initLUT() {
        timeTable.add(0,.9);
        /*timeTable.add(40,1);
        timeTable.add(50,1);
        timeTable.add(60,1);
        timeTable.add(70,1);
        timeTable.add(80,1);
        timeTable.add(90,1);
        timeTable.add(100,1);
        timeTable.add(110,1);
        timeTable.add(120,1);*/
        timeTable.add(10000,1.5);
        timeTable.createLUT();
    }




}
