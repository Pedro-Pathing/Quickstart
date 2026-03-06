package org.firstinspires.ftc.teamcode.common;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.command.Sort;
import org.firstinspires.ftc.teamcode.common.enums.Color;
import org.firstinspires.ftc.teamcode.common.enums.Position;
import org.firstinspires.ftc.teamcode.common.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Lifter;
import org.firstinspires.ftc.teamcode.common.subsystems.Lights;
import org.firstinspires.ftc.teamcode.common.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.common.subsystems.Pins;
import org.firstinspires.ftc.teamcode.common.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.common.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.common.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
public class Robot {

    private long lastTime;

    private Pose goalPos;
    private final Pose redGoalPos = new Pose(133, 131, Math.toRadians(45));
    private final Pose blueGoalPos = new Pose(11, 131, Math.toRadians(135));
    private double turretTargetAngle = 0;

    public static Pose robotPos;

    public static Color color ;

    public static int motif;

    public static Position position;

    public static Boolean sort;

    public static Boolean threeBalls = false;

    public Follower follower;
    public Intake intake;
    public Shooter shooter;
    public Transfer transfer;

    public Limelight limelight;

    public Pins pins;
    public ColorSensors colorSensors;
    private final TimestampedAngleBacklog backlog = new TimestampedAngleBacklog();
    public Lights lights;
    public Turret turret;

    public Lifter lifter;






    public Robot(HardwareMap hardwareMap){

        double dt = System.currentTimeMillis() - lastTime;
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        pins = new Pins(hardwareMap);
        lifter = new Lifter(hardwareMap);
        colorSensors = new ColorSensors(hardwareMap);
        lights = new Lights(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);
        lastTime = System.currentTimeMillis();
    }

    public void update(){
        setGoalPose();
        robotPos = follower.getPose();
        backlog.addAngle(turret.getTurretAngle());
        limelight.update();
        follower.update();
        shooter.update(getShooterTargetVelocity(getDistance()));
        turret.update(getTurretTarget());
        if (sort){
            lights.setLights(colorSensors.getColors());
        } else {
            lights.setToDefault();
        }






    }
    private double getShooterTargetVelocity(double distance){

        double velocity =  -(0.000294289 * Math.pow(distance, 3)) + (0.111951 * Math.pow(distance, 2)) - (9.3151 * distance) + 1218.2967;


        if (velocity > 1500){
            return 1500;
        } else{
            return velocity;
        }

    }


    private double getDistance(){
        double distance;
        if (limelight.getDistance() != 0){
            distance = limelight.getDistance();
        } else {
            if (color == Color.BLUE){
                distance = robotPos.distanceFrom(blueGoalPos);
            } else {
                distance = robotPos.distanceFrom(redGoalPos);
            }
        }
        return distance;
    }

    private double getTurretTarget(){
        double targetAngle;
        if (limelight.limeLightResult()){
            double tx = limelight.getResult().getTx();
            double prevAngle= backlog.getAngle(limelight.getResult().getControlHubTimeStampNanos());
            targetAngle = prevAngle - tx;
        } else {
            double globalAngle=Math.atan2(goalPos.getY()-robotPos.getY(),goalPos.getX()-robotPos.getX());
            double targetAngleRad = globalAngle - robotPos.getHeading()-Math.PI;
            targetAngleRad =Math.atan2(Math.sin(targetAngleRad),Math.cos(targetAngleRad));
            targetAngle = Math.toDegrees(targetAngleRad);
        }

        return targetAngle;
    }
    public void setTurretAngle(double turretTargetAngle){
        this.turretTargetAngle = turretTargetAngle;
    }
    private void setGoalPose(){
        if (Robot.color == Color.RED){
            goalPos = redGoalPos;
        } else {
            goalPos = blueGoalPos;
        }
    }

    public boolean inFrontTriangle() {
        double px = robotPos.getX();
        double py = robotPos.getY();
        double ax = 0;
        double ay = 144;
        double bx = 72;
        double by = 72;
        double cx = 144;
        double cy = 144;
        double d1 = sign(px, py, ax, ay, bx, by);
        double d2 = sign(px, py, bx, by, cx, cy);
        double d3 = sign(px, py, cx, cy, ax, ay);

        boolean hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(hasNeg && hasPos) && getDistance() > 60;
    }

    public boolean inBackTriangle(){
        double px = robotPos.getX();
        double py = robotPos.getY();
        double ax = 30;
        double ay = 0;
        double bx = 72;
        double by = 33;
        double cx = 99;
        double cy = 0;
        double d1 = sign(px, py, ax, ay, bx, by);
        double d2 = sign(px, py, bx, by, cx, cy);
        double d3 = sign(px, py, cx, cy, ax, ay);
        boolean hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(hasNeg && hasPos);
    }


    private double sign(double px, double py,
                               double ax, double ay,
                               double bx, double by) {
        return (px - bx) * (ay - by) - (ax - bx) * (py - by);
    }

    public boolean ballsInIntake(){
        return intake.threeBalls() && colorSensors.ballsInIntake();
    }

}
