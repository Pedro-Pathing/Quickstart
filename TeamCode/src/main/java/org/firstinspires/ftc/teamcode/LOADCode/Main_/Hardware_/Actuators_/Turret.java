package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;


public class Turret {


    public final Devices.DcMotorExClass rotation = new Devices.DcMotorExClass();
    public final Devices.DcMotorExClass flywheel = new Devices.DcMotorExClass();
    public final Devices.ServoClass hood = new Devices.ServoClass();
    public final Devices.ServoClass gate = new Devices.ServoClass();

    public static PIDCoefficients turretCoefficients = new PIDCoefficients(0.002, 0, 0);
    public static PIDCoefficients flywheelCoefficients = new PIDCoefficients(0, 0, 0);
    public static BasicFeedforwardParameters ffCoefficients = new BasicFeedforwardParameters(0,0,0);

    public enum gatestate {
        OPEN,
        CLOSED,
    }

    public void init(OpMode opmode){
        rotation.init(opmode, "turret", 145.1); //Previously 103.8
        flywheel.init(opmode, "flywheel");
        hood.init(opmode, "hood");
        gate.init(opmode, "gate");

        rotation.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

        gate.setAngle(0.5);

        // Pass PID pidCoefficients to motor classes
        rotation.setPidCoefficients(turretCoefficients);
        flywheel.setPidCoefficients(flywheelCoefficients);
        flywheel.setFFCoefficients(ffCoefficients);
    }

    public void updatePIDs(){
        // Pass PID pidCoefficients to motor classes
        rotation.setPidCoefficients(turretCoefficients);
        flywheel.setPidCoefficients(flywheelCoefficients);
        flywheel.setFFCoefficients(ffCoefficients);
    }

    /**
     * @param robotPose The pose of the robot, gotten from PedroPathing's localization
     * @param targetRedGoal Set this to true to target the red goal, otherwise targets the blue goal.
     */
    public void updateAimbot(Pose robotPose, boolean targetRedGoal){
        rotation.setAngle(calcLocalizer(robotPose, targetRedGoal));
    }

    public void setGate(gatestate state){
        if (state == gatestate.OPEN){
            gate.setAngle(0.5);
        }else if (state == gatestate.CLOSED){
            gate.setAngle(0.25);
        }
    }

    public double calcLocalizer (Pose robotPose, boolean targetRedGoal){
        Pose goalPose = new Pose(0,144,0);
        if (targetRedGoal) {goalPose = new Pose(144, 144, 0);}

        return Math.toDegrees(Math.atan2(
                goalPose.getY()-robotPose.getY(),
                goalPose.getX()-robotPose.getX())
        ) - Math.toDegrees(robotPose.getHeading()) + 180;
    }

    public void setFlywheelRPM(double rpm){
        flywheel.setRPM(rpm);
    }
}
