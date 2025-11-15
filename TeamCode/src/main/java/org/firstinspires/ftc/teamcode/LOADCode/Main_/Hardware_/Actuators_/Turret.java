package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Generic_.DcMotorExClass;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Generic_.ServoClass;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Calculation_.Turret_Heading;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

public class Turret {
    public final DcMotorExClass rotation = new DcMotorExClass();
    public final DcMotorExClass flywheel = new DcMotorExClass();
    public final ServoClass hood = new ServoClass();

    Turret_Heading targeting = new Turret_Heading();

    public static PIDCoefficients turretCoefficients = new PIDCoefficients(0.002, 0, 0);
    public static PIDCoefficients flywheelCoefficients = new PIDCoefficients(0, 0, 0);
    public static BasicFeedforwardParameters ffCoefficients = new BasicFeedforwardParameters(0,0,0);

    public void init(OpMode opmode){
        rotation.init(opmode, "turret", 103.8);
        flywheel.init(opmode, "flywheel");
        hood.init(opmode, "hood");

        rotation.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void updateAimbot(Pose robotPose, boolean goal){
        rotation.setAngle(targeting.calcLocalizer(robotPose, goal));
    }
}
