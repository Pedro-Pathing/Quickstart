package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.Utility;

public class Turret implements Subsystem {

    //---------------- Hardware ----------------
    private final Servo turretLeft;
    private final Servo turretRight;
    private final AnalogInput turretAnalog;
    private final AbsoluteAnalogEncoder turretEnc;
    private final Utility util;

    //---------------- Software ----------------
    private final double rightTurretOffset = -0.008;
    private boolean negativeOffset;
    private final double turretPwmMinUs = 500; // are these the right limits?
    private final double turretPwmMaxUs = 2500;
    //TODO find turret limits
    private final double turretMinPos = 0;
    private final double turretMaxPos = 1;
    private final double turretStartPos = 0.5;
    private final double turretDefaultVel = 0.5; // servo units per second, 0.5 = full travel in 2 sec

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double turretCurrentPos = turretStartPos;
    private double turretTargetPos = turretStartPos;
    private double turretVel = turretDefaultVel;

    //---------------- Constructor ----------------
    public Turret(HardwareMap map) {
        turretLeft = map.get(Servo.class, "turret_left");
        turretRight = map.get(Servo.class, "turret_right");
        turretAnalog = map.get(AnalogInput.class, "turret_left_analog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        util = new Utility();
        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(turretPwmMinUs, turretPwmMaxUs);
        ((PwmControl) turretLeft).setPwmRange(pwmRange);
        ((PwmControl) turretRight).setPwmRange(pwmRange);

        negativeOffset = rightTurretOffset < 0;
    }

    //---------------- Core Methods ----------------
    private double clampTurret(double pos){
        return util.clamp(pos, (negativeOffset ? turretMinPos-rightTurretOffset : turretMinPos), (negativeOffset ? turretMaxPos : turretMaxPos-rightTurretOffset));
    }

    private void writeTurret(double pos){
        turretCurrentPos = clampTurret(pos);
        turretLeft.setPosition(turretCurrentPos);
        turretRight.setPosition(turretCurrentPos+rightTurretOffset);
    }

    // Moves instantly, and cancels any move in progress
    public void setTurret(double pos){
        turretTargetPos = clampTurret(pos);
        writeTurret(turretTargetPos);
    }

    //---------------- Methods ----------------
    // Moves toward pos a bit each update(), at vel servo units per second
    public void moveTurret(double pos, double vel){
        turretTargetPos = clampTurret(pos);
        turretVel = Math.abs(vel);
    }

    public void moveTurret(double pos){
        moveTurret(pos, turretDefaultVel);
    }

    // Keeps moving at vel servo units per second (sign = direction) until it hits a limit, 0 stops it where it is
    public void setTurretVel(double vel){
        if (vel == 0){
            turretTargetPos = turretCurrentPos;
        } else {
            moveTurret(vel > 0 ? turretMaxPos : turretMinPos, vel);
        }
    }

    public boolean isTurretAtTarget(){
        return turretCurrentPos == turretTargetPos;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void init(){
        setTurret(turretStartPos);
        loopTimer.reset();
    }

    @Override
    public void update(){
        double dt = loopTimer.seconds();
        loopTimer.reset();

        if (!isTurretAtTarget()){
            double maxStep = turretVel * dt;
            double error = turretTargetPos - turretCurrentPos;
            // Snap to the target on the last step so isTurretAtTarget() becomes exactly true
            writeTurret(Math.abs(error) <= maxStep ? turretTargetPos : turretCurrentPos + Math.signum(error) * maxStep);
        }
    }

}
