package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

@Configurable
public class Turret {

    // Hardware definitions
    public final Devices.DcMotorExClass rotation = new Devices.DcMotorExClass();
    public final Devices.DcMotorExClass flywheel = new Devices.DcMotorExClass();
    private final Devices.DcMotorExClass flywheel2 = new Devices.DcMotorExClass();
    private final Devices.ServoClass hood = new Devices.ServoClass();
    private final Devices.ServoClass gate = new Devices.ServoClass();
    private final Devices.REVHallEffectSensorClass hall = new Devices.REVHallEffectSensorClass();

    // Turret PID coefficients
    public static PIDCoefficients turretCoefficients = new PIDCoefficients(0.06, 0, 0); // 223RPM
    //public static PIDCoefficients turretCoefficients = new PIDCoefficients(0.3, 0, 0.007); // 1150RPM

    // Flywheel PID coefficients
    // 4500RPM
    public static PIDCoefficients flywheelCoefficients = new PIDCoefficients(0.0002, 0, 0);
    public static BasicFeedforwardParameters flywheelFFCoefficients = new BasicFeedforwardParameters(0.000026,0,0);

    public enum gatestate {
        OPEN,
        CLOSED,
    }

    public enum flywheelstate {
        ON,
        OFF
    }
    public flywheelstate flywheelState = flywheelstate.OFF;

    /** Controls the target speed of the flywheel when it is on */
    public static double flywheelSpeed = 4500;
    /** Controls the upper software limit of the hood */
    public static double upperHoodLimit = 260;
    /** Controls the speed at which the turret will zero itself */
    public static double zeroSpeed = 0.2;

    // Stores the opmode object for later access
    OpMode opMode = null;

    public void init(OpMode opmode){
        opMode = opmode;
        rotation.init(opmode, "turret", 145.1 * ((double) 131 /24)); //Previously 103.8
        flywheel.init(opmode, "flywheel", 28);
        flywheel2.init(opmode, "flywheel2", 28);
        hood.init(opmode, "hood");
        gate.init(opmode, "gate");
        hall.init(opmode, "hall");

        rotation.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        rotation.setDirection(DcMotorSimple.Direction.REVERSE);

        gate.setAngle(0.5);
        hood.setDirection(Servo.Direction.REVERSE);

        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Pass PID pidCoefficients to motor classes
        rotation.setPidCoefficients(turretCoefficients);
        flywheel.setPidCoefficients(flywheelCoefficients);
        flywheel.setFFCoefficients(flywheelFFCoefficients);
        flywheel2.setPidCoefficients(flywheelCoefficients);
        flywheel2.setFFCoefficients(flywheelFFCoefficients);
    }

    public void updatePIDs(){
        // Pass PID pidCoefficients to motor classes
        rotation.setPidCoefficients(turretCoefficients);
        flywheel.setPidCoefficients(flywheelCoefficients);
        flywheel.setFFCoefficients(flywheelFFCoefficients);
        flywheel2.setPidCoefficients(flywheelCoefficients);
        flywheel2.setFFCoefficients(flywheelFFCoefficients);
    }

    /**
     * @param robot The robot object in that opmode
     */
    public void updateAimbot(@NonNull LoadHardwareClass robot){
        rotation.setAngle(calcLocalizer(robot.drivetrain.follower.getPose(), LoadHardwareClass.selectedAlliance));
    }

    public void setGateState(gatestate state){
        if (state == gatestate.OPEN){
            gate.setAngle(0.5);
        }else if (state == gatestate.CLOSED){
            gate.setAngle(0.25);
        }
    }

    /**
     * Sets the angle of the hood.
     * @param angle An angle in degrees that is constrained to between 0 and 320 degrees
     */
    public void setHood(double angle){
        hood.setAngle(Math.min(Math.max(angle, 0), upperHoodLimit)/(360*5));
    }

    /**
     * Gets the angle of the hood
     * @return A value between 0 and 45 degrees
     */
    public double getHood(){
        return hood.getAngle() * 360 * 5;
    }

    /**
     * Outputs one of the following modes
     * <ul>
     *     <li><code>gatestate.OPEN</code></li>
     *     <li><code>gatestate.CLOSED</code></li>
     * </ul>
     */
    public gatestate getGate(){
        if (gate.getAngle() == 0.5){
            return gatestate.OPEN;
        } else {
            return gatestate.CLOSED;
        }
    }

    public double calcLocalizer (Pose robotPose, LoadHardwareClass.Alliance alliance){
        Pose goalPose = new Pose(4,140,0);
        if (alliance == LoadHardwareClass.Alliance.RED) {goalPose = new Pose(140, 140, 0);}

        return Math.toDegrees(Math.atan2(
                goalPose.getY()-robotPose.getY(),
                goalPose.getX()-robotPose.getX())
        ) - Math.toDegrees(robotPose.getHeading());
    }

    /**
     * @param rpm
     * RPM Range [0,6000]
     */
    private void setFlywheelRPM(double rpm){
        flywheel.setRPM(rpm);
        flywheel2.setPower(flywheel.getPower());
    }

    public double getFlywheelRPM(){
        return flywheel.getRPM();
    }

    /**
     * Sets the target state of the Flywheel. </br>
     * <code>updateFlywheel()</code> must be called every loop for this to be effective.
     * @param state The state to set the flywheel to (ON/OFF)
     */
    public void setFlywheelState(flywheelstate state){
        flywheelState = state;
    }

    public void zeroTurret(boolean stopIsCalled){
        while (!(hall.getTriggered() || stopIsCalled)){
            rotation.setPower(zeroSpeed);
        }
        rotation.setPower(0);
        rotation.resetEncoder();
    }

    /**
     * Updates the flywheel PID. Must be called every loop.
     */
    public void updateFlywheel(){
        if (flywheelState == flywheelstate.ON){
            setFlywheelRPM(flywheelSpeed);
        } else if (flywheelState == flywheelstate.OFF){
            setFlywheelRPM(0);
        }
    }
}
