package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Utils_;

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
    public static PIDCoefficients turretCoefficients = new PIDCoefficients(0.06, 0, 0); // 223RPM Motor
    //public static PIDCoefficients turretCoefficients = new PIDCoefficients(0.3, 0, 0.007); // 1150RPM Motor

    // Flywheel PID coefficients
    // 4500RPM
    public static PIDCoefficients flywheelCoefficients = new PIDCoefficients(0.0002, 0, 0);
    public static BasicFeedforwardParameters flywheelFFCoefficients = new BasicFeedforwardParameters(0.000026,0,0);

    // Define any Enums here
    public enum gatestate {
        OPEN,
        CLOSED,
    }
    public enum flywheelstate {
        ON,
        OFF
    }

    // Define any state variables or important parameters here
    /** Stores the current state of the flywheel.*/
    public flywheelstate flywheelState = flywheelstate.OFF;
    /** Controls the target speed of the flywheel when it is on.*/
    public static double flywheelSpeed = 4500;
    /** Controls the upper software limit of the hood.*/
    public static double upperHoodLimit = 260;
    /** Controls the speed at which the turret will move to zero itself.*/
    public static double zeroSpeed = 0.2;

    // Stores important objects for later access
    OpMode opMode = null;
    LoadHardwareClass Robot = null;

    // The variable to store the InterpLUT table for turret hood aimbot
    Utils_.InterpLUT hoodLUT = new Utils_.InterpLUT();

    public void init(OpMode opmode, LoadHardwareClass robot){
        // Store important objects in their respective variables
        opMode = opmode;
        Robot = robot;

        // Initialize hardware objects
        rotation.init(opmode, "turret", 145.1 * ((double) 131 /24)); //Previously 103.8
        flywheel.init(opmode, "flywheel", 28);
        flywheel2.init(opmode, "flywheel2", 28);
        hood.init(opmode, "hood");
        gate.init(opmode, "gate");
        hall.init(opmode, "hall");

        // Set servos to initial positions
        setGateState(gatestate.CLOSED);
        setHood(0);

        // Set servo directions
        hood.setDirection(Servo.Direction.REVERSE);

        // Set motor directions and zero power behaviour
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        rotation.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        rotation.setDirection(DcMotorSimple.Direction.REVERSE);

        // Pass PID pidCoefficients to motor classes
        rotation.setPidCoefficients(turretCoefficients);
        flywheel.setPidCoefficients(flywheelCoefficients);
        flywheel.setFFCoefficients(flywheelFFCoefficients);
        flywheel2.setPidCoefficients(flywheelCoefficients);
        flywheel2.setFFCoefficients(flywheelFFCoefficients);

        // TODO Build hood InterpLUT for autoaim
//        hoodLUT.add(0,0);
//        hoodLUT.add(1,1);
    }

    /** Sets the value of the internal motor PID coefficients */
    public void updatePIDs(){
        // Pass PID pidCoefficients to motor classes
        rotation.setPidCoefficients(turretCoefficients);
        flywheel.setPidCoefficients(flywheelCoefficients);
        flywheel.setFFCoefficients(flywheelFFCoefficients);
        flywheel2.setPidCoefficients(flywheelCoefficients);
        flywheel2.setFFCoefficients(flywheelFFCoefficients);
    }

    /**
     * Runs the aimbot program to control the turret rotation and hood angle. </br>
     * Must be called every loop to function properly.
     */
    public void updateAimbot(){
        // Set the turret rotation
        rotation.setAngle(calcLocalizer());
        // Set the hood angle
        Pose goalPose = new Pose(4,140,0);
        if (LoadHardwareClass.selectedAlliance == LoadHardwareClass.Alliance.RED) {goalPose = new Pose(140, 140, 0);}
        //setHood(hoodLUT.get(Robot.drivetrain.follower.getPose().distanceFrom(goalPose)));
    }

    /**
     * Sets the state of the turret gate.
     */
    public void setGateState(gatestate state){
        if (state == gatestate.OPEN){
            gate.setAngle(0.5);
        }else if (state == gatestate.CLOSED){
            gate.setAngle(0.25);
        }
    }

    /**
     * Sets the angle of the hood.
     * @param angle An angle in degrees that is constrained to between 0 and the upper hood limit.
     */
    public void setHood(double angle){
        hood.setAngle(Math.min(Math.max(angle, 0), upperHoodLimit)/(360*5));
    }

    /**
     * Gets the last set position of the turret hood.
     * @return The angle of the hood in degrees.
     */
    public double getHood(){
        return hood.getAngle() * 360 * 5;
    }

    /**
     * Gets the current state of the turret gate.
     * Outputs one of the following modes.
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

    /**
     * Calculates the target angle to rotate the turret to in order to aim at the correct goal. </br>
     * Currently uses Pinpoint Odometry and trigonometry to get the angle.
     */
    public double calcLocalizer (){
        Pose goalPose = new Pose(4,140,0);
        if (LoadHardwareClass.selectedAlliance == LoadHardwareClass.Alliance.RED) {goalPose = new Pose(140, 140, 0);}

        return Math.toDegrees(Math.atan2(
                goalPose.getY()-Robot.drivetrain.follower.getPose().getY(),
                goalPose.getX()-Robot.drivetrain.follower.getPose().getX())
        ) - Math.toDegrees(Robot.drivetrain.follower.getPose().getHeading());
    }

    /**
     * Sets the RPM of the flywheel.
     * @param rpm
     * Range [0,6000]
     */
    private void setFlywheelRPM(double rpm){
        flywheel.setRPM(rpm);
        flywheel2.setPower(flywheel.getPower());
    }

    /**
     * Gets the current RPM of the flywheel.
     */
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

    /**
     * FIXME This function is untested!!
     * Resets the zero position of the turret using the hall effect switch
     * @param stopIsCalled
     * Pass in the value of <code>isStopRequested()</code>
     */
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
