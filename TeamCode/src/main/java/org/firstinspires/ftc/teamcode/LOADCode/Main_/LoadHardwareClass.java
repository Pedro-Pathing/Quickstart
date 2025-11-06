/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.LOADCode.Main_;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_Classes.DcMotorExClass;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;

/*
 * This file is designed to work with our OpModes to handle all our hardware functionality to de-clutter our main scripts
 *
 * The logic goes in the OpModes and the hardware control is handled here.
 */

//TODO Abstract all the individual systems rather than the entire hardware
/*
Example: Abstract class for drivetrain, Turret, intake, etc
They could be all individually referenced in the main OpMode
Drivetrain.doThing(args);
Turret.doOtherThing(args);
etc etc
*/
public class LoadHardwareClass {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Motors and servos go here
    // Drivetrain
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    // Turret
    private DcMotorEx turretRotationMotor = null;
    private DcMotorEx turretFlywheel = null;
    private Servo turretHoodServo = null;
    // Other
    private DcMotorEx turretMotor = null;
    private DcMotorEx intakeMotor = null;

    // Misc Constants
    private Follower follower = null;
    public Pose initialPose = new Pose(0,0, 0);

    // Public drive constants
    public double speedMultiplier = 1.0; // make this slower for outreaches

    // Declare subclasses
    public final Drivetrain drivetrain;
    public final DcMotorExClass turret;
    public final DcMotorExClass flywheel;
    public final DcMotorExClass intake;

    /**
     * Constructor that allows the OpMode to pass a reference to itself.
     * @param opmode The input for this parameter should almost always be "this".
     */
    public LoadHardwareClass(LinearOpMode opmode) {
        myOpMode = opmode;
        this.turret     = new DcMotorExClass(turretMotor);
        this.flywheel   = new DcMotorExClass(flywheelMotor);
        this.intake     = new DcMotorExClass(intakeMotor);
        this.drivetrain = new Drivetrain();
    }

    /**
     * Initializes all hardware for the robot.
     * Must be called once at the start of each op-mode.
     */
    public void init()    {
        // Define and initialize motors (note: need to use reference to actual OpMode).
        frontLeft  = myOpMode.hardwareMap.get(DcMotor.class, "FL");
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, "FR");
        backLeft   = myOpMode.hardwareMap.get(DcMotor.class, "BL");
        backRight  = myOpMode.hardwareMap.get(DcMotor.class, "BR");

        turretMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "intake");

        // Set motor/servo directions
            // Drivetrain
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.FORWARD);

        // If the motors have encoders, handle all that here

        // PedroPathing initialization
            follower = Constants.createFollower(myOpMode.hardwareMap);  // Initializes the PedroPathing path follower
            follower.setStartingPose(initialPose);                      // Sets the initial position of the robot on the field
            follower.update();                                          // Applies the initialization

        // Misc telemetry
            myOpMode.telemetry.addData(">", "Hardware Initialized");
            myOpMode.telemetry.update();
    }

    public class Drivetrain {
        /**
         * Implements a custom mecanum drive controller.
         * Must be called every loop to function properly.
         * @param Forward The joystick value for driving forward/backward
         * @param Strafe The joystick value for strafing
         * @param Rotate The joystick value to turn left/right
         */
        public void mecanumDrive(double Forward, double Strafe, double Rotate) {
            // Puts the inputted joystick values into their own variable to allow for modification.
            double forward = Forward * speedMultiplier;
            double strafe = Strafe * speedMultiplier;
            double rotate = Rotate * speedMultiplier;

            // This calculates the power needed for each wheel based on the amount of forward,
            // strafe right, and rotate
            double frontLeftPower = -forward + strafe + rotate;
            double frontRightPower = -forward - strafe - rotate;
            double backRightPower = -forward + strafe - rotate;
            double backLeftPower = -forward - strafe + rotate;

            double maxPower = 1.0;

            // This is needed to make sure we don't pass > 1.0 to any wheel
            // It allows us to keep all of the motors in proportion to what they should
            // be and not get clipped
            maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));

            // We multiply by maxSpeed so that it can be set lower for outreaches
            // When a young child is driving the robot, we may not want to allow full
            // speed.
            setDrivePower(
                    (frontLeftPower / maxPower),
                    (frontRightPower / maxPower),
                    (backLeftPower / maxPower),
                    (backRightPower / maxPower)
            );
        }

        /**
         * Uses PedroPathing's follower class to implement a mecanum drive controller.
         * Must be called every loop to function properly.
         * @param forward The joystick value for driving forward/backward
         * @param strafe The joystick value for strafing
         * @param rotate The joystick value to turn left/right
         * @param robotCentric If true, enables robot centric. If false, enables field centric.
         */
        public void pedroMecanumDrive(double forward, double strafe, double rotate, boolean robotCentric){
            follower.setTeleOpDrive(
                    forward * speedMultiplier,
                    strafe * speedMultiplier,
                    rotate * speedMultiplier,
                    robotCentric);
            follower.update();
        }

        /**
         * Sets the output power of all four drivetrain motors.
         */
        public void setDrivePower(double flPower, double frPower, double blPower, double brPower) {
            // Output the values to the motor drives.
            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);
        }
    }

    public class Turret {
        public final TurretHeading heading = new TurretHeading();

        /**
         * This class contains the controls and functions for the heading of the turret
         */
        public class TurretHeading {
            // Turret Constants
            // PID coefficients
            PIDCoefficients turretCoefficients = new PIDCoefficients(0.005, 0, 0);
            // Encoder ticks/rotation
            // 1620rpm - 103.8 ticks at the motor shaft
            double ticksPerRotation = 103.8;
            /**
             * @return The current position of the turret motor in encoder ticks. Can be any value.
             */
            public double getEncoderTicks(){
                return turretRotationMotor.getCurrentPosition();
            }
            /**
             * @return The resolution of the turret's encoder in ticks/rotation.
             */
            public double getEncoderResolution(){
                return ticksPerRotation;
            }
            /**
             * @param power A value between -1 and 1 that the turret motor's power will be set to.
             */
            public void setPower(double power){
                turretRotationMotor.setPower(power);
            }
            /**
             * @return The angle of the turret in degrees. Can be any value.
             */
            public double getAngleAbsolute(){
                return (getEncoderTicks()/ticksPerRotation*360);
            }
            /**
             * @return The angle of the turret in degrees. Can be any value between 0 and 360.
             */
            public double getAngle(){
                return getAngleAbsolute()%360;
            }
            /**
             * @return The velocity of the turret in encoder ticks/second.
             */
            public double getVelocity(){
                return turretRotationMotor.getVelocity();
            }
            /**
             * @return The velocity of the turret in degrees/second.
             */
            public double getVelocityDegrees(){
                return (getVelocity()/ticksPerRotation*360);
            }
            /**
             * @return The velocity of the turret in RPM.
             */
            public double getVelocityRPM(){
                return ((getVelocity()*60)/ticksPerRotation);
            }
            /**
             * @return The power that the turret motor has been set to.
             */
            public double getPower(){
                return turretRotationMotor.getPower();
            }
            /**
             * Uses a PID controller to move the turret to the desired position.
             * Must be called every loop to function properly.
             * @param angle The angle to move the turret to.
             */
            public void setTargetAngle(double angle){
                ControlSystem turretPID = ControlSystem.builder().posPid(turretCoefficients).build();
                KineticState currentKineticState = new KineticState(getAngleAbsolute(), getVelocity());
                turretPID.setGoal(new KineticState(angle));
                setPower(turretPID.calculate(currentKineticState));
            }
        }
    }

    public class Intake {
        // Intake Constants
        // Encoder ticks/rotation
        // 1620rpm - 103.8 ticks at the motor shaft
        double ticksPerRotation = 103.8;


        /**
         * @param power A value between -1 and 1 that the intake motor's power will be set to.
         */
        public void setPower(double power){
            intakeMotor.setPower(power);
        }

        /**
         * @return The power that the intake motor has been set to.
         */
        public double getPower(){
            return intakeMotor.getPower();
        }

        /**
         * @return The velocity of the turret in encoder ticks/second.
         */
        public double getTurretVelocity(){
            return intakeMotor.getVelocity();
        }

        /**
         * @return The velocity of the intake in RPM.
         */
        public double getTurretVelocityRPM(){
            return ((getTurretVelocity()*60)/ticksPerRotation);
        }
    }

}
