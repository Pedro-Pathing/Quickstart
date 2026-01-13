package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

public class Devices {

    public static class CRServoClass {
        private CRServo servo;

        public void init(@NonNull OpMode opmode, String servoName) {
            servo = opmode.hardwareMap.get(CRServo.class, servoName);
        }

        /**
         * @param power The power to set the servo to. Must be a value between -1 and 1.
         */
        public void setPower(double power) {
            servo.setPower(power);
        }

        /**
         * @param direction The direction to set the servo to.
         */
        public void setDirection(DcMotorSimple.Direction direction) {
            servo.setDirection(direction);
        }

        /**
         * @return The power the servo has been set to.
         */
        public double getPower() {
            return servo.getPower();
        }
    }

    public static class DcMotorExClass {

        // Old PID Coefficients
        PIDCoefficients old_pidCoefficients = new PIDCoefficients(0, 0, 0);
        BasicFeedforwardParameters old_ffCoefficients = new BasicFeedforwardParameters(0,0,0);

        // PID Coefficients
        PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0, 0);
        BasicFeedforwardParameters ffCoefficients = new BasicFeedforwardParameters(0,0,0);
        /**
         * <h4>Encoder ticks/rotation:</h4><br>
         *      1620rpm Gobilda - 103.8<br>
         *      1150rpm Gobilda - 145.1<br>
         *      223rpm Gobilda - 751.8<br>
         */
        public double ticksPerRotation = 103.8;
        // Target position/velocity of the motor
        public double target = 0;
        // Offset position of the motor
        public double offset = 0;
        // Motor object
        private DcMotorEx motorObject = null;

        /**
         * Initializes the motor hardware
         * @param opmode Allows this class to access the robot hardware objects.
         * @param motorName The name of the motor in the robot's configuration.
         * @param encoderResolution The resolution of the motor's encoder in ticks/rotation.
         */
        public void init (@NonNull OpMode opmode, String motorName, double encoderResolution){
            // Initialize the motor object
            motorObject  = opmode.hardwareMap.get(DcMotorEx.class, motorName);
            ticksPerRotation = encoderResolution;
        }
        /**
         * Initializes the motor hardware.
         * The resolution of the motor encoder will default to
         *      103.8 ticks/rotation, the value for a 1620RPM Gobilda motor.
         * @param opmode Allows this class to access the robot hardware objects.
         * @param motorName The name of the motor in the robot's configuration.
         */
        public void init (@NonNull OpMode opmode, String motorName){
            // Initialize the motor object
            motorObject  = opmode.hardwareMap.get(DcMotorEx.class, motorName);
        }

        ControlSystem velPID = null;
        ControlSystem posPID = null;

        public void buildPIDs(){
            if (old_pidCoefficients != pidCoefficients || old_ffCoefficients != ffCoefficients){
                posPID = ControlSystem.builder().posPid(pidCoefficients).build();
                velPID = ControlSystem.builder()
                        .velPid(pidCoefficients)
                        .basicFF(ffCoefficients)
                        .build();
            }
            old_pidCoefficients = pidCoefficients;
            old_ffCoefficients = ffCoefficients;
        }

        /**
         * Sets the value of the PID coefficients of the motor.
         * @param coefficients The values to set the coefficients to.
         */
        public void setPidCoefficients(PIDCoefficients coefficients) {
            pidCoefficients = coefficients;
        }
        /**
         * Sets the value of the FeedForward coefficients of the motor.
         * @param coefficients The values to set the coefficients to.
         */
        public void setFFCoefficients(BasicFeedforwardParameters coefficients) {
            ffCoefficients = coefficients;
        }
        /**
         * Resets the internal encoder of the motor to zero.
         */
        public void resetEncoder(){
            motorObject.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorObject.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        /**
         * Sets a position offset on the motor in encoder ticks.
         */
        public void setOffsetTicks(double ticks){
            offset = ticks;
        }
        public void setOffsetDegrees(double degrees){
            setOffsetTicks(degrees * (ticksPerRotation/360));
        }
        /**
         * Sets the runMode of the motor.
         * @param runMode The mode to set the motor to.
         */
        public void setRunMode(DcMotor.RunMode runMode){
            motorObject.setMode(runMode);
        }
        /**
         * Sets the zeroPowerBehaviour of the motor.
         * @param behaviour The behaviour to apply to the motor.
         */
        public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behaviour){
            motorObject.setZeroPowerBehavior(behaviour);
        }
        /**
         * @param direction The direction to set the motor to.
         */
        public void setDirection(DcMotorSimple.Direction direction){
            motorObject.setDirection(direction);
        }
        public void setEncoderTicks(int ticks){
            motorObject.setTargetPosition(ticks);
            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        /**
         * @return The current position of the turret motor in encoder ticks. Can be any value.
         */
        public double getEncoderTicks(){
            return motorObject.getCurrentPosition() + offset;
        }
        /**
         * @param power A value between -1 and 1 that the turret motor's power will be set to.
         */
        public void setPower(double power){
            motorObject.setPower(power);
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
            return motorObject.getVelocity();
        }
        /**
         * @return The velocity of the turret in degrees/second.
         */
        public double getDegreesPerSecond(){
            return (getVelocity()/ticksPerRotation)*360;
        }
        /**
         * @return The velocity of the turret in RPM.
         */
        public double getRPM(){
            return (getVelocity()/ticksPerRotation)*60;
        }
        /**
         * @return The power that the turret motor has been set to.
         */
        public double getPower(){
            return motorObject.getPower();
        }
        /**
         * Uses a PID controller to move the motor to the desired position.
         * Must be called every loop to function properly.
         * @param angle The angle in degrees to move the motor to. Can be any number. </br>
         */
        public void setAngle(double angle){
            setAngle(angle, 0);
        }
        /**
         * Uses a PID controller to move the motor to the desired position.
         * Must be called every loop to function properly.
         * @param angle The angle in degrees to move the motor to. Can be any number.
         * @param velocity The velocity in degrees/sec that the motor should be spinning at when it reaches the target point.
         */
        public void setAngle(double angle, double velocity){
            target = angle;
            buildPIDs();
            KineticState currentKineticState = new KineticState(getAngleAbsolute(), getDegreesPerSecond());
            posPID.setGoal(new KineticState(target, velocity));
            setPower(posPID.calculate(currentKineticState));
        }

        /**
         * Uses a PID controller to accelerate the motor to the desired RPM.
         * Must be called every loop to function properly.
         * @param rpm The RPM to accelerate the motor to. Can be any number
         */
        public void setRPM(double rpm){
            target = rpm;
            buildPIDs();
            double degreesPerSecond = target*6;
            KineticState currentKineticState = new KineticState(getAngleAbsolute(), getDegreesPerSecond());
            velPID.setGoal(new KineticState(0, degreesPerSecond));
            setPower(velPID.calculate(currentKineticState));
        }
    }

    public static class ServoClass {
        private Servo servo;

        public void init(@NonNull OpMode opmode, String servoName){
            servo = opmode.hardwareMap.get(Servo.class, servoName);
        }

        /**
         * @param angle The angle to set the servo to. Must be a value between 0 and 1,
         *              representing the endpoints of it's movement.
         */
        public void setAngle(double angle){
            servo.setPosition(angle);
        }

        /**
         * @param direction The direction to set the servo to.
         */
        public void setDirection(Servo.Direction direction){
            servo.setDirection(direction);
        }

        /**
         * @return A value between 0 and 1 that the servo has been set to.
         */
        public double getAngle(){
            return servo.getPosition();
        }
    }

    public static class REVColorSensorV3Class {
        private NormalizedColorSensor sensor;

        public void init(@NonNull OpMode opmode, String sensorName){
            sensor = opmode.hardwareMap.get(NormalizedColorSensor.class, sensorName);
        }

        public NormalizedRGBA getNormalizedColors(){
            return sensor.getNormalizedColors();
        }

        public double getGain(){
            return sensor.getGain();
        }

        public void setGain(double gain){
            sensor.setGain((float) gain);
        }

        public double getDistance(DistanceUnit units){
            return ((DistanceSensor) sensor).getDistance(units);
        }
    }

    public static class DualProximitySensorClass {
        private final REVColorSensorV3Class sensor1 = new REVColorSensorV3Class();
        private final REVColorSensorV3Class sensor2 = new REVColorSensorV3Class();

        public double threshold = 2;
        public DistanceUnit units = DistanceUnit.CM;

        public void init(@NonNull OpMode opmode, String sensor1Name, String sensor2Name){
            sensor1.init(opmode, sensor1Name);
            sensor2.init(opmode, sensor2Name);
        }

        public void setGain(double gain){
            sensor1.setGain(gain);
            sensor2.setGain(gain);
        }

        public boolean objectDetected(){
            return (sensor1.getDistance(units) < threshold || sensor2.getDistance(units) < threshold);
        }

        public double[] getDistances(){
            return new double[]{sensor1.getDistance(units), sensor2.getDistance(units)};
        }
    }

    public static class REVHallEffectSensorClass {
        private DigitalChannel sensor;

        public void init(@NonNull OpMode opMode, String sensorName){
            sensor = opMode.hardwareMap.get(DigitalChannel.class, sensorName);
            sensor.setMode(DigitalChannel.Mode.INPUT);
        }

        public Boolean getTriggered(){
            return !sensor.getState();
        }
    }
}