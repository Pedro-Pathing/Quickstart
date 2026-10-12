package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive implements Subsystem{

    //---------------- Hardware ----------------
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private IMU imu;

    //---------------- Software ----------------
    double leftFrontPow = 0.0;
    double rightFrontPow = 0.0;
    double leftBackPow = 0.0;
    double rightBackPow = 0.0;

    public double FAST_MULT = 0.95;
    public double SLOW_MULT = 0.5;
    public double speed = SLOW_MULT;
    public boolean manualDrive = true;
    public boolean useSlowMode = false;
    public boolean useFieldCentric = false;

    //---------------- Constructor ----------------
    public Drive(HardwareMap map) {
        frontLeftDrive = map.get(DcMotor.class, "FrontLeft");
        frontRightDrive = map.get(DcMotor.class, "FrontRight");
        backLeftDrive = map.get(DcMotor.class, "BackLeft");
        backRightDrive = map.get(DcMotor.class, "BackRight");

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = map.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }


    //---------------- Core Methods ----------------


    //---------------- Methods ----------------
    public void setDrivePowers(double lf, double rf, double lb, double rb){
        leftFrontPow = lf;
        rightFrontPow = rf;
        leftBackPow = lb;
        rightBackPow = rb;
    }

    public void toggleSlowMode(){
        useSlowMode = !useSlowMode;
    }

    public void toggleFieldCentric() {
        useFieldCentric = !useFieldCentric;
    }
    public void driveFieldRelative(double forward, double right, double rotate) {

        Double[] params = {forward, right, rotate};
        for (int i = 0; i < params.length; i++) {
            if (Math.abs(params[i]) <= 0.1) {
                params[i] = 0.0;
            }
        }

        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 0.5;  // make this slower for outreaches

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
        setDrivePowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    //---------------- Interface Methods ----------------
    @Override
    public void init(){}

    @Override
    public void update(){
        speed = (useSlowMode ? SLOW_MULT : FAST_MULT);
        if (manualDrive) {
            leftFrontPow *= speed;
            rightFrontPow *= speed;
            leftBackPow *= speed;
            rightBackPow *= speed;
            frontLeftDrive.setPower(leftFrontPow);
            frontRightDrive.setPower(rightFrontPow);
            backLeftDrive.setPower(leftBackPow);
            backRightDrive.setPower(rightBackPow);
        }
    }

    @Override
    public void stop(){
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
