package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();
    public static MecanumConstants mecanumConstants = new MecanumConstants();
    public static ThreeWheelIMUConstants threeWheelIMUConstants = new ThreeWheelIMUConstants();

    static {
        // --- 1. DRIVETRAIN MOTOR NAMES (must match your Driver Station Hardware Map) ---
        mecanumConstants.leftFrontMotorName = "frontLeft";
        mecanumConstants.leftRearMotorName = "backLeft";
        mecanumConstants.rightFrontMotorName = "frontRight";
        mecanumConstants.rightRearMotorName = "backRight";

        // Motor directions
        mecanumConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        mecanumConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        mecanumConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        mecanumConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        // --- 2. ENCODER / DEADWHEEL HARDWARE MAP NAMES ---
        threeWheelIMUConstants.leftEncoder_HardwareMapName = "frontLeft";
        threeWheelIMUConstants.rightEncoder_HardwareMapName = "backRight";
        threeWheelIMUConstants.strafeEncoder_HardwareMapName = "frontRight";

        // --- 3. IMU NAME AND ORIENTATION ---
        // Change "imu" to match the IMU name in your Driver Station config (e.g., "imu", "imu 1", etc.)
        threeWheelIMUConstants.IMU_HardwareMapName = "imu";

        // Adjust Control Hub orientation on robot if needed
        threeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
    }

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .threeWheelIMULocalizer(threeWheelIMUConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
