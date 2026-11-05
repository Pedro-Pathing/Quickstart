package org.firstinspires.ftc.teamcode.hardwareControl.actuators.nectarElevator;


import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.nectarElevator.NectarElevatorConstants;

@Configurable
public class NectarElevatorController {

    private ServoImplEx turretServo;

    private NectarElevatorConstants turretConstants;

    private double TURRET_TARGET_POSITION;
    //private double lastTargetPosition = 0.0;
    private double MAX_DEGREES;
    private double MIN_DEGREES;
    private double TURRET_X_OFFSET;
    private double TURRET_Y_OFFSET;
    private double RESET_POSITION;

    private double servoPosition = 0.0;

    private boolean initialized = false;

    // Singleton instance
    private static final NectarElevatorController INSTANCE = new NectarElevatorController();
    private Telemetry telemetry;

    private NectarElevatorController() {}

    public static NectarElevatorController getInstance() {
        return INSTANCE;
    }


    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        if (initialized) return;

        setupConstants();
        this.telemetry = telemetry;
        initializeServo(hardwareMap);
        initialized = true;
    }

    private void setupConstants() {
        turretConstants = new NectarElevatorConstants();

        MAX_DEGREES = turretConstants.maxDegrees;
        MIN_DEGREES = turretConstants.minDegrees;

        TURRET_X_OFFSET = turretConstants.turretXOffset;
        TURRET_Y_OFFSET = turretConstants.turretYOffset;

        RESET_POSITION = turretConstants.resetPosition;
    }

    private void initializeServo(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(ServoImplEx.class, turretConstants.name);

        turretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void setTurretTargetAngle(double turretTargetAngle) {

        turretTargetAngle = turretTargetAngle + 93;

        turretTargetAngle = turretTargetAngle % 360;

        if (turretTargetAngle < 0) {
            turretTargetAngle += 360;
        }

        turretTargetAngle = Range.clip(turretTargetAngle, MIN_DEGREES, MAX_DEGREES);

        TURRET_TARGET_POSITION = turretTargetAngle;

        servoPosition = turretTargetAngle / 339.267;

        turretServo.setPosition(servoPosition);
//        telemetry.addData("Servo Target", servoPosition);
        telemetry.addData("Turret Target", this.TURRET_TARGET_POSITION);
    }

    public void moveToTargetPosition(Pose robotPose, Position targetPosition, double scalarAddition) {
        double turretTargetAngle = this.getTurretAngleDegrees(robotPose, targetPosition, TURRET_X_OFFSET, TURRET_Y_OFFSET);

        TURRET_TARGET_POSITION = -turretTargetAngle;
        setTurretTargetAngle(TURRET_TARGET_POSITION + scalarAddition);
    }

    public double getPosition() {
        return TURRET_TARGET_POSITION - 122;
    }

    public void update() {}

    public void reset() {
        if (!initialized) return;
        initialized = false;
        setTurretTargetAngle(RESET_POSITION);
    }

    /**
     * Computes turret angle (degrees) relative to robot heading.
     *
     * @param robotPose Pose3D of robot (field coordinates)
     * @param targetPosition target position (field)
     * @param turretXOffset turret offset forward from robot center
     * @param turretYOffset turret offset right from robot center
     * @return turret angle in degrees (0 = aligned with robot heading)
     */
    private double getTurretAngleDegrees(
            Pose robotPose,
            Position targetPosition,
            double turretXOffset,
            double turretYOffset
    ) {
        // --- Target coordinates ---
        double targetX = targetPosition.x;
        double targetY = targetPosition.y;

        // --- Robot pose ---
        double robotX = robotPose.x();
        double robotY = robotPose.y();

        // Yaw in radians
        double robotYawRad = robotPose.heading();

        // --- Rotate turret offset into field frame ---
        double cos = Math.cos(robotYawRad);
        double sin = Math.sin(robotYawRad);

        double turretX = robotX + turretXOffset * cos - turretYOffset * sin;
        double turretY = robotY + turretXOffset * sin + turretYOffset * cos;

        // --- Vector from turret to target ---
        double vx = targetX - turretX;
        double vy = targetY - turretY;

        // --- Absolute angle to target in field frame ---
        double angleToTargetRad = Math.atan2(vy, vx);

        // --- Relative turret angle ---
        double turretAngleRad = angleToTargetRad - robotYawRad;
        double turretAngleDeg = Math.toDegrees(turretAngleRad);

        return turretAngleDeg;
    }
}
