package org.firstinspires.ftc.teamcode.hardwareControl.actuators.intakeSweeper;


import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.intakeSweeper.IntakeSweeperConstants;

@Configurable
public class IntakeSweeperController {

    private ServoImplEx intakeSweeperServo;

    private IntakeSweeperConstants intakeSweeperConstants;

    private double INTAKE_SWEEPER_TARGET_POSITION;
    //private double lastTargetPosition = 0.0;
    private double MAX_DEGREES;
    private double MIN_DEGREES;
    private double INTAKE_SWEEPER_X_OFFSET;
    private double INTAKE_SWEEPER_Y_OFFSET;
    private double RESET_POSITION;

    private double servoPosition = 0.0;

    private boolean initialized = false;

    // Singleton instance
    private static final IntakeSweeperController INSTANCE = new IntakeSweeperController();
    private Telemetry telemetry;

    private IntakeSweeperController() {}

    public static IntakeSweeperController getInstance() {
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
        intakeSweeperConstants = new IntakeSweeperConstants();

        MAX_DEGREES = intakeSweeperConstants.maxDegrees;
        MIN_DEGREES = intakeSweeperConstants.minDegrees;

        INTAKE_SWEEPER_X_OFFSET = intakeSweeperConstants.turretXOffset;
        INTAKE_SWEEPER_Y_OFFSET = intakeSweeperConstants.turretYOffset;

        RESET_POSITION = intakeSweeperConstants.resetPosition;
    }

    private void initializeServo(HardwareMap hardwareMap) {
        intakeSweeperServo = hardwareMap.get(ServoImplEx.class, intakeSweeperConstants.name);

        intakeSweeperServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void setTurretTargetAngle(double intakeSweeperTargetAngle) {

        intakeSweeperTargetAngle = intakeSweeperTargetAngle + 93;

        intakeSweeperTargetAngle = intakeSweeperTargetAngle % 360;

        if (intakeSweeperTargetAngle < 0) {
            intakeSweeperTargetAngle += 360;
        }

        intakeSweeperTargetAngle = Range.clip(intakeSweeperTargetAngle, MIN_DEGREES, MAX_DEGREES);

        INTAKE_SWEEPER_TARGET_POSITION = intakeSweeperTargetAngle;

        servoPosition = intakeSweeperTargetAngle / 339.267;

        intakeSweeperServo.setPosition(servoPosition);
//        telemetry.addData("Servo Target", servoPosition);
        telemetry.addData("Intake Sweeper Target", this.INTAKE_SWEEPER_TARGET_POSITION);
    }

    public void moveToTargetPosition(Pose robotPose, Position targetPosition, double scalarAddition) {
        double turretTargetAngle = this.getTurretAngleDegrees(robotPose, targetPosition, INTAKE_SWEEPER_X_OFFSET, INTAKE_SWEEPER_Y_OFFSET);

        INTAKE_SWEEPER_TARGET_POSITION = -turretTargetAngle;
        setTurretTargetAngle(INTAKE_SWEEPER_TARGET_POSITION + scalarAddition);
    }

    public double getPosition() {
        return INTAKE_SWEEPER_TARGET_POSITION - 122;
    }

    public void update() {}

    public void reset() {
        if (!initialized) return;
        initialized = false;
        setTurretTargetAngle(RESET_POSITION);
    }

    /**
     * Computes inputSweeper angle (degrees) relative to robot heading.
     *
     * @param robotPose Pose3D of robot (field coordinates)
     * @param targetPosition target position (field)
     * @param intakeSweeperXOffset inputSweeper offset forward from robot center
     * @param intakeSweeperYOffset inputSweeper offset right from robot center
     * @return inputSweeper angle in degrees (0 = aligned with robot heading)
     */
    private double getTurretAngleDegrees(
            Pose robotPose,
            Position targetPosition,
            double intakeSweeperXOffset,
            double intakeSweeperYOffset
    ) {
        // --- Target coordinates ---
        double targetX = targetPosition.x;
        double targetY = targetPosition.y;

        // --- Robot pose ---
        double robotX = robotPose.x();
        double robotY = robotPose.y();

        // Yaw in radians
        double robotYawRad = robotPose.heading();

        // --- Rotate inputSweeper offset into field frame ---
        double cos = Math.cos(robotYawRad);
        double sin = Math.sin(robotYawRad);

        double intakeSweeperX = robotX + intakeSweeperXOffset * cos - intakeSweeperYOffset * sin;
        double intakeSweeperY = robotY + intakeSweeperXOffset * sin + intakeSweeperYOffset * cos;

        // --- Vector from inputSweeper to target ---
        double vx = targetX - intakeSweeperX;
        double vy = targetY - intakeSweeperY;

        // --- Absolute angle to target in field frame ---
        double angleToTargetRad = Math.atan2(vy, vx);

        // --- Relative inputSweeper angle ---
        double intakeSweeperAngleRad = angleToTargetRad - robotYawRad;
        double intakeSweeperAngleDeg = Math.toDegrees(intakeSweeperAngleRad);

        return intakeSweeperAngleDeg;
    }
}
