package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import android.graphics.Color;

/**
 * Shared functions for TeleOp and Autonomous modes
 */
public class RobotFunctionsAuto {
    
    private HardwareConfigAuto robot;
    
    // Bang-bang controller state
    private double lastShooterPower = 0.0;
    
    // Light flash state removed - using solid colors only
    
    public RobotFunctionsAuto(HardwareConfigAuto robot) {
        this.robot = robot;
    }
    
    /**
     * Mecanum drive control
     * @param drive Forward/backward movement (-1.0 to 1.0)
     * @param strafe Left/right movement (-1.0 to 1.0)
     * @param turn Rotation (-1.0 to 1.0)
     * @param speedMultiplier Speed scaling factor (0.0 to 1.0)
     */
//    public void mecanumDrive(double drive, double strafe, double turn, double speedMultiplier) {
//        // Calculate motor powers using mecanum drive kinematics
//        double frontLeftPower = (drive + strafe + turn) * speedMultiplier;
//        double frontRightPower = (drive - strafe - turn) * speedMultiplier;
//        double backLeftPower = (drive - strafe + turn) * speedMultiplier;
//        double backRightPower = (drive + strafe - turn) * speedMultiplier;
//
//        // Normalize wheel powers to be within -1.0 to 1.0
//        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(backRightPower));
//
//        if (maxPower > 1.0) {
//            frontLeftPower /= maxPower;
//            frontRightPower /= maxPower;
//            backLeftPower /= maxPower;
//            backRightPower /= maxPower;
//        }
//
//        // Set motor powers
//        robot.frontLeft.setPower(frontLeftPower);
//        robot.frontRight.setPower(frontRightPower);
//        robot.backLeft.setPower(backLeftPower);
//        robot.backRight.setPower(backRightPower);
//    }
    
    /**
     * Run intake and transfer motors
     * @param power Power level (-1.0 to 1.0, negative for reverse)
     */
    public void runIntakeSystem(double power) {
        robot.intakeMotor.setPower(power);
        robot.transferMotor.setPower(power);
    }
    
    /**
     * Run intake system with dynamic transfer speed based on distance
     * Used during shooting - reduces transfer speed when far away
     */
    public void runIntakeSystemShooting() {
        robot.intakeMotor.setPower(HardwareConfigAuto.INTAKE_POWER);
        robot.transferMotor.setPower(getTransferSpeed());
    }
    // public void Lclimb(double position){
    //     robot.Lclimb.setPosition(position);
    // }
    // public void Rclimb(double position){
    //     robot.Rclimb.setPosition(position);
    // }
    // public double getLeftClimb(){
    //     return robot.Lclimb.getPosition();
    // }
    // public double getRightClimb(){
    //     return robot.Rclimb.getPosition();
    // }
    /**
     * Stop intake and transfer motors
     */
    public void stopIntakeSystem() {
        robot.intakeMotor.setPower(0);
        robot.transferMotor.setPower(0);
    }
    
    /**
     * Get current shooter speed in ticks per second
     * @return Current velocity in ticks per second
     */
    public double getShooterTPS() {
        // Get velocity in ticks per second from shooter motor encoder
        return Math.abs(robot.shooterMotor.getVelocity());
    }
    
    /**
     * Get current shooter RPM (for telemetry/backwards compatibility)
     * @return Current RPM
     */
    public double getShooterRPM() {
        final double TICKS_PER_REV = 28.0;
        double tps = getShooterTPS();
        return (tps * 60.0) / TICKS_PER_REV;
    }
    
    /**
     * Calculate target shooter speed based on Limelight area
     * Uses piecewise linear interpolation between 4 calibrated points:
     * - 3.75% area = 1300 TPS
     * - 2.40% area = 1400 TPS
     * - 1.05% area = 1500 TPS
     * - 0.37% area = 1750 TPS
     * @return Target ticks per second
     */
    public double getTargetShooterTPS() {
        LLResult result = robot.limelight.getLatestResult();
        
        // If no valid target, return default speed
        if (result == null || !result.isValid()) {
            return HardwareConfigAuto.SHOOTER_DEFAULT_TPS;
        }
        
        // Get target area
        double ta = result.getTa();
        
        // Piecewise linear interpolation with 6 calibration points
        double targetTPS;
        
        if (ta >= HardwareConfigAuto.LIMELIGHT_AREA_1) {
            // Very close or closer than 4.3%
            targetTPS = HardwareConfigAuto.SHOOTER_TPS_1;
            
        } else if (ta >= HardwareConfigAuto.LIMELIGHT_AREA_2) {
            // Between 4.3% and 1.5% - interpolate
            double areaRange = HardwareConfigAuto.LIMELIGHT_AREA_1 - HardwareConfigAuto.LIMELIGHT_AREA_2;
            double speedRange = HardwareConfigAuto.SHOOTER_TPS_1 - HardwareConfigAuto.SHOOTER_TPS_2;
            double normalizedArea = (ta - HardwareConfigAuto.LIMELIGHT_AREA_2) / areaRange;
            targetTPS = HardwareConfigAuto.SHOOTER_TPS_2 + (normalizedArea * speedRange);
            
        } else if (ta >= HardwareConfigAuto.LIMELIGHT_AREA_3) {
            // Between 1.5% and 0.88% - interpolate
            double areaRange = HardwareConfigAuto.LIMELIGHT_AREA_2 - HardwareConfigAuto.LIMELIGHT_AREA_3;
            double speedRange = HardwareConfigAuto.SHOOTER_TPS_2 - HardwareConfigAuto.SHOOTER_TPS_3;
            double normalizedArea = (ta - HardwareConfigAuto.LIMELIGHT_AREA_3) / areaRange;
            targetTPS = HardwareConfigAuto.SHOOTER_TPS_3 + (normalizedArea * speedRange);
            
        } else if (ta >= HardwareConfigAuto.LIMELIGHT_AREA_4) {
            // Between 0.88% and 0.7% - interpolate
            double areaRange = HardwareConfigAuto.LIMELIGHT_AREA_3 - HardwareConfigAuto.LIMELIGHT_AREA_4;
            double speedRange = HardwareConfigAuto.SHOOTER_TPS_3 - HardwareConfigAuto.SHOOTER_TPS_4;
            double normalizedArea = (ta - HardwareConfigAuto.LIMELIGHT_AREA_4) / areaRange;
            targetTPS = HardwareConfigAuto.SHOOTER_TPS_4 + (normalizedArea * speedRange);
            
        } else if (ta >= HardwareConfigAuto.LIMELIGHT_AREA_5) {
            // Between 0.7% and 0.4% - interpolate
            double areaRange = HardwareConfigAuto.LIMELIGHT_AREA_4 - HardwareConfigAuto.LIMELIGHT_AREA_5;
            double speedRange = HardwareConfigAuto.SHOOTER_TPS_4 - HardwareConfigAuto.SHOOTER_TPS_5;
            double normalizedArea = (ta - HardwareConfigAuto.LIMELIGHT_AREA_5) / areaRange;
            targetTPS = HardwareConfigAuto.SHOOTER_TPS_5 + (normalizedArea * speedRange);
            
        } else if (ta >= HardwareConfigAuto.LIMELIGHT_AREA_6) {
            // Between 0.4% and 0.3% - interpolate
            double areaRange = HardwareConfigAuto.LIMELIGHT_AREA_5 - HardwareConfigAuto.LIMELIGHT_AREA_6;
            double speedRange = HardwareConfigAuto.SHOOTER_TPS_5 - HardwareConfigAuto.SHOOTER_TPS_6;
            double normalizedArea = (ta - HardwareConfigAuto.LIMELIGHT_AREA_6) / areaRange;
            targetTPS = HardwareConfigAuto.SHOOTER_TPS_6 + (normalizedArea * speedRange);
            
        } else {
            // Very far or farther than 0.3%
            targetTPS = HardwareConfigAuto.SHOOTER_TPS_6;
        }
        
        // Clamp to overall min/max for safety
        if (targetTPS < HardwareConfigAuto.SHOOTER_MIN_TPS) {
            targetTPS = HardwareConfigAuto.SHOOTER_MIN_TPS;
        }
        if (targetTPS > HardwareConfigAuto.SHOOTER_MAX_TPS) {
            targetTPS = HardwareConfigAuto.SHOOTER_MAX_TPS;
        }
        
        return targetTPS;
    }
    
    /**
     * Bang-bang controller for shooter motor
     * Automatically determines target speed based on Limelight area
     * @param running Whether the shooter should be running
     */
    public void controlShooter(boolean running) {
        if (!running) {
            robot.shooterMotor.setPower(0);
            lastShooterPower = 0.0;
            return;
        }
        
        double targetTPS = getTargetShooterTPS();
        double currentTPS = getShooterTPS();
        double power;
        
        // Bang-bang control: full power if below target, zero if above
        if (currentTPS < targetTPS) {
            power = HardwareConfigAuto.SHOOTER_MAX_POWER;
        } else {
            power = 0.0;
        }
        
        // Apply power to shooter motor
        robot.shooterMotor.setPower(power);
        
        lastShooterPower = power;
    }
    
    /**
     * Bang-bang controller with manual target speed
     * @param targetTPS Target speed in ticks per second
     * @param running Whether the shooter should be running
     */
    public void controlShooterManual(double targetTPS, boolean running) {
        if (!running) {
            robot.shooterMotor.setPower(0);
            lastShooterPower = 0.0;
            return;
        }
        
        double currentTPS = getShooterTPS();
        double power;
        
        // Bang-bang control: full power if below target, zero if above
        if (currentTPS < targetTPS) {
            power = HardwareConfigAuto.SHOOTER_MAX_POWER;
        } else {
            power = 0.0;
        }
        
        // Apply power to shooter motor
        robot.shooterMotor.setPower(power);
        
        lastShooterPower = power;
    }
    
    /**
     * Check if shooter is at target speed and robot is aligned (automatic mode)
     * Uses dynamic alignment tolerance based on distance (AprilTag area)
     * @return True if shooter is within tolerance of target speed AND aligned
     */
    public boolean isShooterReady() {
        double targetTPS = getTargetShooterTPS();
        double currentTPS = getShooterTPS();
        
        // Check if shooter speed is ready
        boolean speedReady = Math.abs(currentTPS - targetTPS) <= HardwareConfigAuto.SHOOTER_READY_THRESHOLD;
        
        // Check if aligned to target (within tolerance of target offset, not just TX = 0)
        boolean aligned = false;
        int tagId = getDetectedAprilTagId();
        if (tagId == 20 || tagId == 24) {
            double currentTx = getLimelightTx();
            double targetOffset = getCurrentTargetOffset();
            double error = Math.abs(currentTx - targetOffset);
            
            // Use dynamic tolerance based on AprilTag area (distance)
            double tolerance = getAlignmentTolerance();
            aligned = error < tolerance;
        }
        
        return speedReady && aligned;
    }
    
    /**
     * Get alignment tolerance based on AprilTag area (distance)
     * Closer targets (area >= 0.5) allow more tolerance (5°)
     * Farther targets (area < 0.5) require stricter tolerance (2°)
     * @return Alignment tolerance in degrees
     */
    public double getAlignmentTolerance() {
        double area = getAprilTagArea();
        
        if (area >= HardwareConfigAuto.APRILTAG_AREA_CLOSE_THRESHOLD) {
            return HardwareConfigAuto.SHOOTER_READY_ALIGNMENT_TOLERANCE_CLOSE;
        } else {
            return HardwareConfigAuto.SHOOTER_READY_ALIGNMENT_TOLERANCE_FAR;
        }
    }
    
    /**
     * Get transfer motor speed based on AprilTag area (distance)
     * Farther targets (area < 0.5) use reduced speed (0.75)
     * Closer targets (area >= 0.5) use full speed (1.1)
     * @return Transfer motor power
     */
    public double getTransferSpeed() {
        double area = getAprilTagArea();
        
        if (area >= HardwareConfigAuto.APRILTAG_AREA_CLOSE_THRESHOLD) {
            return HardwareConfigAuto.TRANSFER_POWER;
        } else {
            return HardwareConfigAuto.TRANSFER_POWER_FAR;
        }
    }
    
    /**
     * Check if shooter is at target speed (manual mode)
     * @param targetTPS Manual target speed
     * @return True if shooter is within tolerance of target
     */
    public boolean isShooterReady(double targetTPS) {
        double currentTPS = getShooterTPS();
        return Math.abs(currentTPS - targetTPS) <= HardwareConfigAuto.SHOOTER_READY_THRESHOLD;
    }
    
    /**
     * Set blocker servo position
     * @param blocked True for blocked state, false for unblocked
     */
    public void setBlocker(boolean blocked) {
        if (blocked) {
            robot.blockerServo.setPosition(HardwareConfigAuto.BLOCKER_BLOCKED);
        } else {
            robot.blockerServo.setPosition(HardwareConfigAuto.BLOCKER_UNBLOCKED);
        }
    }
    
    /**
     * Update light servo based on robot state with priority:
     * 1. Actively shooting (purple)
     * 2. Shooter ready (blue)
     * 3. Shooter off (green)
     * 
     * @param shooterRunning Whether shooter is currently running
     * @param shooterReady Whether shooter is at target speed
     * @param intakeRunning Whether intake system is actively running
     */
    public void updateLightServo(boolean shooterRunning, boolean shooterReady, boolean intakeRunning) {
        double targetColor;
        
        // Priority 1: Actively shooting (intake running with shooter on)
        if (shooterRunning && intakeRunning) {
            targetColor = HardwareConfigAuto.LIGHT_PURPLE;
        }
        // Priority 2: Shooter at correct speed
        else if (shooterRunning && shooterReady) {
            targetColor = HardwareConfigAuto.LIGHT_BLUE;
        }
        // Priority 3: Shooter spinning up
        else if (shooterRunning) {
            targetColor = HardwareConfigAuto.LIGHT_YELLOW;
        }
        // Default: Shooter off
        else {
            targetColor = HardwareConfigAuto.LIGHT_GREEN;
        }
        
        // Set the light servo to the target color (no flashing)
        robot.lightServo.setPosition(targetColor);
    }
    
    /**
     * Auto-align to target using Limelight tx value
     * Only aligns to AprilTag IDs 20 or 24
     * Calculates offset based on robot position on field
     * @param drive Forward/backward movement from driver
     * @param strafe Left/right movement from driver
     * @return The turn power being applied for alignment (for telemetry)
     */
    public double limelightAutoAlign(double drive, double strafe) {
        LLResult result = robot.limelight.getLatestResult();
        double turnPower = 0.0;
        
        if (result != null && result.isValid()) {
            // Check if we have fiducial (AprilTag) results
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                // Get the first detected fiducial
                LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
                int tagId = (int) fiducial.getFiducialId();
                
                // Only align to tags 20 or 24
                if (tagId == 20 || tagId == 24) {
            // Get horizontal offset (tx) from Limelight
            double tx = result.getTx();
                    
                    // Calculate target offset based on camera pitch angle
                    double targetOffset = calculateTargetOffset(tagId);
                    
                    // Calculate error (difference from target offset)
                    double error = tx - targetOffset;
            
            // Calculate turn power using proportional control
                    if (Math.abs(error) > HardwareConfigAuto.LIMELIGHT_TOLERANCE) {
                // Proportional control: turn power proportional to error
                        // Note: negative sign because positive error means we need to turn left (negative power)
                        turnPower = -error * HardwareConfigAuto.LIMELIGHT_KP;
                
                        // Apply min/max power limits with proper sign handling
                if (turnPower > 0) {
                            // Positive: ensure minimum power, then clamp to maximum
                    turnPower = Math.max(turnPower, HardwareConfigAuto.LIMELIGHT_MIN_POWER);
                    turnPower = Math.min(turnPower, HardwareConfigAuto.LIMELIGHT_MAX_POWER);
                } else if (turnPower < 0) {
                            // Negative: clamp to maximum magnitude first, then ensure minimum magnitude
                    turnPower = Math.max(turnPower, -HardwareConfigAuto.LIMELIGHT_MAX_POWER);
                            turnPower = Math.min(turnPower, -HardwareConfigAuto.LIMELIGHT_MIN_POWER);
                }
            }
                    // If within tolerance (abs(error) <= 2.0), turnPower stays 0
                }
            }
        }
        
        // Apply mecanum drive with calculated turn power
//        mecanumDrive(drive, strafe, turnPower, 1);
        
        return turnPower;
    }
    
    /**
     * Calculate target TX angle to aim at the goal position behind the AprilTag
     * Goal is 20" behind the AprilTag (in the tag's backward direction)
     * Works the same for both tag 20 (left side) and tag 24 (right side)
     * Uses 3D pose to naturally handle different tag positions and orientations
     * @param tagId The detected AprilTag ID (20 or 24)
     * @return Target tx offset in degrees
     */
    private double calculateTargetOffset(int tagId) {
        LLResult result = robot.limelight.getLatestResult();
        
        if (result == null || !result.isValid()) {
            return 0.0;
        }
        
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            return 0.0;
        }
        
        // Get the first detected fiducial
        LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
        
        // Get tag pose relative to camera
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D tagPose = fiducial.getTargetPoseCameraSpace();
        
        if (tagPose == null) {
            return 0.0;
        }
        
        // Tag position in camera space (millimeters)
        double tagX = tagPose.getPosition().x;  // Left/right from camera
        double tagY = tagPose.getPosition().y;  // Up/down from camera
        double tagZ = tagPose.getPosition().z;  // Forward/back from camera
        
        // Get tag orientation to find which way is "behind" the tag
        // The tag's yaw tells us which way it's facing
        double tagYaw = tagPose.getOrientation().getYaw(AngleUnit.RADIANS);
        
        // Goal is behind the tag in the direction the tag is facing
        double goalDistanceBehind = HardwareConfigAuto.APRILTAG_GOAL_OFFSET_MM;
        
        // Calculate goal position in camera space
        // The tag's forward direction is based on its yaw
        double goalX = tagX + goalDistanceBehind * Math.sin(tagYaw);
        double goalZ = tagZ + goalDistanceBehind * Math.cos(tagYaw);
        
        // Calculate the horizontal angle to the goal from the camera
        // TX is the horizontal offset angle
        double targetTx = Math.toDegrees(Math.atan2(goalX, goalZ));
        
        // No reversal needed - 3D calculation naturally handles both tags
        return targetTx;
    }
    
    /**
     * Get the current target offset (for telemetry/debugging)
     * @return Current target tx offset in degrees
     */
    public double getCurrentTargetOffset() {
        int tagId = getDetectedAprilTagId();
        if (tagId == 20 || tagId == 24) {
            return calculateTargetOffset(tagId);
        }
        return 0.0;
    }
    
    /**
     * Get the detected AprilTag ID (if any)
     * @return AprilTag ID, or -1 if no tag detected
     */
    public int getDetectedAprilTagId() {
        LLResult result = robot.limelight.getLatestResult();
        
        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
                return (int) fiducial.getFiducialId();
            }
        }
        
        return -1;  // No tag detected
    }
    
    /**
     * Get the AprilTag area (for data collection)
     * @return Target area percentage, or 0.0 if no target
     */
    public double getAprilTagArea() {
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTa();
        }
        return 0.0;
    }
    
    /**
     * Get the AprilTag's yaw angle from robot pose (robot relative to tag)
     * @return Tag yaw angle in degrees, or 0.0 if no tag detected
     */
    public double getAprilTagYaw() {
        LLResult result = robot.limelight.getLatestResult();
        
        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
                
                // Get robot space pose (robot's pose relative to tag)
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
                
                if (robotPose != null) {
                    return robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                }
            }
        }
        
        return 0.0;
    }
    
    /**
     * Get the AprilTag's roll angle from robot pose (robot relative to tag)
     * @return Tag roll angle in degrees, or 0.0 if no tag detected
     */
    public double getAprilTagRoll() {
        LLResult result = robot.limelight.getLatestResult();
        
        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
                
                if (robotPose != null) {
                    return robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
                }
            }
        }
        
        return 0.0;
    }
    
    /**
     * Get the AprilTag's pitch angle from robot pose (robot relative to tag)
     * @return Tag pitch angle in degrees, or 0.0 if no tag detected
     */
    public double getAprilTagPitch() {
        LLResult result = robot.limelight.getLatestResult();
        
        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
                
                if (robotPose != null) {
                    return robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
                }
            }
        }
        
        return 0.0;
    }
    
    /**
     * Get the AprilTag's yaw angle from camera space (tag relative to camera)
     * @return Tag yaw angle in degrees, or 0.0 if no tag detected
     */
    public double getAprilTagYawCameraSpace() {
        LLResult result = robot.limelight.getLatestResult();
        
        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
                
                // Get target pose in camera space (tag's pose relative to camera)
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D targetPose = fiducial.getTargetPoseCameraSpace();
                
                if (targetPose != null) {
                    return targetPose.getOrientation().getYaw(AngleUnit.DEGREES);
                }
            }
        }
        
        return 0.0;
    }
    
    /**
     * Get the AprilTag's roll angle from camera space (tag relative to camera)
     * @return Tag roll angle in degrees, or 0.0 if no tag detected
     */
    public double getAprilTagRollCameraSpace() {
        LLResult result = robot.limelight.getLatestResult();
        
        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D targetPose = fiducial.getTargetPoseCameraSpace();
                
                if (targetPose != null) {
                    return targetPose.getOrientation().getRoll(AngleUnit.DEGREES);
                }
            }
        }
        
        return 0.0;
    }
    
    /**
     * Get the AprilTag's pitch angle from camera space (tag relative to camera)
     * @return Tag pitch angle in degrees, or 0.0 if no tag detected
     */
    public double getAprilTagPitchCameraSpace() {
        LLResult result = robot.limelight.getLatestResult();
        
        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D targetPose = fiducial.getTargetPoseCameraSpace();
                
                if (targetPose != null) {
                    return targetPose.getOrientation().getPitch(AngleUnit.DEGREES);
                }
            }
        }
        
        return 0.0;
    }
    
    /**
     * Get the distance to the AprilTag
     * @return Distance in millimeters, or 0.0 if no tag detected
     */
    public double getAprilTagDistance() {
        LLResult result = robot.limelight.getLatestResult();
        
        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
                
                // Get robot space pose (tag's position relative to robot)
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
                
                if (robotPose != null) {
                    // Calculate distance using X and Z (forward distance)
                    double x = robotPose.getPosition().x;  // Side to side
                    double y = robotPose.getPosition().y;  // Up/down
                    double z = robotPose.getPosition().z;  // Forward/back
                    
                    // Return 3D distance
                    return Math.sqrt(x * x + y * y + z * z);
                }
            }
        }
        
        return 0.0;  // No tag detected
    }
    
    /**
     * Get Limelight tx value (horizontal offset)
     * @return tx value in degrees, or 0.0 if no valid target
     */
    public double getLimelightTx() {
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0.0;
    }
    
    /**
     * Get Limelight ty value (vertical offset)
     * @return ty value in degrees, or 0.0 if no valid target
     */
    public double getLimelightTy() {
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTy();
        }
        return 0.0;
    }
    
    /**
     * Check if Limelight has a valid target
     * @return True if target is detected
     */
    public boolean hasLimelightTarget() {
        LLResult result = robot.limelight.getLatestResult();
        return result != null && result.isValid();
    }
    
    // ========== INDEXING FUNCTIONS ==========
    
    // Indexing servo positions
    private static final double INDEXER_INDEXED_POSITION = 0.4;
    private static final double INDEXER_MIDDLE_POSITION = 0.94;
    
    /**
     * Move indexing servo to indexed position (ball is moved out of the way)
     */
    public void setIndexerIndexed() {
        robot.indexingServo.setPosition(INDEXER_INDEXED_POSITION);
    }
    
    /**
     * Move indexing servo to middle position (ball is ready to shoot)
     */
    public void setIndexerMiddle() {
        robot.indexingServo.setPosition(INDEXER_MIDDLE_POSITION);
    }
    
    /**
     * Get current indexer position
     * @return Current servo position
     */
    public double getIndexerPosition() {
        return robot.indexingServo.getPosition();
    }
    
    /**
     * Check if indexer is at middle position
     * @return True if at middle position
     */
    public boolean isIndexerAtMiddle() {
        double currentPos = robot.indexingServo.getPosition();
        return Math.abs(currentPos - INDEXER_MIDDLE_POSITION) < 0.05;
    }
    
    /**
     * Check if either gamepad has a button pressed
     * @param gamepad1 First gamepad
     * @param gamepad2 Second gamepad
     * @param buttonName Name of the button to check
     * @return True if either gamepad has the button pressed
     */
    public boolean eitherGamepad(Gamepad gamepad1, Gamepad gamepad2, String buttonName) {
        boolean pad1 = false;
        boolean pad2 = false;
        
        switch (buttonName.toLowerCase()) {
            case "a":
                pad1 = gamepad1.a;
                pad2 = gamepad2.a;
                break;
            case "b":
                pad1 = gamepad1.b;
                pad2 = gamepad2.b;
                break;
            case "x":
                pad1 = gamepad1.x;
                pad2 = gamepad2.x;
                break;
            case "y":
                pad1 = gamepad1.y;
                pad2 = gamepad2.y;
                break;
            case "left_bumper":
                pad1 = gamepad1.left_bumper;
                pad2 = gamepad2.left_bumper;
                break;
            case "right_bumper":
                pad1 = gamepad1.right_bumper;
                pad2 = gamepad2.right_bumper;
                break;
        }
        
        return pad1 || pad2;
    }
    
    // ========== PINPOINT ODOMETRY FUNCTIONS ==========
    
    /**
     * Update Pinpoint odometry (should be called regularly in loop)
     * Reads latest position data from the Pinpoint odometry computer
     */
    public void updateOdometry() {
        robot.pinpoint.update();
    }
    
    /**
     * Get X position in millimeters from Pinpoint
     * @return X coordinate
     */
    public double getOdometryX() {
        return robot.pinpoint.getPosX(DistanceUnit.MM);
    }
    
    /**
     * Get Y position in millimeters from Pinpoint
     * @return Y coordinate
     */
    public double getOdometryY() {
        return robot.pinpoint.getPosY(DistanceUnit.MM);
    }
    
    /**
     * Get heading in degrees from Control Hub IMU (more stable than Pinpoint IMU)
     * @return Heading angle in degrees (-180 to 180)
     */
    public double getOdometryHeading() {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    
    /**
     * Get heading from Pinpoint IMU (for comparison/debugging)
     * @return Heading angle in degrees
     */
    public double getPinpointHeading() {
        return robot.pinpoint.getHeading(AngleUnit.DEGREES);
    }
    
    /**
     * Reset odometry position to origin (0, 0, 0) and reset IMU heading
     * Robot must be stationary during this call!
     */
    public void resetOdometry() {
        robot.pinpoint.resetPosAndIMU();
        robot.imu.resetYaw();  // Also reset Control Hub IMU
    }
    
    /**
     * Reset only the IMU heading (for drift correction)
     * Robot must be stationary!
     */
    public void recalibrateIMU() {
        robot.imu.resetYaw();  // Reset Control Hub IMU heading
    }
    
    /**
     * Reset only position to origin without affecting IMU calibration
     */
    public void resetPosition() {
        robot.pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
    }
    
    /**
     * Set odometry position to specific values
     * @param x X coordinate in millimeters
     * @param y Y coordinate in millimeters  
     * @param heading Heading in degrees
     */
    public void setOdometryPosition(double x, double y, double heading) {
        robot.pinpoint.setPosition(new Pose2D(DistanceUnit.MM, x, y, AngleUnit.DEGREES, heading));
    }
    
    /**
     * Get the Pinpoint device status for debugging
     * @return Device status string
     */
    public String getPinpointStatus() {
        return robot.pinpoint.getDeviceStatus().toString();
    }
    
    /**
     * Get the Pinpoint yaw scalar (for debugging heading drift)
     * @return Yaw scalar value
     */
    public double getPinpointYawScalar() {
        return robot.pinpoint.getYawScalar();
    }
    
    /**
     * Get the Pinpoint update frequency
     * @return Frequency in Hz
     */
    public double getPinpointFrequency() {
        return robot.pinpoint.getFrequency();
    }
    
    // ========== ENCODER-BASED AUTONOMOUS DRIVE FUNCTIONS ==========
    
    /**
     * Drive forward/backward for a specific distance using encoders
     * @param distanceMM Distance to drive in millimeters (positive = forward, negative = backward)
     * @param power Power level (0.0 to 1.0)
     */
//    public void driveDistance(double distanceMM, double power) {
//        int targetCounts = (int)(distanceMM * HardwareConfigAuto.COUNTS_PER_MM);
//
//        // Set target positions for all motors
//        int flTarget = robot.frontLeft.getCurrentPosition() + targetCounts;
//        int frTarget = robot.frontRight.getCurrentPosition() + targetCounts;
//        int blTarget = robot.backLeft.getCurrentPosition() + targetCounts;
//        int brTarget = robot.backRight.getCurrentPosition() + targetCounts;
//
//        robot.frontLeft.setTargetPosition(flTarget);
//        robot.frontRight.setTargetPosition(frTarget);
//        robot.backLeft.setTargetPosition(blTarget);
//        robot.backRight.setTargetPosition(brTarget);
//
//        // Switch to RUN_TO_POSITION mode
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // Set power
//        robot.frontLeft.setPower(Math.abs(power));
//        robot.frontRight.setPower(Math.abs(power));
//        robot.backLeft.setPower(Math.abs(power));
//        robot.backRight.setPower(Math.abs(power));
//    }
    
    /**
     * Strafe left/right for a specific distance using encoders
     * @param distanceMM Distance to strafe in millimeters (positive = right, negative = left)
     * @param power Power level (0.0 to 1.0)
     */
//    public void strafeDistance(double distanceMM, double power) {
//        int targetCounts = (int)(distanceMM * HardwareConfigAuto.COUNTS_PER_MM);
//
//        // Mecanum strafe: FL and BR go one way, FR and BL go opposite
//        int flTarget = robot.frontLeft.getCurrentPosition() + targetCounts;
//        int frTarget = robot.frontRight.getCurrentPosition() - targetCounts;
//        int blTarget = robot.backLeft.getCurrentPosition() - targetCounts;
//        int brTarget = robot.backRight.getCurrentPosition() + targetCounts;
//
//        robot.frontLeft.setTargetPosition(flTarget);
//        robot.frontRight.setTargetPosition(frTarget);
//        robot.backLeft.setTargetPosition(blTarget);
//        robot.backRight.setTargetPosition(brTarget);
//
//        // Switch to RUN_TO_POSITION mode
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // Set power
//        robot.frontLeft.setPower(Math.abs(power));
//        robot.frontRight.setPower(Math.abs(power));
//        robot.backLeft.setPower(Math.abs(power));
//        robot.backRight.setPower(Math.abs(power));
//    }
    
    /**
     * Rotate to a target heading using IMU
     * @param targetHeading Target heading in degrees
     * @param power Power level (0.0 to 1.0)
     * @return True if at target heading
     */
//    public boolean rotateToHeading(double targetHeading, double power) {
//        double currentHeading = getOdometryHeading();
//        double errorHeading = normalizeAngle(targetHeading - currentHeading);
//
//        // Check if at target
//        if (Math.abs(errorHeading) < HardwareConfigAuto.DRIVE_HEADING_TOLERANCE) {
////            mecanumDrive(0, 0, 0, 1.0);
//            return true;
//        }
//
//        // Determine rotation direction
//        double turnPower = (errorHeading > 0) ? Math.abs(power) : -Math.abs(power);
//
//        mecanumDrive(0, 0, turnPower, 1.0);
//
//        return false;
//    }
    
    /**
     * Check if drivetrain motors are busy (for RUN_TO_POSITION mode)
     * @return True if any motor is still busy
     */
//    public boolean isDriveBusy() {
//        return robot.frontLeft.isBusy() || robot.frontRight.isBusy() ||
//               robot.backLeft.isBusy() || robot.backRight.isBusy();
//    }
    
    /**
     * Check if drivetrain is busy and update shooter control
     * Use this in loops to keep shooter running during autonomous movements
     * @param useManualSpeed Whether to use manual speed control
     * @param manualTPS Manual target TPS (only used if useManualSpeed is true)
     * @return True if drivetrain is still busy
     */
//    public boolean isDriveBusyWithShooter(boolean useManualSpeed, double manualTPS) {
//        // Update shooter control
//        if (useManualSpeed) {
//            controlShooterManual(manualTPS, true);
//        } else {
//            controlShooter(true);
//        }
//
//        // Update light servo (intake not running during positioning)
//        boolean shooterReady = useManualSpeed ? isShooterReady(manualTPS) : isShooterReady();
//        updateLightServo(true, shooterReady, false);
//
//        // Return drive busy status
//        return isDriveBusy();
//    }
    
    /**
     * Update shooter control for autonomous shooting loops
     * Call this repeatedly while shooting to maintain shooter speed
     * @param useManualSpeed Whether to use manual speed control
     * @param manualTPS Manual target TPS (only used if useManualSpeed is true)
     */
    public void updateShooterControl(boolean useManualSpeed, double manualTPS) {
        // Update shooter control
        if (useManualSpeed) {
            controlShooterManual(manualTPS, true);
        } else {
            controlShooter(true);
        }
        
        // Update light servo (intake not running during this call)
        boolean shooterReady = useManualSpeed ? isShooterReady(manualTPS) : isShooterReady();
        updateLightServo(true, shooterReady, false);
    }
    
    /**
     * Stop and switch drivetrain back to RUN_WITHOUT_ENCODER mode
     */
//    public void stopDriveAndReset() {
//        // Stop all motors
//        robot.frontLeft.setPower(0);
//        robot.frontRight.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.backRight.setPower(0);
//
//        // Switch back to RUN_WITHOUT_ENCODER for teleop
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
    
    /**
     * Get heading error to target
     * @param targetHeading Target heading in degrees
     * @return Heading error in degrees (-180 to 180)
     */
    public double getHeadingError(double targetHeading) {
        double currentHeading = getOdometryHeading();
        return normalizeAngle(targetHeading - currentHeading);
    }
    
    /**
     * Normalize angle to -180 to 180 degrees
     * @param angle Angle in degrees
     * @return Normalized angle
     */
    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
    
    // ========== TURRET FUNCTIONS ==========
    
    /**
     * Convert turret angle to servo position
     * Uses calibrated values for accurate positioning
     * Clamps angle to valid range before conversion
     * @param turretAngle Desired turret angle in degrees (0 = center, positive = right, negative = left)
     * @return Servo position (0.0 to 1.0)
     */
    public double turretAngleToServoPosition(double turretAngle) {
        // Clamp turret angle to valid range first
        double clampedAngle = clampTurretAngle(turretAngle);
        
        // Use calibrated center position and degrees per servo unit
        double servoPosition = HardwareConfigAuto.TURRET_CENTER_POSITION + (clampedAngle / HardwareConfigAuto.TURRET_DEGREES_PER_SERVO_UNIT);
        
        // Clamp to valid servo range (0.0 to 1.0) as safety
        if (servoPosition < 0.0) {
            servoPosition = 0.0;
        } else if (servoPosition > 1.0) {
            servoPosition = 1.0;
        }
        
        return servoPosition;
    }
    
    /**
     * Clamp turret angle to valid physical range (-90 to +90 degrees)
     * @param angle Requested angle in degrees
     * @return Clamped angle within valid range
     */
    public double clampTurretAngle(double angle) {
        if (angle < HardwareConfigAuto.TURRET_MIN_ANGLE) {
            return HardwareConfigAuto.TURRET_MIN_ANGLE;
        } else if (angle > HardwareConfigAuto.TURRET_MAX_ANGLE) {
            return HardwareConfigAuto.TURRET_MAX_ANGLE;
        }
        return angle;
    }
    
    /**
     * Set turret to a specific angle
     * @param turretAngle Desired turret angle in degrees (0 = center, positive = clockwise, negative = counterclockwise)
     */
    public void setTurretAngle(double turretAngle) {
        double servoPosition = turretAngleToServoPosition(turretAngle);
        robot.turretServo.setPosition(servoPosition);
    }
    
    /**
     * Set turret to home/center position (0 degrees)
     */
    public void setTurretHome() {
        robot.turretServo.setPosition(HardwareConfigAuto.TURRET_CENTER_POSITION);
    }
    
    /**
     * Set turret to 90 degrees (right)
     */
    public void setTurret90() {
        robot.turretServo.setPosition(HardwareConfigAuto.TURRET_90_POSITION);
    }
    
    /**
     * Set turret to -90 degrees (left)
     */
    public void setTurretNeg90() {
        robot.turretServo.setPosition(HardwareConfigAuto.TURRET_NEG90_POSITION);
    }
    
    /**
     * Get current turret servo position
     * @return Current servo position (0.0 to 1.0)
     */
    public double getTurretServoPosition() {
        return robot.turretServo.getPosition();
    }
    
    /**
     * Calculate current turret angle from servo position
     * @return Estimated turret angle in degrees
     */
    public double getTurretAngle() {
        double servoPosition = robot.turretServo.getPosition();
        double turretAngle = (servoPosition - HardwareConfigAuto.TURRET_CENTER_POSITION) * HardwareConfigAuto.TURRET_DEGREES_PER_SERVO_UNIT;
        return turretAngle;
    }
    
    /**
     * Calculate turret angle needed to point at a goal position
     * @param goalX Goal X position in mm
     * @param goalY Goal Y position in mm
     * @return Turret angle in degrees (clamped to valid range)
     */
    public double calculateTurretAngleToGoal(double goalX, double goalY) {
        // Get current robot position and heading from Pinpoint
        double robotX = getOdometryX();
        double robotY = getOdometryY();
        double robotHeading = getOdometryHeading();
        
        // Calculate angle from robot to goal (field-relative)
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));
        
        // Convert to turret angle (negated for correct servo direction)
        // Turret 0° points backward (180° from robot front)
        // Blue goal (right, Y-) → turret needs to turn right → negative angle
        // Red goal (left, Y+) → turret needs to turn left → positive angle
        double turretAngle = 180.0 + robotHeading - fieldAngleToGoal;
        
        // Normalize to -180 to 180
        while (turretAngle > 180.0) turretAngle -= 360.0;
        while (turretAngle < -180.0) turretAngle += 360.0;
        
        // Clamp to turret limits
        return clampTurretAngle(turretAngle);
    }
    
    /**
     * Point turret at goal based on alliance color
     * @param isRedAlliance True for red alliance, false for blue
     */
    public void pointTurretAtGoal(boolean isRedAlliance) {
        double goalX = HardwareConfigAuto.GOAL_X;
        double goalY = isRedAlliance ? HardwareConfigAuto.GOAL_Y_RED : HardwareConfigAuto.GOAL_Y_BLUE;
        
        double turretAngle = calculateTurretAngleToGoal(goalX, goalY);
        setTurretAngle(turretAngle);
    }
    
    /**
     * Get the field-relative angle to the goal (for telemetry)
     * @param isRedAlliance True for red alliance, false for blue
     * @return Angle to goal in degrees
     */
    public double getAngleToGoal(boolean isRedAlliance) {
        double goalX = HardwareConfigAuto.GOAL_X;
        double goalY = isRedAlliance ? HardwareConfigAuto.GOAL_Y_RED : HardwareConfigAuto.GOAL_Y_BLUE;
        
        double robotX = getOdometryX();
        double robotY = getOdometryY();
        
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;
        
        return Math.toDegrees(Math.atan2(deltaY, deltaX));
    }
    
    /**
     * Get the distance to the goal (for telemetry)
     * @param isRedAlliance True for red alliance, false for blue
     * @return Distance to goal in mm
     */
    public double getDistanceToGoal(boolean isRedAlliance) {
        double goalX = HardwareConfigAuto.GOAL_X;
        double goalY = isRedAlliance ? HardwareConfigAuto.GOAL_Y_RED : HardwareConfigAuto.GOAL_Y_BLUE;
        
        double robotX = getOdometryX();
        double robotY = getOdometryY();
        
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;
        
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    
    /**
     * Reset robot X/Y position based on Limelight AprilTag camera-relative pose
     * Uses known AprilTag field positions and camera-relative distance to calculate robot position
     * Computes field-relative heading from AprilTag orientation for position calculation
     * Does NOT reset heading - we trust Control Hub IMU more for ongoing heading tracking
     * @param isRedAlliance True for red alliance, false for blue
     * @return True if position was reset, false if no valid tag detected
     */
    public boolean resetPositionFromAprilTag(boolean isRedAlliance) {
        // Check for correct AprilTag based on alliance
        // Blue alliance uses tag 20, Red alliance uses tag 24
        int expectedTagId = isRedAlliance ? 24 : 20;
        int detectedTagId = getDetectedAprilTagId();
        
        if (detectedTagId != expectedTagId) {
            return false;  // Wrong tag or no tag
        }
        
        // Get robot pose from Limelight
        LLResult result = robot.limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return false;
        }
        
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            return false;
        }
        
        // Get the fiducial result
        LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
        
        // Get tag pose in CAMERA space (tag position relative to camera)
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D tagPoseCameraSpace = fiducial.getTargetPoseCameraSpace();
        
        if (tagPoseCameraSpace == null) {
            return false;
        }
        
        // Get tag position relative to camera (Limelight returns METERS, convert to mm)
        double tagX_cam = tagPoseCameraSpace.getPosition().x * 1000.0;   // Tag left/right from camera (mm)
        double tagZ_cam = tagPoseCameraSpace.getPosition().z * 1000.0;   // Tag forward from camera (mm)
        
        // Get tag yaw in camera space (degrees)
        double tagYaw_cam = tagPoseCameraSpace.getOrientation().getYaw(AngleUnit.DEGREES);
        
        // Get known AprilTag field position and facing direction
        double tagFieldX, tagFieldY;
        double tagFieldFacing;  // Direction the tag faces on the field (degrees)
        
        if (expectedTagId == 20) {
            tagFieldX = HardwareConfigAuto.APRILTAG_20_X;
            tagFieldY = HardwareConfigAuto.APRILTAG_20_Y;
            tagFieldFacing = HardwareConfigAuto.APRILTAG_20_FACING;
        } else {
            tagFieldX = HardwareConfigAuto.APRILTAG_24_X;
            tagFieldY = HardwareConfigAuto.APRILTAG_24_Y;
            tagFieldFacing = HardwareConfigAuto.APRILTAG_24_FACING;
        }
        
        // Compute robot's field-relative heading from the AprilTag's orientation
        double fieldHeading = tagFieldFacing + 180.0 - tagYaw_cam;
        
        // Normalize to -180 to 180
        while (fieldHeading > 180.0) fieldHeading -= 360.0;
        while (fieldHeading < -180.0) fieldHeading += 360.0;
        
        double headingRad = Math.toRadians(fieldHeading);
        
        // Transform camera-relative tag position to field coordinates
        double robotX = tagFieldX - (tagZ_cam * Math.cos(headingRad) - tagX_cam * Math.sin(headingRad));
        double robotY = tagFieldY - (tagZ_cam * Math.sin(headingRad) + tagX_cam * Math.cos(headingRad));
        
        // Keep current heading from Control Hub IMU (don't reset it)
        double imuHeading = getOdometryHeading();
        
        // Set Pinpoint position to calculated field position (keep IMU heading)
        robot.pinpoint.setPosition(new Pose2D(DistanceUnit.MM, robotX, robotY, AngleUnit.DEGREES, imuHeading));
        
        return true;
    }
    
    /**
     * Check if the correct AprilTag for the alliance is visible
     * @param isRedAlliance True for red alliance, false for blue
     * @return True if the correct tag is visible
     */
    public boolean isCorrectAprilTagVisible(boolean isRedAlliance) {
        int expectedTagId = isRedAlliance ? 24 : 20;
        int detectedTagId = getDetectedAprilTagId();
        return detectedTagId == expectedTagId;
    }
    
    // Cached values for telemetry display
    private double lastLimelightEstimatedX = 0.0;
    private double lastLimelightEstimatedY = 0.0;
    private double lastLimelightTagX = 0.0;
    private double lastLimelightTagZ = 0.0;
    private double lastLimelightTagYaw = 0.0;
    private double lastComputedFieldHeading = 0.0;
    private boolean lastLimelightPoseValid = false;
    
    // Heading stability tracking for position reset
    private double headingStabilityStartValue = 0.0;
    private long headingStabilityStartTime = 0;
    private boolean isHeadingStable = false;
    private static final double HEADING_STABILITY_THRESHOLD = 1.0;  // degrees
    private static final long HEADING_STABILITY_TIME_MS = 500;      // 0.5 seconds
    
    /**
     * Get estimated robot X position from Limelight (for telemetry)
     * @return Estimated X position in mm
     */
    public double getLimelightEstimatedX() {
        return lastLimelightEstimatedX;
    }
    
    /**
     * Get estimated robot Y position from Limelight (for telemetry)
     * @return Estimated Y position in mm
     */
    public double getLimelightEstimatedY() {
        return lastLimelightEstimatedY;
    }
    
    /**
     * Get tag X position in camera space (for debugging)
     * @return Tag X in mm (right is positive)
     */
    public double getLimelightTagX() {
        return lastLimelightTagX;
    }
    
    /**
     * Get tag Z position in camera space (for debugging)
     * @return Tag Z in mm (forward is positive)
     */
    public double getLimelightTagZ() {
        return lastLimelightTagZ;
    }
    
    /**
     * Get tag yaw in camera space (for debugging)
     * @return Tag yaw in degrees
     */
    public double getLimelightTagYaw() {
        return lastLimelightTagYaw;
    }
    
    /**
     * Get computed field-relative heading from AprilTag (for debugging)
     * @return Field heading in degrees
     */
    public double getComputedFieldHeading() {
        return lastComputedFieldHeading;
    }
    
    /**
     * Check if last Limelight pose was valid
     * @return True if valid pose data was available
     */
    public boolean isLimelightPoseValid() {
        return lastLimelightPoseValid;
    }
    
    /**
     * Update heading stability tracking
     * Call this every loop to track if robot is stationary
     */
    public void updateHeadingStability() {
        double currentHeading = getOdometryHeading();
        long currentTime = System.currentTimeMillis();
        
        // Check if heading has changed more than threshold
        double headingChange = Math.abs(currentHeading - headingStabilityStartValue);
        // Handle wraparound
        if (headingChange > 180.0) {
            headingChange = 360.0 - headingChange;
        }
        
        if (headingChange > HEADING_STABILITY_THRESHOLD) {
            // Heading changed too much, reset stability tracking
            headingStabilityStartValue = currentHeading;
            headingStabilityStartTime = currentTime;
            isHeadingStable = false;
        } else {
            // Heading is stable, check if enough time has passed
            if (currentTime - headingStabilityStartTime >= HEADING_STABILITY_TIME_MS) {
                isHeadingStable = true;
            }
        }
    }
    
    /**
     * Check if robot heading has been stable (not moving) for 0.5 seconds
     * @return True if heading hasn't changed more than 1 degree in last 0.5 seconds
     */
    public boolean isRobotStationary() {
        return isHeadingStable;
    }
    
    /**
     * Calculate estimated robot position from AprilTag (for telemetry display)
     * Does NOT reset the actual position - just calculates what it would be
     * Uses AprilTag orientation to compute field-relative heading instead of IMU
     * @param isRedAlliance True for red alliance, false for blue
     */
    public void updateLimelightEstimatedPosition(boolean isRedAlliance) {
        lastLimelightPoseValid = false;
        
        // Check for any AprilTag (20 or 24)
        int detectedTagId = getDetectedAprilTagId();
        if (detectedTagId != 20 && detectedTagId != 24) {
            return;
        }
        
        LLResult result = robot.limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }
        
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            return;
        }
        
        LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D tagPoseCameraSpace = fiducial.getTargetPoseCameraSpace();
        
        if (tagPoseCameraSpace == null) {
            return;
        }
        
        // Get tag position relative to camera (Limelight returns METERS, convert to mm)
        lastLimelightTagX = tagPoseCameraSpace.getPosition().x * 1000.0;
        lastLimelightTagZ = tagPoseCameraSpace.getPosition().z * 1000.0;
        
        // Get tag yaw in camera space (degrees)
        lastLimelightTagYaw = tagPoseCameraSpace.getOrientation().getYaw(AngleUnit.DEGREES);
        
        // Get known AprilTag field position and facing direction
        double tagFieldX, tagFieldY;
        double tagFieldFacing;  // Direction the tag faces on the field (degrees)
        
        if (detectedTagId == 20) {
            tagFieldX = HardwareConfigAuto.APRILTAG_20_X;
            tagFieldY = HardwareConfigAuto.APRILTAG_20_Y;
            tagFieldFacing = HardwareConfigAuto.APRILTAG_20_FACING;
        } else {
            tagFieldX = HardwareConfigAuto.APRILTAG_24_X;
            tagFieldY = HardwareConfigAuto.APRILTAG_24_Y;
            tagFieldFacing = HardwareConfigAuto.APRILTAG_24_FACING;
        }
        
        // Compute robot's field-relative heading from the AprilTag's orientation
        // If tag yaw in camera = 0, tag is facing directly at camera
        // Robot heading = tag's field facing direction + 180° - tag yaw in camera
        lastComputedFieldHeading = tagFieldFacing + 180.0 - lastLimelightTagYaw;
        
        // Normalize to -180 to 180
        while (lastComputedFieldHeading > 180.0) lastComputedFieldHeading -= 360.0;
        while (lastComputedFieldHeading < -180.0) lastComputedFieldHeading += 360.0;
        
        double headingRad = Math.toRadians(lastComputedFieldHeading);
        
        // Calculate estimated robot position using the computed field heading
        lastLimelightEstimatedX = tagFieldX - (lastLimelightTagZ * Math.cos(headingRad) - lastLimelightTagX * Math.sin(headingRad));
        lastLimelightEstimatedY = tagFieldY - (lastLimelightTagZ * Math.sin(headingRad) + lastLimelightTagX * Math.cos(headingRad));
        
        lastLimelightPoseValid = true;
    }

    /**
     * Shoot a single ball (autonomous sequence)
     * Uses limelight for shooter speed and drivetrain alignment
     * @param opMode The OpMode calling this (for sleep)
     * @throws InterruptedException If sleep is interrupted
     */
    public void shootSingleBall(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) throws InterruptedException {
        // Step 1 & 2: Spin up flywheel and align drivetrain
        // Continuously update shooter control and alignment until ready
        // Timeout after 3 seconds to prevent infinite loop if no target
        long startTime = System.currentTimeMillis();
        long timeout = 500;  // 3 second timeout
        
        while (opMode.opModeIsActive()) {
            // Update shooter control (this adjusts power based on current speed)
            controlShooter(true);
            
            // Auto-align drivetrain to limelight target (drive=0, strafe=0)
            //limelightAutoAlign(0, 0);
            
            // Check if shooter is ready (includes speed and alignment check within 5°)
            if (isShooterReady()) {
                break;  // Both shooter speed and alignment are good
            }
            
            // Timeout check - if no target visible, just check shooter speed is ready
            if (System.currentTimeMillis() - startTime > timeout) {
                // Fallback: just check if shooter speed is at default TPS
                double currentTPS = getShooterTPS();
                if (Math.abs(currentTPS - HardwareConfigAuto.SHOOTER_DEFAULT_TPS) <= HardwareConfigAuto.SHOOTER_READY_THRESHOLD) {
                    break;  // Shooter is at default speed, proceed anyway
                }
            }
            
            opMode.sleep(10);
        }
        
        // Step 3: Unblock
        setBlocker(false);
        opMode.sleep(100);  // Small delay for servo movement
        
        // Step 4: Run transfer motor for 250 ticks using dynamic speed based on distance
        // Reset encoder and use RUN_TO_POSITION mode
        int startPosition = robot.transferMotor.getCurrentPosition();
        int targetPosition = startPosition + 300;
        
        robot.transferMotor.setTargetPosition(targetPosition);
        robot.transferMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.transferMotor.setPower(Math.abs(0.7));  // Use dynamic speed (absolute value for RUN_TO_POSITION)
        
        // Also run intake at 100%
        robot.intakeMotor.setPower(HardwareConfigAuto.INTAKE_POWER);
        
        // Wait for transfer motor to reach position while maintaining shooter speed
        while (opMode.opModeIsActive() && robot.transferMotor.isBusy()) {
            // Keep shooter running at correct speed while feeding ball
            controlShooter(true);
            opMode.sleep(10);
        }
        
        // Stop intake/transfer
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        
        // Switch transfer motor back to RUN_WITHOUT_ENCODER
        robot.transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Block again
        setBlocker(true);

        // Stop shooter
        controlShooter(false);
        
        // Step 5: Run transfer for 0.5 seconds to load next ball into high sensor
        robot.transferMotor.setPower(1.0);
        robot.intakeMotor.setPower(1.0);
        //opMode.sleep(500);
        
        // Stop intake/transfer
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
    }
    
    /**
     * Shoot a single ball while keeping shooter spinning (for multi-shot sequences)
     * Assumes shooter is already spinning - maintains speed throughout
     * Does NOT stop shooter at the end
     * @param opMode The OpMode calling this (for sleep)
     * @throws InterruptedException If sleep is interrupted
     */
    public void shootSingleBallContinuous(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) throws InterruptedException {
        // Wait for alignment (shooter should already be spinning)
        // Timeout after 3 seconds to prevent infinite loop if no target
        long startTime = System.currentTimeMillis();
        long timeout = 3000;  // 3 second timeout
        
        while (opMode.opModeIsActive()) {
            // Keep shooter running at correct speed
            controlShooter(true);
            
            // Auto-align drivetrain to limelight target (drive=0, strafe=0)
            limelightAutoAlign(0, 0);
            
            // Check if shooter is ready (includes speed and alignment check within 5°)
            if (isShooterReady()) {
                break;  // Both shooter speed and alignment are good
            }
            
            // Timeout check - if no target visible, just check shooter speed is ready
            if (System.currentTimeMillis() - startTime > timeout) {
                // Fallback: just check if shooter speed is at default TPS
                double currentTPS = getShooterTPS();
                if (Math.abs(currentTPS - HardwareConfigAuto.SHOOTER_DEFAULT_TPS) <= HardwareConfigAuto.SHOOTER_READY_THRESHOLD) {
                    break;  // Shooter is at default speed, proceed anyway
                }
            }
            
            opMode.sleep(10);
        }
        
        // Unblock
        setBlocker(false);
        opMode.sleep(100);  // Small delay for servo movement
        
        // Run transfer motor for 250 ticks
        int startPosition = robot.transferMotor.getCurrentPosition();
        int targetPosition = startPosition + 250;
        
        robot.transferMotor.setTargetPosition(targetPosition);
        robot.transferMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.transferMotor.setPower(Math.abs(getTransferSpeed()));
        
        // Also run intake at 100%
        robot.intakeMotor.setPower(HardwareConfigAuto.INTAKE_POWER);
        
        // Wait for transfer motor to reach position while maintaining shooter speed
        while (opMode.opModeIsActive() && robot.transferMotor.isBusy()) {
            // Keep shooter running at correct speed while feeding ball
            controlShooter(true);
            opMode.sleep(10);
        }
        
        // Stop intake/transfer
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        
        // Switch transfer motor back to RUN_WITHOUT_ENCODER
        robot.transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Block again
        setBlocker(true);
        

        // NOTE: Shooter stays ON - do not call controlShooter(false)
    }
    
    /**
     * SSS - Shoot all three balls in sequence
     * Shooter stays spinning throughout all 3 shots for faster cycle time
     * During the delay, transfer runs with blocker closed to load the next ball
     * @param opMode The OpMode calling this (for sleep)
     * @throws InterruptedException If sleep is interrupted
     */
    public void SSS(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) throws InterruptedException {
        // Start shooter spinning immediately (will stay on throughout)
        controlShooter(true);
        
        // Shoot ball 1 (keeps shooter spinning)
        shootSingleBall(opMode);
        
        // 0.1 second delay with transfer running and blocker closed
        // Keep shooter spinning during delay
        setBlocker(true);
        robot.transferMotor.setPower(HardwareConfigAuto.TRANSFER_POWER);
        robot.intakeMotor.setPower(HardwareConfigAuto.INTAKE_POWER);
        long delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 100) {
            controlShooter(true);  // Keep shooter at speed
            opMode.sleep(10);
        }
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        
        // Shoot ball 2 (keeps shooter spinning)
        shootSingleBall(opMode);
        
        // 0.1 second delay with transfer running and blocker closed
        setBlocker(true);
        robot.transferMotor.setPower(HardwareConfigAuto.TRANSFER_POWER);
        robot.intakeMotor.setPower(HardwareConfigAuto.INTAKE_POWER);
        delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 100) {
            controlShooter(true);  // Keep shooter at speed
            opMode.sleep(10);
        }
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        
        // Shoot ball 3 (keeps shooter spinning)
        shootSingleBall(opMode);
        
        // Final cleanup - NOW stop the shooter
        setBlocker(true);
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        controlShooter(false);

    }
    
    /**
     * SISIS - Shoot, Index, Shoot, Index, Shoot sequence
     * Shooter stays spinning throughout all shots for faster cycle time
     * @param opMode The OpMode calling this (for sleep)
     * @throws InterruptedException If sleep is interrupted
     */
    public void SISIS(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) throws InterruptedException {
        // Start shooter spinning immediately (will stay on throughout)
        controlShooter(true);
        
        // Step 1: Shoot first ball (keeps shooter spinning)
        shootSingleBall(opMode);
        
        // Step 2: Run transfer + blocker closed for 0.2 seconds (keep shooter spinning)
        setBlocker(true);
        robot.transferMotor.setPower(HardwareConfigAuto.TRANSFER_POWER);
        robot.intakeMotor.setPower(HardwareConfigAuto.INTAKE_POWER);
        long delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 200) {
            controlShooter(true);
        }
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);

        //moveTransferBallBackSmall(opMode);
        
        // Step 3: Set indexer to indexed position (keep shooter spinning)
        setIndexerIndexed();
        delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 100) {
            controlShooter(true);
        }
        
        // Step 4: Run transfer + blocker closed for 0.2 seconds
        setBlocker(true);
        robot.transferMotor.setPower(HardwareConfigAuto.TRANSFER_POWER);
        robot.intakeMotor.setPower(HardwareConfigAuto.INTAKE_POWER);
        delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 250) {
            controlShooter(true);
        }
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        
        // Step 5: Shoot second ball (keeps shooter spinning)
        shootSingleBall(opMode);
        
        // Step 6: Set indexer to middle position (keep shooter spinning)
        setIndexerMiddle();
        delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 250) {
            controlShooter(true);
            robot.transferMotor.setPower(1);
        }
        
        // Step 7: Shoot third ball (keeps shooter spinning)
        shootSingleBall(opMode);
        
        // Final cleanup - NOW stop the shooter
        setBlocker(true);
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        controlShooter(false);
    }
    
    /**
     * ISSIS - Index, Shoot, Shoot, un-Index, Shoot sequence
     * Shooter starts spinning BEFORE indexing so it's ready by first shot
     * Shooter stays spinning throughout all shots
     * @param opMode The OpMode calling this (for sleep)
     * @throws InterruptedException If sleep is interrupted
     */
    public void ISSIS(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) throws InterruptedException {
        // Start shooter spinning BEFORE indexing (so it's up to speed by first shot)
        controlShooter(true);

        //moveTransferBallBackSmall(opMode);
        
        // Step 1: Set indexer to indexed position (while shooter ramps up)
        setIndexerIndexed();
        long delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 50) {
            controlShooter(true);
        }
        
        // Run transfer to position ball (keep shooter spinning)
        robot.transferMotor.setPower(1);
        robot.intakeMotor.setPower(1);
        delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 250) {
            controlShooter(true);
        }
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        
        // Step 2: Shoot first ball (keeps shooter spinning)
        shootSingleBall(opMode);
        
        // Step 3: Run transfer + blocker closed (keep shooter spinning)
        setBlocker(true);
        robot.transferMotor.setPower(1);
        robot.intakeMotor.setPower(1);
        delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 250) {
            controlShooter(true);
        }
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        
        // Step 4: Shoot second ball (keeps shooter spinning)
        shootSingleBall(opMode);
        
        // Step 5: Set indexer to middle position (keep shooter spinning)
        setIndexerMiddle();
        delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 100) {
            controlShooter(true);
        }
        
        // Step 6: Run transfer + blocker closed (keep shooter spinning)
        setBlocker(true);
        robot.transferMotor.setPower(HardwareConfigAuto.TRANSFER_POWER);
        robot.intakeMotor.setPower(HardwareConfigAuto.INTAKE_POWER);
        delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 350) {
            controlShooter(true);
        }
        robot.transferMotor.setPower(1);
        robot.intakeMotor.setPower(0);
        
        // Step 7: Shoot third ball (keeps shooter spinning)
        shootSingleBall(opMode);
        
        // Final cleanup - NOW stop the shooter
        setBlocker(true);
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        controlShooter(false);
    }
    
    /**
     * Move transfer motor backwards by 250 ticks
     * Used to prevent indexed ball from hitting another ball when moving back
     * Runs intake at full speed while doing this
     * @param opMode The OpMode calling this (for sleep)
     * @throws InterruptedException If sleep is interrupted
     */
    public void moveTransferBallBack(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) throws InterruptedException {
        // Get current position and calculate target (-250 ticks)
        int startPosition = robot.transferMotor.getCurrentPosition();
        int targetPosition = startPosition - 400;
        
        // Set up transfer motor for RUN_TO_POSITION
        robot.transferMotor.setTargetPosition(targetPosition);
        robot.transferMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.transferMotor.setPower(1.0);  // Full speed (direction from target position)
        
        // Run intake at full speed while transfer moves back
        robot.intakeMotor.setPower(1.0);
        
        // Wait for transfer motor to reach position
        while (opMode.opModeIsActive() && robot.transferMotor.isBusy()) {

        }
        
        // Stop motors
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        
        // Switch transfer motor back to RUN_WITHOUT_ENCODER
        robot.transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveTransferBallBackSmall(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) throws InterruptedException {
        // Get current position and calculate target (-250 ticks)
        int startPosition = robot.transferMotor.getCurrentPosition();
        int targetPosition = startPosition - 50;

        // Set up transfer motor for RUN_TO_POSITION
        robot.transferMotor.setTargetPosition(targetPosition);
        robot.transferMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.transferMotor.setPower(1.0);  // Full speed (direction from target position)

        // Run intake at full speed while transfer moves back
        robot.intakeMotor.setPower(1.0);

        // Wait for transfer motor to reach position
        while (opMode.opModeIsActive() && robot.transferMotor.isBusy()) {

        }

        // Stop motors
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);

        // Switch transfer motor back to RUN_WITHOUT_ENCODER
        robot.transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    
    /**
     * ISISS - Index, Shoot, (move back), un-Index, Shoot, Shoot sequence
     * Shooter starts spinning BEFORE indexing so it's ready by first shot
     * Shooter stays spinning throughout all shots
     * @param opMode The OpMode calling this (for sleep)
     * @throws InterruptedException If sleep is interrupted
     */
    public void ISISS(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) throws InterruptedException {
        // Start shooter spinning BEFORE indexing (so it's up to speed by first shot)
        controlShooter(true);
        //moveTransferBallBackSmall(opMode);
        
        // Step 1: Set indexer to indexed position (while shooter ramps up)
        setIndexerIndexed();
        long delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 50) {
            controlShooter(true);
        }
        
        // Run transfer briefly to position ball (keep shooter spinning)
        robot.transferMotor.setPower(1);
        robot.intakeMotor.setPower(1);
        delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 500) {
            controlShooter(true);
        }
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        
        // Step 2: Shoot first ball (keeps shooter spinning)
        shootSingleBall(opMode);
        
        // Step 3: Move transfer ball back (keep shooter spinning during move)
        moveTransferBallBack(opMode);
        
        // Step 4: Set indexer to middle position (keep shooter spinning)
        setIndexerMiddle();
        delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 200) {
            controlShooter(true);
        }
        
        // Run transfer briefly to position ball (keep shooter spinning)
        setBlocker(true);
        robot.transferMotor.setPower(1);
        robot.intakeMotor.setPower(1);
        delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 200) {
            controlShooter(true);
        }
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        
        // Step 5: Shoot second ball (keeps shooter spinning)
        shootSingleBall(opMode);
        
        // Run transfer briefly between shots (keep shooter spinning)
        setBlocker(true);
        robot.transferMotor.setPower(1);
        robot.intakeMotor.setPower(1);
        delayStart = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - delayStart < 350) {
            controlShooter(true);
        }
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        
        // Step 6: Shoot third ball (keeps shooter spinning)
        shootSingleBall(opMode);
        
        // Final cleanup - NOW stop the shooter
        setBlocker(true);
        robot.transferMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        controlShooter(false);
    }
}

