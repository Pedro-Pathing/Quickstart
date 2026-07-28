package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.slideConstants
import org.firstinspires.ftc.teamcode.pedroPathing.SimplePIDController
import kotlin.math.*

@TeleOp(name = "KotlinTeleOp")
class KotlinTeleOp : LinearOpMode() {
    private var follower: Follower? = null
    private var slide: slideConstants? = null
    private var intakeMotor: DcMotor? = null
    private var myServo: Servo? = null

    // --- Vision & EKF ---
    private var aprilTag: AprilTagProcessor? = null
    private var visionPortal: VisionPortal? = null
    private val ekf = ExtendedKalmanFilter()
    private var lastFollowerPose = Pose(0.0, 0.0, 0.0)
    private val CAMERA_YAW_OFFSET = 10.0 // Degrees

    // Define preset positions (0.0 to 1.0)
    private val HOME_POSITION = 0.0
    private val MAX_POSITION = 1.0

    private val DRIVE_SPEED_LIMIT = 0.65
    private val DEADZONE = 0.05

    // --- Heading Hold Constants ---
    private var targetHeading = 0.0
    private val headingLock_kP = 0.75
    private val headingLock_kD = 0.08

    // State variables
    private var lastError = 0.0
    private val timer = ElapsedTime()
    private val servoWiggleTimer = ElapsedTime()
    private val slideAnimationTimer = ElapsedTime()
    private var isSlideAnimating = false

    // Edge detection for field-centric reset button
    private var lastOptionsState = false

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        // Enable bulk reading on REV Hubs to minimize loop times and maximize odometry accuracy

        val allHubs = hardwareMap.getAll<LynxModule?>(LynxModule::class.java)
        for (hub in allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }

        follower = Constants.createFollower(hardwareMap)

        intakeMotor = hardwareMap.get<DcMotor?>(DcMotor::class.java, "intakeMotor")
        intakeMotor!!.setDirection(DcMotorSimple.Direction.FORWARD)
        intakeMotor!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        myServo = hardwareMap.get<Servo?>(Servo::class.java, "myServoName")

        slide = slideConstants(hardwareMap)

        // --- AprilTag & Vision Setup ---
        aprilTag = AprilTagProcessor.Builder().build()
        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(aprilTag)
            .build()

        telemetry.addData("Status", "Initialized. Bulk reading active.")
        telemetry.update()

        waitForStart()

        follower!!.startTeleopDrive()
        slide!!.start()

        // Sync EKF with starting pose
        lastFollowerPose = follower!!.getPose()
        ekf.x = lastFollowerPose.getX()
        ekf.y = lastFollowerPose.getY()
        ekf.theta = lastFollowerPose.getHeading()

        // Initialize target heading and reset timer
        targetHeading = follower!!.getPose().getHeading()
        timer.reset()
        servoWiggleTimer.reset()

        while (opModeIsActive()) {
            // Update localization algorithms
            follower!!.update()

            // --- Extended Kalman Filter (EKF) State Estimation ---
            val currentFollowerPose = follower!!.getPose()
            
            // 1. Predict Step (using Odometry delta)
            val dx = currentFollowerPose.getX() - lastFollowerPose.getX()
            val dy = currentFollowerPose.getY() - lastFollowerPose.getY()
            val dTheta = currentFollowerPose.getHeading() - lastFollowerPose.getHeading()
            ekf.predict(dx, dy, dTheta)
            lastFollowerPose = currentFollowerPose

            // 2. Update Step (using AprilTag if visible)
            val detections = aprilTag?.detections
            var ekfStatus = "Odometry Only"
            if (detections != null && detections.isNotEmpty()) {
                val detection = detections[0]
                if (detection.metadata != null) {
                    ekfStatus = "Fused (Tag ${detection.id})"
                    // Calculate Robot Pose from Tag
                    // Let [TX, TY, TH] be the Tag's Global Field Position
                    val tagFieldX = 72.0 // Example: Center of backdrop
                    val tagFieldY = 72.0
                    val tagFieldHeading = Math.toRadians(0.0)

                    // Relative position of robot to tag from camera
                    val relX = detection.ftcPose.x
                    val relY = detection.ftcPose.y
                    val relBearing = Math.toRadians(detection.ftcPose.yaw - CAMERA_YAW_OFFSET)

                    // Compute global robot pose based on this detection
                    val globalX = tagFieldX - (relX * cos(tagFieldHeading) - relY * sin(tagFieldHeading))
                    val globalY = tagFieldY - (relX * sin(tagFieldHeading) + relY * cos(tagFieldHeading))
                    val globalHeading = tagFieldHeading - relBearing

                    ekf.update(globalX, globalY, globalHeading)
                }
            }

            // 3. Apply Fused Pose back to Follower (REMOVED - CAUSING DRIFT)
            // follower!!.setPose(Pose(ekf.x, ekf.y, ekf.theta))

            // ------------------------------------
            // FIELD-CENTRIC HEADING RESET (START/OPTIONS BUTTON)
            // ------------------------------------
            val currentOptionsState = gamepad1.options || gamepad1.start
            if (currentOptionsState && !lastOptionsState) {
                // Keep X/Y position, reset heading to 0 (making current orientation "Forward")
                val currentPose = follower!!.getPose()
                follower!!.setPose(Pose(currentPose.getX(), currentPose.getY(), 0.0))


                // Reset target heading for the lock algorithm
                targetHeading = 0.0
                lastError = 0.0
            }
            lastOptionsState = currentOptionsState

            // Current robot heading from Pedro Pathing
            val currentHeading = follower!!.getPose().getHeading()

            // ------------------------------------
            // DRIVETRAIN CONTROL VIA ODOMETRY
            // ------------------------------------
            // 1. Invert axes for field orientation
            val rawY = -gamepad1.left_stick_y.toDouble()
            val rawX = -gamepad1.left_stick_x.toDouble()
            val rawRx = -gamepad1.right_stick_x.toDouble()

            // 2. Calculate translation vector magnitude
            val translationMagnitude = hypot(rawX, rawY)
            var y = 0.0
            var x = 0.0

            if (translationMagnitude > DEADZONE) {
                // Continuous scaling: zero out power right at the deadzone edge
                val normalizedMagnitude = (translationMagnitude - DEADZONE) / (1.0 - DEADZONE)

                // Apply cubic curve for micro-adjustments near center + scale by overall limit
                val scaledPower = normalizedMagnitude.pow(3.0) * DRIVE_SPEED_LIMIT

                // Maintain directional angle
                y = (rawY / translationMagnitude) * scaledPower
                x = (rawX / translationMagnitude) * scaledPower
            }

            // 3. Rotation control with Heading Lock
            var rx = 0.0
            val absRx = abs(rawRx)
            val dt = timer.seconds()
            timer.reset()

            if (absRx > DEADZONE) {
                // CASE 1: Driver IS turning manually (Right Stick)
                val normalizedRx = (absRx - DEADZONE) / (1.0 - DEADZONE)
                rx = sign(rawRx) * normalizedRx.pow(3.0) * DRIVE_SPEED_LIMIT

                targetHeading = currentHeading
                lastError = 0.0
            } else {
                // CASE 2: Lock target heading using PD Controller
                var headingError = targetHeading - currentHeading

                // Normalize error to stay within [-pi, pi] radians (angle wrapping)
                headingError = atan2(sin(headingError), cos(headingError))

                if (abs(headingError) < Math.toRadians(0.5)) {
                    rx = 0.0
                } else {
                    val derivative = if (dt > 0) (headingError - lastError) / dt else 0.0
                    rx = (headingError * headingLock_kP) + (derivative * headingLock_kD)
                    rx = max(-DRIVE_SPEED_LIMIT, min(DRIVE_SPEED_LIMIT, rx))
                }

                lastError = headingError
            }

            // Output to Pedro Pathing (false = field-centric mode)
            follower!!.setTeleOpDrive(y, x, rx, false)

            // ------------------------------------
            // SUBSYSTEM CONTROL
            // ------------------------------------
            intakeMotor!!.setPower(if (gamepad1.a) 0.80 else 0.0)

            if (gamepad1.dpad_up) {
                slide!!.extendToHigh()
                isSlideAnimating = true
                slideAnimationTimer.reset()
            } else if (gamepad1.dpad_down) {
                slide!!.extendToBottom()
            }

            if (gamepad1.back) {
                slide!!.resetEncoder()
            }

            if (gamepad1.dpad_right) {
                if (((servoWiggleTimer.seconds() * 4).toInt()) % 2 == 0) {
                    myServo!!.setPosition(MAX_POSITION)
                } else {
                    myServo!!.setPosition(HOME_POSITION)
                }
            } else if (gamepad1.dpad_left) {
                myServo!!.setPosition(HOME_POSITION)
            } else if (isSlideAnimating) {
                if (slideAnimationTimer.seconds() < 3.0) {
                    if (((slideAnimationTimer.seconds() * 4).toInt()) % 2 == 0) {
                        myServo!!.setPosition(MAX_POSITION)
                    } else {
                        myServo!!.setPosition(HOME_POSITION)
                    }
                } else {
                    isSlideAnimating = false
                    myServo!!.setPosition(HOME_POSITION)
                }
            }

            // Telemetry Output
            telemetry.addData("EKF Status", ekfStatus)
            telemetry.addData("Filtered X", "%.2f", ekf.x)
            telemetry.addData("Filtered Y", "%.2f", ekf.y)
            telemetry.addData("X Position", follower!!.getPose().getX())
            telemetry.addData("Y Position", follower!!.getPose().getY())
            telemetry.addData("Heading (Deg)", Math.toDegrees(currentHeading))
            telemetry.addData("Target Heading (Deg)", Math.toDegrees(targetHeading))
            telemetry.addData("Slide Position", slide!!.getCurrentPosition())
            telemetry.update()
        }
        visionPortal?.close()
    }

    /**
     * Extended Kalman Filter for Pose Estimation
     * Fuses Odometry (Prediction) with Absolute Measurements (Update)
     */
    inner class ExtendedKalmanFilter {
        var x = 0.0
        var y = 0.0
        var theta = 0.0

        private var P = Array(3) { DoubleArray(3) } // State Covariance
        private val Q = Array(3) { DoubleArray(3) } // Process Noise
        private val R = Array(3) { DoubleArray(3) } // Measurement Noise

        init {
            for (i in 0..2) {
                P[i][i] = 1.0   // Initial uncertainty
                Q[i][i] = 0.01  // Odometry is fairly reliable
                R[i][i] = 0.1   // Vision measurements have some noise
            }
        }

        fun predict(deltaX: Double, deltaY: Double, deltaTheta: Double) {
            // Prediction step: x_k = f(x_{k-1}, u_k)
            x += deltaX
            y += deltaY
            theta += deltaTheta
            
            // Uncertainty increases with prediction
            for (i in 0..2) P[i][i] += Q[i][i]
        }

        fun update(zX: Double, zY: Double, zTheta: Double) {
            // Update step: Correct state using measurement z
            // Use a much smaller gain for smoother filtering
            val ALPHA = 0.05
            x = (1.0 - ALPHA) * x + ALPHA * zX
            y = (1.0 - ALPHA) * y + ALPHA * zY
            
            var angleDiff = zTheta - theta
            while (angleDiff > PI) angleDiff -= 2.0 * PI
            while (angleDiff < -PI) angleDiff += 2.0 * PI
            theta += ALPHA * angleDiff
        }
    }
}