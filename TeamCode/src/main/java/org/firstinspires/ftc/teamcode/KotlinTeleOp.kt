package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.slideConstants
import java.util.concurrent.TimeUnit
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

    // --- VISION & FILTERING CONSTANTS ---
    private val DESIRED_TAG_ID = 586
    private val CAMERA_FORWARD_OFFSET = 0.0
    private val CAMERA_LEFT_OFFSET = -2.0
    private val MAX_TRANSLATION_JUMP = 5.0
    private val MAX_YAW_JUMP_DEG = 10.0

    // --- DASHBOARD TAG & FIELD BOUNDARIES (3x5 Tile Layout) ---
    private val DASHBOARD_TAG_X = 0.0
    private val DASHBOARD_TAG_Y = 40.0
    private val DASHBOARD_TAG_HEADING = Math.toRadians(-90.0)

    private val BOUNDARY_X_MIN = -36.0
    private val BOUNDARY_X_MAX = 36.0
    private val BOUNDARY_Y_MIN = -48.0
    private val BOUNDARY_Y_MAX = 72.0

    // Preset positions (0.0 to 1.0)
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

    private var lastOptionsState = false
    private var lastRawX = 0.0
    private var lastRawY = 0.0
    private var lastRawYaw = 0.0

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val allHubs = hardwareMap.getAll<LynxModule?>(LynxModule::class.java)
        for (hub in allHubs) {
            hub?.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
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
            .setCameraResolution(android.util.Size(640, 480))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessor(aprilTag)
            .build()

        FtcDashboard.getInstance().startCameraStream(visionPortal, 15.0)

        while (!isStopRequested() && visionPortal?.cameraState != VisionPortal.CameraState.STREAMING) {
            sleep(20)
        }

        if (isStopRequested()) return

        val exposureControl = visionPortal?.getCameraControl(ExposureControl::class.java)
        if (exposureControl != null && exposureControl.isModeSupported(ExposureControl.Mode.Manual)) {
            exposureControl.setMode(ExposureControl.Mode.Manual)
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS)
        }

        val gainControl = visionPortal?.getCameraControl(GainControl::class.java)
        gainControl?.setGain(200)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addData("Status", "Initialized & Stream Active")
        telemetry.update()

        waitForStart()

        follower!!.startTeleopDrive()
        slide!!.start()

        lastFollowerPose = follower!!.getPose()
        ekf.x = lastFollowerPose.getX()
        ekf.y = lastFollowerPose.getY()
        ekf.theta = lastFollowerPose.getHeading()

        targetHeading = follower!!.getPose().getHeading()
        timer.reset()
        servoWiggleTimer.reset()

        while (opModeIsActive()) {
            follower!!.update()

            // --------------------------------------------------------
            // 1. EXTENDED KALMAN FILTER (EKF) PREDICT & UPDATE
            // --------------------------------------------------------
            val currentFollowerPose = follower!!.getPose()

            // Predict Step (Odometry delta)
            val dx = currentFollowerPose.getX() - lastFollowerPose.getX()
            val dy = currentFollowerPose.getY() - lastFollowerPose.getY()
            val dTheta = currentFollowerPose.getHeading() - lastFollowerPose.getHeading()
            ekf.predict(dx, dy, dTheta)
            lastFollowerPose = currentFollowerPose

            var ekfStatus = "Odometry Only"

            // Update Step (Active when left trigger is held and tag is visible)
            if (gamepad1.left_trigger > 0.1) {
                val detections = aprilTag?.detections
                var targetTag: org.firstinspires.ftc.vision.apriltag.AprilTagDetection? = null

                if (detections != null) {
                    for (detection in detections) {
                        if (detection.id == DESIRED_TAG_ID && detection.metadata != null) {
                            targetTag = detection
                            break
                        }
                    }
                }

                if (targetTag != null) {
                    val rawPedroX = targetTag.ftcPose.y
                    val rawPedroY = -targetTag.ftcPose.x
                    val rawYaw = targetTag.ftcPose.yaw

                    val xJump = abs(rawPedroX - lastRawX)
                    val yJump = abs(rawPedroY - lastRawY)
                    val yawJump = abs(rawYaw - lastRawYaw)

                    if (xJump <= MAX_TRANSLATION_JUMP && yJump <= MAX_TRANSLATION_JUMP && yawJump <= MAX_YAW_JUMP_DEG) {
                        ekfStatus = "Fused (Tag ${targetTag.id})"

                        val tagLocalX = rawPedroX + CAMERA_FORWARD_OFFSET
                        val tagLocalY = rawPedroY + CAMERA_LEFT_OFFSET

                        val trueRobotHeading = DASHBOARD_TAG_HEADING - Math.toRadians(rawYaw)
                        val trueRobotX = DASHBOARD_TAG_X - (tagLocalX * cos(trueRobotHeading)) + (tagLocalY * sin(trueRobotHeading))
                        val trueRobotY = DASHBOARD_TAG_Y - (tagLocalX * sin(trueRobotHeading)) - (tagLocalY * cos(trueRobotHeading))

                        // Feed update into EKF
                        ekf.update(trueRobotX, trueRobotY, trueRobotHeading)

                        // Safely sync back to Follower so localization corrects itself
                        follower!!.setPose(Pose(ekf.x, ekf.y, ekf.theta))
                    }

                    lastRawX = rawPedroX
                    lastRawY = rawPedroY
                    lastRawYaw = rawYaw
                }
            }

            // --------------------------------------------------------
            // 2. FIELD-CENTRIC HEADING RESET (START/OPTIONS BUTTON)
            // --------------------------------------------------------
            val currentOptionsState = gamepad1.options || gamepad1.start
            if (currentOptionsState && !lastOptionsState) {
                val currentPose = follower!!.getPose()
                follower!!.setPose(Pose(currentPose.getX(), currentPose.getY(), 0.0))
                ekf.theta = 0.0
                targetHeading = 0.0
                lastError = 0.0
            }
            lastOptionsState = currentOptionsState

            val currentHeading = follower!!.getPose().getHeading()

            // --------------------------------------------------------
            // 3. DRIVETRAIN CONTROL VIA ODOMETRY
            // --------------------------------------------------------
            val rawY = -gamepad1.left_stick_y.toDouble()
            val rawX = -gamepad1.left_stick_x.toDouble()
            val rawRx = -gamepad1.right_stick_x.toDouble()

            val translationMagnitude = hypot(rawX, rawY)
            var y = 0.0
            var x = 0.0

            if (translationMagnitude > DEADZONE) {
                val normalizedMagnitude = (translationMagnitude - DEADZONE) / (1.0 - DEADZONE)
                val scaledPower = normalizedMagnitude.pow(3.0) * DRIVE_SPEED_LIMIT
                y = (rawY / translationMagnitude) * scaledPower
                x = (rawX / translationMagnitude) * scaledPower
            }

            var rx = 0.0
            val absRx = abs(rawRx)
            val dt = timer.seconds()
            timer.reset()

            if (absRx > DEADZONE) {
                val normalizedRx = (absRx - DEADZONE) / (1.0 - DEADZONE)
                rx = sign(rawRx) * normalizedRx.pow(3.0) * DRIVE_SPEED_LIMIT
                targetHeading = currentHeading
                lastError = 0.0
            } else {
                var headingError = targetHeading - currentHeading
                headingError = atan2(sin(headingError), cos(headingError))

                if (abs(headingError) < Math.toRadians(1.5)) {
                    rx = 0.0
                } else {
                    val derivative = if (dt > 0) (headingError - lastError) / dt else 0.0
                    rx = (headingError * headingLock_kP) + (derivative * headingLock_kD)
                    rx = max(-DRIVE_SPEED_LIMIT, min(DRIVE_SPEED_LIMIT, rx))
                }
                lastError = headingError
            }

            follower!!.setTeleOpDrive(y, x, rx, false)

            // --------------------------------------------------------
            // 4. SUBSYSTEM CONTROL
            // --------------------------------------------------------
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
                if (slideAnimationTimer.seconds() < 6.0) {
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

            // --------------------------------------------------------
            // 5. FTC DASHBOARD DRAWING & TELEMETRY
            // --------------------------------------------------------
            val packet = TelemetryPacket()
            val fieldOverlay: Canvas = packet.fieldOverlay()
            val currentPose = follower!!.getPose()

            // Draw 3x5 Tile Boundary (Thick Black Lines)
            fieldOverlay.setStrokeWidth(3)
            fieldOverlay.setStroke("#000000")
            fieldOverlay.strokeLine(BOUNDARY_X_MIN, BOUNDARY_Y_MIN, BOUNDARY_X_MAX, BOUNDARY_Y_MIN)
            fieldOverlay.strokeLine(BOUNDARY_X_MAX, BOUNDARY_Y_MIN, BOUNDARY_X_MAX, BOUNDARY_Y_MAX)
            fieldOverlay.strokeLine(BOUNDARY_X_MAX, BOUNDARY_Y_MAX, BOUNDARY_X_MIN, BOUNDARY_Y_MAX)
            fieldOverlay.strokeLine(BOUNDARY_X_MIN, BOUNDARY_Y_MAX, BOUNDARY_X_MIN, BOUNDARY_Y_MIN)

            // Draw AprilTag Target (Green Square)
            fieldOverlay.setStrokeWidth(1)
            fieldOverlay.setStroke("#00FF00")
            fieldOverlay.setFill("#00FF00")
            fieldOverlay.fillRect(DASHBOARD_TAG_X - 2.0, DASHBOARD_TAG_Y - 2.0, 4.0, 4.0)

            // Draw Robot (Blue Circle & Heading Vector)
            fieldOverlay.setStroke("#0000FF")
            fieldOverlay.strokeCircle(currentPose.getX(), currentPose.getY(), 9.0)

            val headingLineX = currentPose.getX() + 9.0 * cos(currentHeading)
            val headingLineY = currentPose.getY() + 9.0 * sin(currentHeading)
            fieldOverlay.strokeLine(currentPose.getX(), currentPose.getY(), headingLineX, headingLineY)

            FtcDashboard.getInstance().sendTelemetryPacket(packet)

            telemetry.addData("EKF Status", ekfStatus)
            telemetry.addData("Filtered X", "%.2f", ekf.x)
            telemetry.addData("Filtered Y", "%.2f", ekf.y)
            telemetry.addData("Heading (Deg)", Math.toDegrees(currentHeading))
            telemetry.addData("Slide Position", slide!!.getCurrentPosition())
            telemetry.update()
        }
        visionPortal?.close()
    }

    /**
     * Extended Kalman Filter for Pose Estimation
     */
    inner class ExtendedKalmanFilter {
        var x = 0.0
        var y = 0.0
        var theta = 0.0

        fun predict(deltaX: Double, deltaY: Double, deltaTheta: Double) {
            x += deltaX
            y += deltaY
            theta += deltaTheta
        }

        fun update(zX: Double, zY: Double, zTheta: Double) {
            val alpha = 0.15 // Blending weight for camera measurements
            x = (1.0 - alpha) * x + alpha * zX
            y = (1.0 - alpha) * y + alpha * zY

            var angleDiff = zTheta - theta
            while (angleDiff > PI) angleDiff -= 2.0 * PI
            while (angleDiff < -PI) angleDiff += 2.0 * PI
            theta += alpha * angleDiff
        }
    }
}