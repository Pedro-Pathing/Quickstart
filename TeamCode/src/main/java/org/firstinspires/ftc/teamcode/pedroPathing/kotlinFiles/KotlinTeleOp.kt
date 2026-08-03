package org.firstinspires.ftc.teamcode.pedroPathing.kotlinFiles

import android.util.Size
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.constants.TeleOpConstants.*
import org.firstinspires.ftc.teamcode.pedroPathing.constants.slideConstants
import org.firstinspires.ftc.teamcode.pedroPathing.subSystems.AprilTagAligner
import org.firstinspires.ftc.teamcode.pedroPathing.subSystems.HeadingLockHandler
import org.firstinspires.ftc.teamcode.pedroPathing.subSystems.MechanismStateMachine
import org.firstinspires.ftc.teamcode.pedroPathing.subSystems.ServoAnimationHandler
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.lang.Math.toDegrees
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sin

/** Structured competition Kotlin TeleOp OpMode matching TeleopOffseason architecture with low-pass filters & cubic scaling. */
@TeleOp(name = "KotlinTeleOp")
class KotlinTeleOp : LinearOpMode() {

    private lateinit var follower: Follower
    private lateinit var slide: slideConstants
    private lateinit var intakeMotor: DcMotor

    private lateinit var aprilTag: AprilTagProcessor
    private lateinit var visionPortal: VisionPortal

    private lateinit var aligner: AprilTagAligner
    private lateinit var headingLock: HeadingLockHandler
    private lateinit var mechanisms: MechanismStateMachine

    private var lastOptionsState = false

    // --- Stick Drift Filtering Variables ---
    private var filterX = 0.0
    private var filterY = 0.0
    private var filterRx = 0.0
    private val STICK_ALPHA = 0.2 // Low-pass filter weight

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        // 1. Hardware & Pedro Pathing Initialization
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)

        follower = Constants.createFollower(hardwareMap)
        slide = slideConstants(hardwareMap)
        intakeMotor = hardwareMap.get(DcMotor::class.java, "intakeMotor")
        intakeMotor.direction = DcMotorSimple.Direction.FORWARD
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // 2. Vision Initialization
        aprilTag = AprilTagProcessor.Builder().build()
        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .setCameraResolution(Size(640, 480))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessor(aprilTag)
            .build()

        FtcDashboard.getInstance().startCameraStream(visionPortal, 15.0)
        setupCameraControls()

        // 3. Modular Subsystem Components
        aligner = AprilTagAligner(aprilTag, follower)
        headingLock = HeadingLockHandler(follower)
        mechanisms = MechanismStateMachine(slide, ServoAnimationHandler(hardwareMap.get(Servo::class.java, "myServoName")))

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addData("Status", "Initialized & Stream Active")
        telemetry.update()

        waitForStart()

        follower.startTeleopDrive()
        slide.start()
        headingLock.resetHeading(follower.pose.heading)

        while (opModeIsActive()) {
            follower.update()

            // --- Reset Heading (Field Centric Sync) ---
            val currentOptionsState = gamepad1.options || gamepad1.start
            if (currentOptionsState && !lastOptionsState) {
                follower.setPose(Pose(follower.pose.x, follower.pose.y, 0.0))
                headingLock.resetHeading(0.0)
            }
            lastOptionsState = currentOptionsState

            // --- Driving & Alignment Selection ---
            if (gamepad1.left_trigger > 0.1) {
                if (!aligner.isAligning) aligner.startAlignment()
                aligner.update()
                headingLock.resetHeading(follower.pose.heading)
            } else {
                aligner.stopAlignment()
                handleManualDrive()
            }

            // --- Mechanism & Intake Logic ---
            intakeMotor.power = if (gamepad1.a) 0.80 else 0.0
            mechanisms.update(gamepad1)

            // --- Telemetry & Dashboard Map Rendering ---
            updateDashboardAndTelemetry()
        }
        visionPortal.close()
    }

    /** Translates joystick inputs into robot movement using Low-Pass Filtering, Cubic Scaling, and Settled Heading Lock. */
    private fun handleManualDrive() {
        // 1. Apply Low-Pass Filter to raw joystick values to eradicate micro-jitter and stick drift
        filterY = STICK_ALPHA * (-gamepad1.left_stick_y) + (1.0 - STICK_ALPHA) * filterY
        filterX = STICK_ALPHA * (-gamepad1.left_stick_x) + (1.0 - STICK_ALPHA) * filterX
        filterRx = STICK_ALPHA * (-gamepad1.right_stick_x) + (1.0 - STICK_ALPHA) * filterRx

        val magnitude = hypot(filterX, filterY)

        var y = 0.0
        var x = 0.0
        if (magnitude > DEADZONE) {
            // 2. Apply Cubic Scaling for precise micro-control near center stick values
            val scalar = ((magnitude - DEADZONE) / (1.0 - DEADZONE)).pow(3.0) * DRIVE_SPEED_LIMIT
            y = (filterY / magnitude) * scalar
            x = (filterX / magnitude) * scalar
        }

        val rx = headingLock.calculateRotationPower(filterRx, magnitude)
        follower.setTeleOpDrive(y, x, rx, false)
    }

    /** Visualizes the robot and target state on the Dashboard field map with diagnostic data. */
    private fun updateDashboardAndTelemetry() {
        val packet = TelemetryPacket()
        val canvas: Canvas = packet.fieldOverlay()
        val p = follower.pose

        // 1. Render Map Graphics
        canvas.setStrokeWidth(1)
        canvas.setStroke("#00FF00").setFill("#00FF00").fillRect(DASHBOARD_TAG_X - 2.0, DASHBOARD_TAG_Y - 2.0, 4.0, 4.0) // Target
        canvas.setStroke("#0000FF").strokeCircle(p.x, p.y, 9.0) // Robot
        canvas.strokeLine(p.x, p.y, p.x + 9.0 * cos(p.heading), p.y + 9.0 * sin(p.heading)) // Heading

        if (aligner.isAligning && aligner.lastTargetPose != null) {
            val target = aligner.lastTargetPose!!
            canvas.setStroke("#FF0000").strokeLine(p.x, p.y, target.x, target.y) // Path
            canvas.strokeCircle(target.x, target.y, 2.0)
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        // 2. Textual Diagnostic Data
        telemetry.addData("Pose", "X: %.1f, Y: %.1f, H: %.1f deg", p.x, p.y, toDegrees(p.heading))
        telemetry.addData("Target Heading", "%.1f deg", toDegrees(headingLock.targetHeading))
        telemetry.addData("Slide", slide.currentPosition)
        telemetry.addData("Mode", if (aligner.isAligning) "AUTO-ALIGN" else "MANUAL")

        val v = follower.velocity
        val stopTime = if (v.magnitude > 0) v.magnitude / 50.0 else 0.0
        telemetry.addData("Est. Stop Dist", "%.1f in (%.2f s)", 0.5 * v.magnitude * stopTime, stopTime)
        telemetry.update()
    }

    /** Configures camera controls for optimized frame processing. */
    private fun setupCameraControls() {
        while (!isStopRequested && visionPortal.cameraState != VisionPortal.CameraState.STREAMING) {
            sleep(20)
        }
        if (isStopRequested) return

        visionPortal.getCameraControl(ExposureControl::class.java)?.let { ec ->
            if (ec.isModeSupported(ExposureControl.Mode.Manual)) {
                ec.mode = ExposureControl.Mode.Manual
                ec.setExposure(15, TimeUnit.MILLISECONDS)
            }
        }
        visionPortal.getCameraControl(GainControl::class.java)?.let { gc ->
            gc.gain = 200
        }
    }
}