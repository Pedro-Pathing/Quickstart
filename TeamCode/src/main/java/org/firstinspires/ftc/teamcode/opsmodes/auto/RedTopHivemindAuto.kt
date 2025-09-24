import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.opsmodes.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.opsmodes.shared.PathState


@Autonomous(name = "Hivemind - Red Top")
class RedTopHivemindAuto : LinearOpMode() {

    var pathState = PathState.DRIVE_TO_SHOOT
    lateinit var follower: Follower
    lateinit var bottomShootPath: PathChain
    lateinit var endPath: PathChain
    lateinit var shootTimer: ElapsedTime
    lateinit var feeder: Servo
    lateinit var shooter: DcMotor

    override fun runOpMode() {
        feeder = hardwareMap.get(Servo::class.java, "feeder");
        shooter = hardwareMap.get(DcMotor::class.java, "shooter");
        shootTimer = ElapsedTime()
        follower = Constants.createFollower(hardwareMap)
        val telemetryM = PanelsTelemetry.telemetry
        feeder.position = 1.0

        buildPath()
        waitForStart()
        shooter.power = 0.75

        while (opModeIsActive()) {
            updatePath()
            follower.update()
            telemetryM.update()
            telemetryM.debug("position", follower.getPose())
            telemetryM.debug("velocity", follower.getVelocity())

            idle()
        }
    }

    private fun buildPath() {
        val startingPose = Pose(0.0, 0.0)
        val shootingPose = Pose(48.0, 0.0, Math.toRadians(180.0))
        val endPose = Pose(24.0, -24.0, Math.toRadians(135.0))

        bottomShootPath = follower.pathBuilder()
            .setGlobalDeceleration()
            .addPath(BezierLine(startingPose, shootingPose))
            .setLinearHeadingInterpolation(startingPose.heading, shootingPose.heading)
            .build()

        endPath = follower.pathBuilder()
            .setGlobalDeceleration()
            .addPath(BezierLine(shootingPose, endPose))
            .setLinearHeadingInterpolation(shootingPose.heading, endPose.heading)
            .build()
    }

    private fun updatePath() {
        if (follower.isBusy) {
            return
        }

        when (pathState) {
            PathState.DRIVE_TO_SHOOT -> {
                follower.followPath(bottomShootPath)
                pathState = PathState.SHOOT_START
            }

            PathState.SHOOT_START -> {
                shootTimer.reset()
                pathState = PathState.SHOOT
            }

            PathState.SHOOT -> {
                if (shootTimer.time() >= 1 && shootTimer.time() < 1.2) {
                    feeder.position = 0.5
                }
                if (shootTimer.time() >= 1.2 && shootTimer.time() < 1.4) {
                    feeder.position = 1.0
                }

                if (shootTimer.time() >= 4 && shootTimer.time() < 4.2) {
                    feeder.position = 0.5
                }
                if (shootTimer.time() >= 4.2 && shootTimer.time() < 4.4) {
                    feeder.position = 1.0
                }

                if (shootTimer.time() >= 7 && shootTimer.time() < 7.2) {
                    feeder.position = 0.5
                }
                if (shootTimer.time() >= 7.2 && shootTimer.time() < 7.4) {
                    feeder.position = 1.0
                }
                if (shootTimer.time() > 8) {
                    pathState = PathState.DRIVE_TO_END
                }
            }

            PathState.DRIVE_TO_END -> {
                follower.followPath(endPath)
                pathState = PathState.STOP
                shooter.power = 0.0
            }

            else -> {
                return
            }
        }

    }
}