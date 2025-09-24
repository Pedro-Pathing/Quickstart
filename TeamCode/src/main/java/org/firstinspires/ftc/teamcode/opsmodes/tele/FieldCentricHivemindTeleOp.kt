import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.opsmodes.pedroPathing.Constants


@TeleOp(name = "Hivemind - Field Centric")
class FieldCentricHivemindTeleOp : LinearOpMode() {

    var isShooting: Boolean = false
    lateinit var follower: Follower
    lateinit var shootTimer: ElapsedTime
    lateinit var feeder: Servo
    lateinit var shooter: DcMotor

    override fun runOpMode() {
        feeder = hardwareMap.get(Servo::class.java, "feeder");
        shooter = hardwareMap.get(DcMotor::class.java, "shooter");
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(Pose())
        follower.update()
        shootTimer = ElapsedTime()
        feeder.position = 1.0
        isShooting = false

        waitForStart()
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            follower.update()
            follower.setTeleOpDrive(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
                false
            )
            doShooting()
            idle()
        }
    }

    private fun doShooting() {
        var shooterPower = gamepad2.right_trigger.toDouble()
        if (shooterPower > 0.75) {
            shooterPower = 0.75
        }
        shooter.power = shooterPower
        if (!isShooting && gamepad2.a && shooterPower > 0.5) {
            shootTimer.reset()
            isShooting = true
        }
        if (isShooting) {
            if (shootTimer.time() < 0.2) {
                feeder.position = 0.5
            }
            if (shootTimer.time() >= 0.2) {
                feeder.position = 1.0
                isShooting = false
            }
        }
    }

}