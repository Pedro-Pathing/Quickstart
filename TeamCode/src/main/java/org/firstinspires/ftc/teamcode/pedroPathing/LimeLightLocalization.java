package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Configurable
@TeleOp
public class LimeLightLocalization extends OpMode {
    DcMotorEx motorFrontLeft;
    DcMotorEx motorBackLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackRight;
    Limelight3A limelight;

    private final Pose startPose = new Pose(39, 33, Math.toRadians(180));
    private TelemetryManager telemetryM;
    private Follower follower;
    private boolean isSeeded = false;
    GoBildaPinpointDriver pinpoint;

    public static int wait = 5;

    public void drawCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    @Override
    // runs on init press
    public void init() {
        // define and init robot
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        // 2) Keep Pinpoint aligned to the same field pose
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                startPose.getX(),
                startPose.getY(),
                AngleUnit.DEGREES,
                Math.toDegrees(startPose.getHeading())));

    }

    @Override
    // runs on start press
    public void start() {
        // run everything to start positions
        limelight.start();
    }

    ElapsedTime lastRecolaized = new ElapsedTime();

    @SuppressLint("DefaultLocale")
    @Override
    // loops after start
    // press
    public void loop() {
        pinpoint.update();

        if (!isSeeded) {
            follower.setPose(startPose);
            follower.update();
            isSeeded = true;
            return;
        }

        if (gamepad1.a) {
            follower.setPose(startPose);
            follower.update();
        }
        follower.update();

        Pose currentPose = follower.getPose();
        telemetry.addData("Pedro Pose", String.format("x=%.2f in, y=%.2f in, h=%.1f deg", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));

        Drawing.drawPoseHistory(follower.getPoseHistory());
        drawCurrent();

//       if (gamepad2.start || gamepad1.start) return;
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (!gamepad1.right_bumper){
            motorFrontLeft.setPower(y + x + rx);
            motorBackLeft.setPower(y - x + rx);
            motorFrontRight.setPower(y - x - rx);
            motorBackRight.setPower(y + x - rx);
        }
        else {
            motorFrontLeft.setPower(0.3 * (y + x + rx));
            motorBackLeft.setPower(0.3 * (y - x + rx));
            motorFrontRight.setPower(0.3 * (y - x - rx));
            motorBackRight.setPower(0.3 * (y + x - rx));
        }

        double ppYaw = pinpoint.getHeading(AngleUnit.DEGREES);

        telemetry.addData("ppYaw", ppYaw);
        limelight.updateRobotOrientation(ppYaw);
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            telemetry.addData("Botpose_MT1 exists", botpose_mt2 != null);

            if (botpose_mt2 != null) {
                double a = botpose_mt2.getPosition().x;
                double b = botpose_mt2.getPosition().y;
                double z = botpose_mt2.getPosition().z;
                // Note: Pose3D might not have getHeading() method
                telemetry.addData("MT2 Location:", "(" + a + ", " + b + ", " + z + ")");
                // telemetry.addData("MT2 Heading:", heading);

                // Note: WPIBLUE and WPIRED methods don't exist in this API
                // Re-localize robot pose based on AprilTag field pose when detected
                try {
                    double headingDeg = Math.toDegrees(botpose_mt2.getOrientation().getPitch());
                    double xInches = a * 39.3701;
                    double yInches = b * 39.3701;

                    // Keep Pedro follower in sync
//                    follower.setPose(new Pose(xInches, yInches, follower.getHeading()));

                    boolean clearView = false;

                    if (lastRecolaized.seconds() >= wait) {
                        if (clearView) {
                            telemetry.addData("Re-localized", String.format("x=%.2f in, y=%.2f in, h=%.1f deg", xInches, yInches, headingDeg));
                            lastRecolaized.reset();
                        } else {
                            telemetry.addData("April Tag not in clear view", String.format("wait=%.2f, x=%.2f in, y=%.2f in, h=%.1f deg", lastRecolaized.seconds(), xInches, yInches, headingDeg));
                        }
                    } else {
                        telemetry.addData("Waiting to Re-localized, ", String.format("wait=%.2f, x=%.2f in, y=%.2f in, h=%.1f deg", lastRecolaized.seconds(), xInches, yInches, headingDeg));
                    }

                } catch (Exception ignored) { }
            } else {
                telemetry.addData("Botpose_MT2", "NULL - Check pipeline configuration");
            }
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.update();
    }
}