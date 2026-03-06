package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.common.CustomPIDController;
import org.firstinspires.ftc.teamcode.common.TimestampedAngleBacklog;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp
@Configurable
public class odometryTurretTest extends LinearOpMode {
    private final Pose goalPos = new Pose(144, 144, Math.toRadians(45));
    public static double newX = 0;
    public static double newY = 0;

    public static double newHeading;
    public static double kp = 4;
    public static double ki = 0.007;
    public static double kd = 0.0015;
    public static double kf = 0.001;
    public static double ks = 0.075;
    public static long ms_comp = 40;

    private final TimestampedAngleBacklog backlog = new TimestampedAngleBacklog();
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    LoopTimer timer = new LoopTimer();


    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8,5,Math.toRadians(90)));


        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "motor_tm");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limeLight");
        CustomPIDController controller = new CustomPIDController();

        waitForStart();
        limelight3A.pipelineSwitch(0);
        limelight3A.start();


        while (!isStopRequested()) {
            timer.start();
            sleep(50);
            controller.setkD(kd);

            controller.setkP(kp);
            controller.setkI(ki);
            controller.setkF(kf);
            controller.setkS(ks);

            double targetAngle;
            double globalAngle = 0;
            boolean useVision;
            double k = -360. / (4096. * (70. / 20.));
            double turretAngle = k * turret.getCurrentPosition();
            double turretVelocity = k * turret.getVelocity();

            backlog.addAngle(turretAngle);

            LLResult result = limelight3A.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double prevAngle = backlog.getAngle(result.getControlHubTimeStampNanos() - ms_comp * 1000000);
                panelsTelemetry.addData("prevAngle", prevAngle);
                targetAngle = prevAngle - tx;
                useVision = true;
            } else {

                Pose robotPos = follower.getPose();
                globalAngle = Math.atan2(goalPos.getY() - robotPos.getY(), goalPos.getX() - robotPos.getX());

                double targetAngleRad = globalAngle - robotPos.getHeading() - Math.PI;
                targetAngleRad = Math.atan2(Math.sin(targetAngleRad), Math.cos(targetAngleRad));
                targetAngle = Math.toDegrees(targetAngleRad);
                useVision = false;
            }
            boolean atAngle = Math.abs(turretAngle - targetAngle) <= 1;

            if (Math.abs(targetAngle) < 165 && !atAngle) {
                controller.setTarget(targetAngle);
                double power = Math.clamp(controller.update(turretAngle, turretVelocity), -0.65, 0.65);
                turret.setPower(power);
                panelsTelemetry.addData("power", power);
            } else {
                turret.setPower(0);
                controller.skip();
            }
            limelight3A.updateRobotOrientation(Math.toDegrees(follower.getHeading()));


            follower.update();
            timer.end();

            panelsTelemetry.addData("atAngle", atAngle);
            panelsTelemetry.addData("targetAngle", targetAngle);
            panelsTelemetry.addData("globalAngle", Math.toDegrees(globalAngle));
            panelsTelemetry.addData("follower", follower.getPose());

            panelsTelemetry.addData("turretAngle", turretAngle);
            panelsTelemetry.addData("turretPosition", turret.getCurrentPosition());
            panelsTelemetry.addData("useVision", useVision);

            panelsTelemetry.addData("looptime", timer.getMs());
            panelsTelemetry.update();

        }
    }
}








