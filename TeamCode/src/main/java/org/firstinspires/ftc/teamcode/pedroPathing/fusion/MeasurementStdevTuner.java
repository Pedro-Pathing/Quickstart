package org.firstinspires.ftc.teamcode.pedroPathing.fusion;

import android.util.Size;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Measurement STDEV Tuner", group = "Tuners")
public class MeasurementStdevTuner extends OpMode {
    private static final Position cameraPosition = new Position(
            DistanceUnit.INCH,
            2.204,
            5.96,
            8.82,
            0
    );
    private static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0,
            -80,
            180,
            0
    );
    public static boolean on = false;
    public static double actualX = 0;
    public static double actualY = 0;
    public static double actualHeading = 0;
    private final Signal varianceX = new Signal();
    private final Signal varianceY = new Signal();
    private final Signal varianceHeading = new Signal();
    private AprilTagProcessor processor;
    private Follower follower;

    @Override
    public void init() {
        processor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(544.2876017217492, 543.8059217350639, 332.20336755183894, 248.65289514406953)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .build();
    }

    @Override
    public void loop() {
        follower.manual(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (on) {
            List<AprilTagDetection> detections = processor.getDetections();
            telemetry.addData("AprilTag/Detections", detections.size());

            for (AprilTagDetection detection : detections) {
                if (detection.metadata == null || detection.metadata.name.contains("Obelisk")) continue;

                telemetry.addData(detection.metadata.name, detection.robotPose);

                Pose pose = new Pose(
                        detection.robotPose.getPosition().y + 72,
                        -detection.robotPose.getPosition().x + 72,
                        detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
                );

                varianceX.update(actualX - pose.x());
                varianceY.update(actualY - pose.y());
                varianceHeading.update(actualHeading - pose.heading());
            }
        }

        telemetry.addData("variance x", varianceX.variance());
        telemetry.addData("variance y", varianceY.variance());
        telemetry.addData("variance heading", varianceHeading.variance());

        telemetry.addData("stdev x", varianceX.stdDev());
        telemetry.addData("stdev y", varianceY.stdDev());
        telemetry.addData("stdev heading", varianceHeading.stdDev());

        telemetry.addData("mean x", varianceX.mean());
        telemetry.addData("mean y", varianceY.mean());
        telemetry.addData("mean heading", varianceHeading.mean());
    }

    public static final class Signal {
        private int n = 0;
        private double mean = 0;
        private double m2 = 0;

        public void update(double x) {
            n++;
            double delta = x - mean;
            mean += delta / n;
            double delta2 = x - mean;
            m2 += delta * delta2;
        }

        public double variance() {
            return m2 / (n - 1);
        }

        public double mean() {
            return mean;
        }

        public int n() {
            return n;
        }

        public double stdDev() {
            return Math.sqrt(variance());
        }
    }
}