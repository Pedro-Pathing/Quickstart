package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Logitech Camera & Pollen Test")
public class CameraTest extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private PollenDetectorProcessor pollenProcessor;

    @Override
    public void runOpMode() {
        // Telemetry to both Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. Initialize AprilTag Processor (Uses default Logitech intrinsics built into FTC SDK)
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .build();

        // 2. Initialize Multi-Blob Pollen Processor
        pollenProcessor = new PollenDetectorProcessor();

        // 3. Build VisionPortal optimized for Logitech Webcams
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // Smooth hardware streaming for Logitech
                .addProcessors(aprilTagProcessor, pollenProcessor)
                .build();

        // 4. Send camera stream to FTC Dashboard (15 FPS prevents CPU lag)
        FtcDashboard.getInstance().startCameraStream(visionPortal, 15);

        telemetry.addData("Status", "Vision Portal Initialized (Logitech Mode)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- APRILTAG DETECTION ---
            List<AprilTagDetection> aprilTags = aprilTagProcessor.getDetections();
            telemetry.addData("AprilTags Found", aprilTags.size());

            for (AprilTagDetection tag : aprilTags) {
                if (tag.metadata != null) {
                    telemetry.addLine(String.format("Tag ID %d: X=%.1f in, Y=%.1f in, Yaw=%.1f°",
                            tag.id, tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.yaw));
                }
            }

            // --- MULTI-BLOB POLLEN DETECTION ---
            List<PollenDetectorProcessor.DetectedPollen> pollens = pollenProcessor.getDetectedPollenList();
            telemetry.addData("Valid Pollen Detected", pollens.size());

            int index = 1;
            for (PollenDetectorProcessor.DetectedPollen pollen : pollens) {
                telemetry.addLine(String.format("Pollen #%d: Center(%.0f, %.0f) | Circularity: %.2f",
                        index++, pollen.center.x, pollen.center.y, pollen.circularity));
            }

            // --- CAMERA PERFORMANCE ---
            telemetry.addData("Camera FPS", String.format("%.1f", visionPortal.getFps()));
            telemetry.update();
        }

        visionPortal.close();
    }

    // =========================================================================
    // CUSTOM POLLEN BLOB PROCESSOR (Multi-Blob + Shape Validation)
    // =========================================================================
    public static class PollenDetectorProcessor implements VisionProcessor {

        public static class DetectedPollen {
            public Point center;
            public Rect boundingBox;
            public double area;
            public double circularity;

            public DetectedPollen(Point center, Rect boundingBox, double area, double circularity) {
                this.center = center;
                this.boundingBox = boundingBox;
                this.area = area;
                this.circularity = circularity;
            }
        }

        private final Mat hsvMat = new Mat();
        private final Mat mask = new Mat();
        private final Mat morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new org.opencv.core.Size(5, 5));

        // Color & Threshold Tuning Variables
        public Scalar lowerYellow = new Scalar(15, 120, 100);
        public Scalar upperYellow = new Scalar(35, 255, 255);

        public double minArea = 300.0;       // Filters out small noise specks
        public double maxArea = 45000.0;     // Filters out large yellow backgrounds
        public double minCircularity = 0.65; // Rejects tape lines / non-spherical shapes
        public double minAspectRatio = 0.70;
        public double maxAspectRatio = 1.30;

        private final List<DetectedPollen> detectedPollenList = new ArrayList<>();
        private final Paint greenPaint = new Paint();

        public PollenDetectorProcessor() {
            greenPaint.setColor(Color.GREEN);
            greenPaint.setStyle(Paint.Style.STROKE);
            greenPaint.setStrokeWidth(3);
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // Smooth noise
            Imgproc.GaussianBlur(frame, frame, new org.opencv.core.Size(3, 3), 0);

            // Convert RGB to HSV
            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Threshold yellow
            Core.inRange(hsvMat, lowerYellow, upperYellow, mask);

            // Clean up mask
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, morphKernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, morphKernel);

            // Find Contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            synchronized (detectedPollenList) {
                detectedPollenList.clear();

                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area < minArea || area > maxArea) continue;

                    Rect rect = Imgproc.boundingRect(contour);
                    double aspectRatio = (double) rect.width / rect.height;
                    if (aspectRatio < minAspectRatio || aspectRatio > maxAspectRatio) continue;

                    MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                    double perimeter = Imgproc.arcLength(contour2f, true);
                    if (perimeter == 0) continue;

                    double circularity = (4 * Math.PI * area) / (perimeter * perimeter);
                    if (circularity < minCircularity) continue; // Rejects tape lines

                    Point center = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
                    detectedPollenList.add(new DetectedPollen(center, rect, area, circularity));
                }
            }

            hierarchy.release();
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onDrawFrameWidth, int onDrawFrameHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            synchronized (detectedPollenList) {
                for (DetectedPollen pollen : detectedPollenList) {
                    float left = (float) pollen.boundingBox.x * scaleBmpPxToCanvasPx;
                    float top = (float) pollen.boundingBox.y * scaleBmpPxToCanvasPx;
                    float right = left + (float) pollen.boundingBox.width * scaleBmpPxToCanvasPx;
                    float bottom = top + (float) pollen.boundingBox.height * scaleBmpPxToCanvasPx;

                    canvas.drawRect(left, top, right, bottom, greenPaint);
                }
            }
        }

        public List<DetectedPollen> getDetectedPollenList() {
            synchronized (detectedPollenList) {
                return new ArrayList<>(detectedPollenList);
            }
        }
    }
}