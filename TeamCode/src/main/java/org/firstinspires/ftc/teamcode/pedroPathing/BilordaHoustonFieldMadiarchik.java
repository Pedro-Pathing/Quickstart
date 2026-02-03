package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Madiarchik", group = "TeleOp")
public class    BilordaHoustonFieldMadiarchik extends OpMode {

    private static Follower follower;
    private static Pose autoEndPose;

    private long lastTime = 0;
    public static int vel = 750;


    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    private static final double REVERSE_POWER = 0.15;

    // Field-centric heading offset (для сброса направления)
    private double headingOffset = 0;
    private boolean lastRightStickPressed = false;

    @Override
    public void init() {


        // follower уже содержит pinpoint
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(autoEndPose == null ? new Pose() : autoEndPose);
        follower.setHeadingPIDFCoefficients(new PIDFCoefficients(0.7, 0.1, 0.25, 0));
        follower.update();

        telemetry.addData("Status", "Initialized ✅ (Field Centric + Pinpoint)");
        telemetry.update();
    }

    @Override
    public void start() {

        lastTime = System.currentTimeMillis();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        follower.update();

        // Получаем heading напрямую из follower (Pinpoint локализатор)
        double rawHeading = follower.getPose().getHeading();
        double botHeading = rawHeading - headingOffset;

        // === Field Centric управление ===
        // === Обнуление позы при нажатии правого стика ===
        if (gamepad1.right_stick_button) {
            follower.setPose(new Pose(0, 0, 0)); // или можешь задать другую начальную позу
            telemetry.addData("Pose Reset", "Pose set to (0, 0, 0)");
        }



        // Преобразование в field-centric
        follower.setTeleOpDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);




        // === Телеметрия ===
        telemetry.addData("Pose", "(%.2f, %.2f, %.2f°)",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Heading Offset", Math.toDegrees(headingOffset));
        telemetry.addData("Shooter Vel", vel);
        telemetry.addData("Field Centric Reset", "Press Right Stick");
        telemetry.update();
    }

    @Override
    public void stop() {
        autoEndPose = follower.getPose();

    }


}
