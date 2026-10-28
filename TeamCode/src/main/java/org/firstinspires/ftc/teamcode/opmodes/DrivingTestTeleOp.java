package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.TeleOpDrive;
import org.firstinspires.ftc.teamcode.helpers.BulkCache;
import org.firstinspires.ftc.teamcode.helpers.SystemStatus;

/**
 * Driving test TeleOp: just the drive controls in TeleOpDrive (heading lock, robot/field-centric
 * toggle on circle, slow and sprint on the bumpers).
 */
@TeleOp(name = "Driving Test", group = "TeleOp")
@Configurable
public class DrivingTestTeleOp extends OpMode {

    public static Pose startingPose = new Pose(72, 72, 0);

    private final SystemStatus status = new SystemStatus();
    private BulkCache bulkCache;
    private Follower follower;  // null (and drive too) when a drive device is missing
    private TeleOpDrive drive;
    private TelemetryManager telemetryM;

    @Override
    public void init() {
        bulkCache = new BulkCache(hardwareMap);
        follower = TeleOpDrive.createFollower(hardwareMap, startingPose, status);
        drive = follower != null ? new TeleOpDrive(follower) : null;

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addData("Status", "Initialized. Circle toggles robot/field-centric.");
        status.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        if (drive != null) drive.start();
    }

    @Override
    public void loop() {
        bulkCache.clear();
        if (drive != null) {
            follower.update();
            drive.update(gamepad1);
        }

        // Telemetry is only built when Panels is due to send it, to keep the loop fast
        if (telemetryM.getShouldUpdateLines()) {
            status.addTelemetry(telemetryM);
            if (drive != null) drive.addTelemetry(telemetryM);
            telemetryM.update(telemetry);
        }
    }
}
