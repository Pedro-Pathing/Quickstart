package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


/**
 * This is the OffsetsTuner OpMode. This tracks the movement of the robot as it turns 180 degrees,
 * and calculates what the robot's strafeX and forwardY offsets should be. Ensure that your strafeX and forwardY offsets
 * are set to 0 before running this OpMode. After running, input the displayed offsets into your localizer constants.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
@TeleOp(group = "1")
public class OffsetsTuner extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(70.75,70.75);

    @Override
    public void init() {
        Constants.localizerConfig.xPodOffset.set(0.0);
        Constants.localizerConfig.yPodOffset.set(0.0);
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        follower.update();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Prerequisite: Make sure both your offsets are set to 0 in your localizer constants.");
        telemetry.addLine("Turn your robot " + Math.PI + " radians. Your offsets in inches will be shown on the telemetry.");
        telemetry.update();
    }

    /**
     * This updates the robot's pose estimate, and updates telemetry with the calculated offsets
     */
    @Override
    public void loop() {
        follower.update();
        telemetry.addLine("The following values are the offsets in inches that should be applied to your localizer.");
        telemetry.addLine("heading: " + (startPose.heading() - follower.pose().heading()));
        telemetry.addLine("xPodOffset: " + ((startPose.y()-follower.pose().y()) / 2.0));
        telemetry.addLine("yPodOffset: " + ((startPose.x()-follower.pose().x()) / 2.0));
        telemetry.update();
    }
}
