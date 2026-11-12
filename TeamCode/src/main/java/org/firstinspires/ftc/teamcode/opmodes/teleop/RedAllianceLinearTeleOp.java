package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.AllianceConfig;
import org.firstinspires.ftc.teamcode.opmodes.RobotConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Red Alliance TeleOp - all control logic lives in BaseLinearTeleOp.
 */
@TeleOp(name = "Red TeleOp", group = "Red")
public class RedAllianceLinearTeleOp extends BaseLinearTeleOp {

    @Override
    protected AllianceConfig getAllianceConfig() {
        return AllianceConfig.createRedAllianceConfig();
    }

    @Override
    protected void handleShootingPosition(){
        boolean bothBumpers1 = gamepad1.left_bumper && gamepad1.right_bumper;
        boolean bothBumpers2 = gamepad2.left_bumper && gamepad2.right_bumper;

        if (!bothBumpers1 && !bothBumpers2) {
            // Bumpers released - stop following; handleManualControl takes over next loop
            drivingToShootingPosition = false;
            return;
        }

        if (drivingToShootingPosition) {
            telemetry.addData("Shooting Position", follower.isBusy() ? "DRIVING" : "ARRIVED");
            return;
        }

        Pose currentPose = follower.pose();
        if(bothBumpers1) {
            Pose targetPose = new Pose(RobotConstants.RED_AF_SHOOTING_POSITION_X, RobotConstants.RED_AF_SHOOTING_POSITION_Y,
                    Math.toRadians(RobotConstants.RED_AF_SHOOTING_POSITION_HEADING_DEG));
            Path path = Paths.line(currentPose, targetPose)
                    .linear(currentPose, targetPose)
                    .with(Constants.foresightConfig.maxPathSpeed.at(RobotConstants.SHOOTING_POSITION_PATH_SPEED));
            follower.follow(path);
            drivingToShootingPosition = true;
        }else if(bothBumpers2){
            Pose targetPose = new Pose(RobotConstants.RED_SF_SHOOTING_POSITION_X, RobotConstants.RED_SF_SHOOTING_POSITION_Y,
                    Math.toRadians(RobotConstants.RED_SF_SHOOTING_POSITION_HEADING_DEG));
            Path path = Paths.line(currentPose, targetPose)
                    .linear(currentPose, targetPose)
                    .with(Constants.foresightConfig.maxPathSpeed.at(RobotConstants.SHOOTING_POSITION_PATH_SPEED));
            follower.follow(path);
            drivingToShootingPosition = true;

        }
    }
}
