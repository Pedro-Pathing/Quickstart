package org.firstinspires.ftc.teamcode.opmodes.auton.red;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AllianceConfig;
import org.firstinspires.ftc.teamcode.opmodes.RobotConstants;
import org.firstinspires.ftc.teamcode.opmodes.auton.BaseLinearAutonomous;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Red Alliance center autonomous (SF):
 * 1. Wait until the Limelight sees at least RobotConstants.TIP_CLUSTER_MIN_PERCENT of the SF (32-35) goal tags
 *    (gives up and parks at RobotConstants.TIP_WAIT_GIVE_UP_MS)
 * 2. Spin up the pollen launcher, close the pollen gate and feed with the intake
 * 3. Stop the launcher and intake
 * 4. Return to auton end position (AUTON_END_SF)
 *
 * Alliance-specific poses come from AllianceConfig / LaunchTarget; the sequence itself is shared logic.
 */
@Autonomous(name = "Red-Center-SF", group = "Red-Center-SF")
public class RedCenterAutonomousSF extends BaseLinearAutonomous {

    @Override
    protected AllianceConfig getAllianceConfig() {
        return AllianceConfig.createRedAllianceConfigSF();
    }

    @Override
    protected List<StateNode> buildStateSequence() {
        List<StateNode> sequence = new ArrayList<>();

        // Start launcher motors
        sequence.add(state(AutonomousState.WAIT_FOR_TIP));
        sequence.add(state(AutonomousState.START_POLLEN_LAUNCHER));
        sequence.add(waitMs(3000));
        sequence.add(state(AutonomousState.CLOSE_POLLEN_SERVO));
        sequence.add(state(AutonomousState.START_INTAKE));
        sequence.add(waitMs(4000));
        sequence.add(state(AutonomousState.STOP_POLLEN_LAUNCHER));
        sequence.add(state(AutonomousState.STOP_INTAKE));
        // Return to auton end position
        sequence.add(state(AutonomousState.RETURN_HOME));
        return sequence;
    }

    @Override
    protected Path park() {
        Pose start = follower.pose();
        Pose end = allianceConfig.getAutonEndPose();
        double startHrading = Math.toDegrees(start.heading());
        // Push the control points away from the nearest side wall (x = 0 for Red, x = 144 for Blue)
        Pose control1 = new Pose(start.x() , start.y()-10, 0);  // Bezier control points; their headings are unused
        //Pose control2 = new Pose(end.x()+12 , end.y()-50, 0);

        return Paths.curve(start, control1, end)
                .linear(start, end)
                .with(Constants.foresightConfig.maxPathSpeed.at(RobotConstants.TEST_PATH_SPEED));
    }
}
