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
 * Red Alliance center autonomous:
 * 1. Move to near launch position and pause (add mechanism states here)
 * 2. PPG collection cycle (move to intake start, slow move to intake end)
 * 3. Return to auton end position
 *
 * Alliance-specific poses come from AllianceConfig / LaunchTarget; the sequence itself is shared logic.
 */
@Autonomous(name = "Red-Center-AF", group = "Red-Center-AF")
public class RedCenterAutonomousAF extends BaseLinearAutonomous {

    @Override
    protected AllianceConfig getAllianceConfig() {
        return AllianceConfig.createRedAllianceConfigAF();
    }

    @Override
    protected List<StateNode> buildStateSequence() {
        List<StateNode> sequence = new ArrayList<>();

        // Start launcher motors
        sequence.add(state(AutonomousState.START_POLLEN_LAUNCHER));
        sequence.add(waitMs(3000));
        sequence.add(state(AutonomousState.CLOSE_POLLEN_SERVO));
        sequence.add(waitMs(1000));
        sequence.add(state(AutonomousState.START_INTAKE));
        sequence.add(waitMs(6000));
        sequence.add(state(AutonomousState.STOP_POLLEN_LAUNCHER));
        sequence.add(state(AutonomousState.STOP_INTAKE));
        sequence.add(waitMs(500));
        sequence.add(state(AutonomousState.OPEN_POLLEN_SERVO));
        // Return to auton end position
        sequence.add(state(AutonomousState.RETURN_HOME));
        return sequence;
    }

    /**
     * Park from the launch spot in front of the hive (~(55, 9, 90°)) to AUTON_END (12, 96, 90°).
     * The default park() curve cuts diagonally past the hive's corner (~x = 48), so here the robot first
     * slides left along its starting row, gets onto the wall line, and only then drives up to park.
     */
    @Override
    protected Path park() {
        Pose start = follower.pose();
        Pose end = allianceConfig.getAutonEndPose();
        double startHrading = Math.toDegrees(start.heading());
        // Push the control points away from the nearest side wall (x = 0 for Red, x = 144 for Blue)
        Pose control1 = new Pose(start.x() , start.y()+RobotConstants.PARK_WALL_CLEARANCE_IN, 0);  // Bezier control points; their headings are unused
        Pose control2 = new Pose(end.x()+12 , end.y()-50, 0);

        return Paths.curve(start, control1, control2, end)
                .linear(start, end)
                .with(Constants.foresightConfig.maxPathSpeed.at(RobotConstants.TEST_PATH_SPEED));
    }
}
