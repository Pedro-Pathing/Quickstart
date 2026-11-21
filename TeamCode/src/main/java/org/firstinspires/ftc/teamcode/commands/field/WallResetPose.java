package org.firstinspires.ftc.teamcode.commands.field;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.util.PosePersistency;

@Config
public class WallResetPose extends InstantCommand {
    public static double ROBOT_LENGTH_IN = 18;
    public static double ROBOT_WIDTH_IN = 18;

    public enum Axis {
        X, Y
    }

    public WallResetPose(Follower follower, Axis axis) {
        super(() -> {
            Pose pose = resetAgainstWall(follower.pose(), axis);
            follower.setPose(pose);
            PosePersistency.savePose(pose);
        });
    }

    public static Pose resetAgainstWall(Pose pose, Axis axis) {
        double halfLength = ROBOT_LENGTH_IN / 2;
        double halfWidth = ROBOT_WIDTH_IN / 2;
        double cos = Math.abs(Math.cos(pose.heading()));
        double sin = Math.abs(Math.sin(pose.heading()));

        if (axis == Axis.X) {
            double extent = cos * halfLength + sin * halfWidth;
            double x = pose.x() < Field.CENTER_X ? extent : Field.WIDTH - extent;
            return new Pose(x, pose.y(), pose.heading());
        }
        double extent = sin * halfLength + cos * halfWidth;
        double y = pose.y() < Field.CENTER_Y ? extent : Field.WIDTH - extent;
        return new Pose(pose.x(), y, pose.heading());
    }
}
