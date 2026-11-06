package org.firstinspires.ftc.teamcode.commands.field;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.util.PosePersistency;

import java.util.function.Supplier;

@Config
public class ResetPose extends InstantCommand {
    public static double RED_X_IN = 8;
    public static double RED_Y_IN = 8;
    public static double RED_HEADING_DEG = 90;

    public static double BLUE_X_IN = 133;
    public static double BLUE_Y_IN = 132;
    public static double BLUE_HEADING_DEG = -90;

    public ResetPose(Follower follower, Field.Alliance alliance) {
        this(follower, () -> alliance);
    }

    public ResetPose(Follower follower, Supplier<Field.Alliance> alliance) {
        super(() -> {
            Pose pose = poseFor(alliance.get());
            follower.setPose(pose);
            PosePersistency.savePose(pose);
        });
    }

    public static Pose poseFor(Field.Alliance alliance) {
        if (alliance == Field.Alliance.RED) {
            return new Pose(RED_X_IN, RED_Y_IN, Math.toRadians(RED_HEADING_DEG));
        }
        return new Pose(BLUE_X_IN, BLUE_Y_IN, Math.toRadians(BLUE_HEADING_DEG));
    }
}
