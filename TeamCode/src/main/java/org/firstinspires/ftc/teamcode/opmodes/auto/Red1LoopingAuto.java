package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.Alliance;

/**
 * Red 1: starts with its back to the audience wall, between the red GARDEN and the centre line, and works the
 * red audience-side quarter: along the audience wall through the red GARDEN, then up the red wall short of its
 * FLOWER, shooting at the audience-side CELL.
 */
@Autonomous(name = "Red 1 Looping Auto", group = "Autonomous")
public class Red1LoopingAuto extends LoopingAuto {

    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }

    @Override
    protected Pose startPose() {
        return poseFactory.of(50, 10.4, 90);
    }

    @Override
    protected Pose shootingSpot() {
        return poseFactory.of(40, 26, 90);
    }

    @Override
    protected Pose[] collectingLoop() {
        return new Pose[] {
                poseFactory.of(20, 12, 180),
                poseFactory.of(16, 34, 90),
                poseFactory.of(30, 40, 0),
        };
    }

    @Override
    protected Pose flowerPose() {
        return poseFactory.of(12.8, 48.6, 180);  // the FLOWER on the red wall
    }
}
