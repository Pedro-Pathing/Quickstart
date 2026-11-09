package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.Alliance;

/**
 * Blue 1: starts with its back to the rear (scoring-side) wall, between the centre line and the blue GARDEN, and
 * works the blue scoring-side quarter: along the rear wall through the blue GARDEN, then down the blue wall short of
 * its FLOWER, shooting at the scoring-side CELL.
 */
@Autonomous(name = "Blue 1 Looping Auto", group = "Autonomous")
public class Blue1LoopingAuto extends LoopingAuto {

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected Pose startPose() {
        return poseFactory.of(94, 133.6, 270);
    }

    @Override
    protected Pose shootingSpot() {
        return poseFactory.of(104, 118, 270);
    }

    @Override
    protected Pose[] collectingLoop() {
        return new Pose[] {
                poseFactory.of(124, 132, 0),
                poseFactory.of(128, 110, 270),
                poseFactory.of(114, 104, 180),
        };
    }

    @Override
    protected Pose flowerPose() {
        return poseFactory.of(131.2, 95.4, 0);  // the FLOWER on the blue wall
    }
}
