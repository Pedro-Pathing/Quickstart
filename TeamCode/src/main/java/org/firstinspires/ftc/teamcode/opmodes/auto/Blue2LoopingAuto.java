package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.Alliance;

/**
 * Blue 2: starts with its back to the blue wall, between the blue LOADING ZONE and the blue wall's FLOWER, and works
 * the blue audience-side quarter: towards the audience wall short of its FLOWER, then up through the blue LOADING
 * ZONE (its NECTAR), shooting at the audience-side CELL.
 */
@Autonomous(name = "Blue 2 Looping Auto", group = "Autonomous")
public class Blue2LoopingAuto extends LoopingAuto {

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected Pose startPose() {
        return poseFactory.of(133.6, 72, 180);
    }

    @Override
    protected Pose shootingSpot() {
        return poseFactory.of(104, 26, 270);
    }

    @Override
    protected Pose[] collectingLoop() {
        return new Pose[] {
                poseFactory.of(122, 14, 0),
                poseFactory.of(128, 38, 90),
                poseFactory.of(114, 50, 180),
        };
    }

    @Override
    protected Pose flowerPose() {
        return poseFactory.of(95.4, 12.8, 270);  // the FLOWER on the audience wall
    }

    @Override
    protected boolean waitsForPartner() {
        return true;
    }
}
