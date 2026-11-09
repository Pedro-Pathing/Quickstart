package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.Alliance;

/**
 * Red 2: starts with its back to the red wall, between the red wall's FLOWER and the red LOADING ZONE, and works
 * the red scoring-side quarter: across towards the rear wall short of its FLOWER, then down through the red LOADING
 * ZONE (its NECTAR), shooting at the scoring-side CELL.
 */
@Autonomous(name = "SIM Red 2 Looping Auto", group = "Sim")
public class Red2LoopingAuto extends LoopingAuto {

    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }

    @Override
    protected Pose startPose() {
        return poseFactory.of(10.4, 72, 0);
    }

    @Override
    protected Pose shootingSpot() {
        return poseFactory.of(40, 118, 90);
    }

    @Override
    protected Pose[] collectingLoop() {
        return new Pose[] {
                poseFactory.of(22, 130, 180),
                poseFactory.of(16, 106, 270),
                poseFactory.of(28, 92, 0),
        };
    }

    @Override
    protected Pose flowerPose() {
        return poseFactory.of(48.6, 131.2, 90);  // the FLOWER on the scoring wall
    }

    @Override
    protected boolean waitsForPartner() {
        return true;
    }
}
