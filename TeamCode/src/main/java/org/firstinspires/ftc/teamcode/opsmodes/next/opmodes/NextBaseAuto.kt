package org.firstinspires.ftc.teamcode.opsmodes.next.opmodes

import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opsmodes.next.subsystems.Feeder
import org.firstinspires.ftc.teamcode.opsmodes.next.subsystems.Shooter
import org.firstinspires.ftc.teamcode.opsmodes.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

abstract class NextBaseAuto : NextFTCOpMode() {
    lateinit var shootPath: PathChain
    lateinit var endPath: PathChain

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(Shooter, Feeder),
            BulkReadComponent
        )
    }

    abstract fun buildPathList(): List<Pose>


    override fun onStartButtonPressed() {
        SequentialGroup(
            // Wait 3 seconds
            Delay(1.seconds),

            // Start shooter
            Shooter.on,

            // Drive to shoot location
            FollowPath(shootPath),

            // First shot
            Delay(3.seconds),
            Feeder.open,
            Delay(0.2.seconds),
            Feeder.close,

            // Second shot
            Delay(3.seconds),
            Feeder.open,
            Delay(0.2.seconds),
            Feeder.close,

            // Third shot
            Delay(3.seconds),
            Feeder.open,
            Delay(0.2.seconds),
            Feeder.close,

            // Turn off shooter
            Shooter.off,

            // Drive to park location
            FollowPath(endPath),
        )()
    }

    override fun onInit() {
        buildPath()
    }

    override fun onUpdate() {
        ActiveOpMode.telemetry.update()
    }

    override fun onWaitForStart() {
        ActiveOpMode.telemetry.update()
    }

    private fun buildPath() {
        val pathList = buildPathList();
        val startingPose = pathList[0]
        val shootingPose = pathList[1]
        val endPose = pathList[2]

        follower.setStartingPose(startingPose)

        shootPath = follower.pathBuilder()
            .setGlobalDeceleration()
            .addPath(BezierLine(startingPose, shootingPose))
            .setLinearHeadingInterpolation(startingPose.heading, shootingPose.heading)
            .build()

        endPath = follower.pathBuilder()
            .setGlobalDeceleration()
            .addPath(BezierLine(shootingPose, endPose))
            .setLinearHeadingInterpolation(shootingPose.heading, endPose.heading)
            .build()
    }
}