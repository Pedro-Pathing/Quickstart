package org.firstinspires.ftc.teamcode.opsmodes.next.opmodes

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Next Hivemind - Blue Bottom")
open class NextBlueBottomHivemindAuto : NextBaseAuto() {
    override fun buildPathList(): List<Pose> {
        return listOf(
            Pose(0.0, 0.0),
            Pose(90.0, 0.0, Math.toRadians(50.0)),
            Pose(120.0, 0.0, Math.toRadians(90.0))
        )
    }
}

@Autonomous(name = "Next Hivemind - Red Bottom")
class NextRedBottomHivemindAuto : NextBlueBottomHivemindAuto() {
    override fun buildPathList(): List<Pose> {
        val pathList = super.buildPathList()
        return listOf(
            pathList[0].mirror(),
            pathList[1].mirror(),
            pathList[2].mirror(),
        )
    }
}

@Autonomous(name = "Next Hivemind - Blue Top")
open class NextBlueTopHivemindAuto : NextBaseAuto() {
    override fun buildPathList(): List<Pose> {
        return listOf(
            Pose(0.0, 0.0),
            Pose(38.0, 0.0, Math.toRadians(185.0)),
            Pose(24.0, 24.0, Math.toRadians(-135.0))
        )
    }
}

@Autonomous(name = "Next Hivemind - Red Top")
class NextRedTopHivemindAuto : NextBlueTopHivemindAuto() {
    override fun buildPathList(): List<Pose> {
        val pathList = super.buildPathList()
        return listOf(
            pathList[0],
            Pose(pathList[1].x, pathList[1].y, -pathList[1].heading),
            Pose(pathList[2].x, -pathList[2].y, -pathList[2].heading),
        )
    }
}