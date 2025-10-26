package org.firstinspires.ftc.teamcode.opsmodes.auto

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "Hivemind - Blue Bottom")
open class BlueBottomHivemindAuto : BaseAuto() {
    override fun buildPathList(): List<Pose> {
        return listOf(
            Pose(0.0, 0.0),
            Pose(94.0, 0.0, Math.toRadians(45.0)),
            Pose(120.0, 0.0, Math.toRadians(90.0))
        )
    }
}

@Autonomous(name = "Hivemind - Red Bottom")
class RedBottomHivemindAuto : BlueBottomHivemindAuto() {
    override fun buildPathList(): List<Pose> {
        val pathList = super.buildPathList()
        return listOf(
            pathList[0].mirror(),
            pathList[1].mirror(),
            pathList[2].mirror(),
        )
    }
}

@Autonomous(name = "Hivemind - Blue Top")
open class BlueTopHivemindAuto : BaseAuto() {
    override fun buildPathList(): List<Pose> {
        return listOf(
            Pose(0.0, 0.0),
            Pose(48.0, 0.0, Math.toRadians(180.0)),
            Pose(24.0, 24.0, Math.toRadians(-135.0))
        )
    }
}

@Autonomous(name = "Hivemind - Red Top")
class RedTopHivemindAuto : BlueTopHivemindAuto() {
    override fun buildPathList(): List<Pose> {
        val pathList = super.buildPathList()
        return listOf(
            pathList[0],
            pathList[1],
            Pose(pathList[2].x, -pathList[2].y, -pathList[2].heading),
        )
    }
}