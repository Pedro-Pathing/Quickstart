package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.pedropathing.ivy.Scheduler.schedule
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.subsystems.Flywheel

@Disabled
class OtherAuto: LinearOpMode() {

    private val flywheel by lazy {
        Flywheel.get(hardwareMap)
    }

    override fun runOpMode() {
        schedule(

        )
    }

}