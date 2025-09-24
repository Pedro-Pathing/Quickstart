package org.firstinspires.ftc.teamcode.opsmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime

//@Autonomous(name = "Blah")
class Blah : LinearOpMode() {

    override fun runOpMode() {

        // 0 - Left Front
        // 1 - Right Front
        // 2 - Left Back
        // 3 - Right Back
        var FL = hardwareMap.get(DcMotor::class.java, "FL")
        var FR = hardwareMap.get(DcMotor::class.java, "FR")
        var BL = hardwareMap.get(DcMotor::class.java, "BL")
        var BR = hardwareMap.get(DcMotor::class.java, "BR")
        var timer = ElapsedTime()
        waitForStart()
        timer.reset()

        while (opModeIsActive()) {

            if (timer.seconds() <= 1.5) {
                FL.power = -1.0
                FR.power = 1.0
                BL.power = -1.0
                BR.power = 1.0
            }
            else if (timer.seconds() > 1.5 && timer.seconds() <= 1.51) {
                FL.power = 1.0
                FR.power = 1.0
                BL.power = 1.0
                BR.power = 1.0
            } else {
                FL.power = 0.0
                FR.power = 0.0
                BL.power = 0.0
                BR.power = 0.0

            }
        }
    }
}