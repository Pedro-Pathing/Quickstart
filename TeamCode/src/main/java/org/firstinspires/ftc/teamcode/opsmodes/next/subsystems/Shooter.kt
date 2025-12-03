package org.firstinspires.ftc.teamcode.opsmodes.next.subsystems

import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.opsmodes.shared.Utils

object Shooter : Subsystem {
    private val motor = MotorEx("shooter", -1.0)
    lateinit var voltageSensor: VoltageSensor
    private var enabled = false

    val on = LambdaCommand()
        .setStart { enabled = true }
        .setIsDone { true }
        .requires(this)

    val off = LambdaCommand()
        .setStart { enabled = false }
        .setIsDone { true }
        .requires(this)

    override fun initialize() {
        voltageSensor = hardwareMap.get(VoltageSensor::class.java, "Control Hub")
        motor.power = 0.0
    }

    override fun periodic() {
        if (enabled) {
            motor.power = Utils.getShootingPower(voltageSensor)
        } else {
            motor.power = 0.0
        }
    }
}
