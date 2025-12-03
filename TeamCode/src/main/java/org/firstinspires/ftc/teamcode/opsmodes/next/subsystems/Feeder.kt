package org.firstinspires.ftc.teamcode.opsmodes.next.subsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

object Feeder : Subsystem {
    private val servo = ServoEx("feeder", -1.0)

    val open = SetPosition(servo, 0.5).requires(this)
    val close = SetPosition(servo, 1.0).requires(this)

    override fun initialize() {
        servo.position = 1.0
    }
}