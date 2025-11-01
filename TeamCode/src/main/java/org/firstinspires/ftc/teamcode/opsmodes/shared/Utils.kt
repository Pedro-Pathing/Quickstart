package org.firstinspires.ftc.teamcode.opsmodes.shared

import com.qualcomm.robotcore.hardware.VoltageSensor

class Utils {
    companion object {
        fun getShootingPower(voltageSensor: VoltageSensor): Double {
            if (voltageSensor.voltage < 12.0) {
                return 0.75
            } else if (voltageSensor.voltage < 12.5) {
                return 0.70
            } else if (voltageSensor.voltage < 13.0) {
                return 0.70
            } else {
                return .65
            }
        }
    }
}