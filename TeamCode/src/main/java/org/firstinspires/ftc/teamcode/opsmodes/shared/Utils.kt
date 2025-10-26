package org.firstinspires.ftc.teamcode.opsmodes.shared

import com.qualcomm.robotcore.hardware.VoltageSensor

class Utils {
    companion object {
        fun getShootingPower(voltageSensor: VoltageSensor): Double {
            var currentVoltage = voltageSensor.voltage

            if (currentVoltage > 13.5) {
                currentVoltage = 13.5
            }
            else if (currentVoltage <= 12.5) {
                currentVoltage = 12.5
            }

            return ((13.5 - currentVoltage) / 4) + .75
        }
    }
}