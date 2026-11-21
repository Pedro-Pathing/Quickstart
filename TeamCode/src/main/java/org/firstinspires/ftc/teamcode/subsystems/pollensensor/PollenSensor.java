package org.firstinspires.ftc.teamcode.subsystems.pollensensor;

import static org.firstinspires.ftc.teamcode.subsystems.pollensensor.PollenSensorConstants.*;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PollenSensor extends SubsystemBase {
    private final RevColorSensorV3 sensor1;
    private final RevColorSensorV3 sensor2;
    private final ElapsedTime clearTimer = new ElapsedTime();

    private double distance1 = Double.NaN;
    private double distance2 = Double.NaN;
    private boolean seen1 = false;
    private boolean seen2 = false;
    private boolean present = false;

    public PollenSensor(HardwareMap hardwareMap) {
        sensor1 = hardwareMap.get(RevColorSensorV3.class, SENSOR_1_NAME);
        sensor2 = hardwareMap.get(RevColorSensorV3.class, SENSOR_2_NAME);
    }

    @Override
    public void periodic() {
        distance1 = sensor1.getDistance(DistanceUnit.CM);
        distance2 = sensor2.getDistance(DistanceUnit.CM);
        seen1 = distance1 < PRESENT_DISTANCE_CM;
        seen2 = distance2 < PRESENT_DISTANCE_CM;

        if (seen1 || seen2) {
            clearTimer.reset();
            present = true;
        } else if (present && clearTimer.seconds() >= CLEAR_TIME_S) {
            present = false;
        }
    }

    public boolean isPresent() {
        return present;
    }

    public double getDistance1() {
        return distance1;
    }

    public double getDistance2() {
        return distance2;
    }

    public void addTelemetry(TelemetryData telemetry) {
        telemetry.addData("Pollen present", present);
        telemetry.addData("Pollen sensor 1 cm", distance1);
        telemetry.addData("Pollen sensor 2 cm", distance2);
        telemetry.addData("Pollen seen 1 / 2", seen1 + " / " + seen2);
    }
}
