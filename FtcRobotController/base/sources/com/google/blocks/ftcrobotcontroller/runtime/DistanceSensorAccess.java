package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* JADX INFO: loaded from: classes8.dex */
class DistanceSensorAccess extends HardwareAccess<DistanceSensor> {
    private final DistanceSensor distanceSensor;

    DistanceSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, DistanceSensor.class);
        this.distanceSensor = (DistanceSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {DistanceSensor.class}, methodName = {"getDistance"})
    public double getDistance(String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getDistance");
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (distanceUnit != null) {
                return this.distanceSensor.getDistance(distanceUnit);
            }
            endBlockExecution();
            return LynxServoController.apiPositionFirst;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }
}
