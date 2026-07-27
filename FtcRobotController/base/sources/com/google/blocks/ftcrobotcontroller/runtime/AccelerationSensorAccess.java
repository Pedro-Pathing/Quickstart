package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

/* JADX INFO: loaded from: classes8.dex */
class AccelerationSensorAccess extends HardwareAccess<AccelerationSensor> {
    private final AccelerationSensor accelerationSensor;

    AccelerationSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, AccelerationSensor.class);
        this.accelerationSensor = (AccelerationSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public double getXAccel() {
        try {
            startBlockExecution(BlockType.GETTER, ".XAccel");
            Acceleration acceleration = this.accelerationSensor.getAcceleration();
            if (acceleration != null) {
                return acceleration.xAccel;
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

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public double getYAccel() {
        try {
            startBlockExecution(BlockType.GETTER, ".YAccel");
            Acceleration acceleration = this.accelerationSensor.getAcceleration();
            if (acceleration != null) {
                return acceleration.yAccel;
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

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public double getZAccel() {
        try {
            startBlockExecution(BlockType.GETTER, ".ZAccel");
            Acceleration acceleration = this.accelerationSensor.getAcceleration();
            if (acceleration != null) {
                return acceleration.zAccel;
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

    @JavascriptInterface
    @Block(classes = {AccelerationSensor.class}, methodName = {"getAcceleration"})
    public Acceleration getAcceleration() {
        try {
            startBlockExecution(BlockType.GETTER, ".Acceleration");
            return this.accelerationSensor.getAcceleration();
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public String toText() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            return this.accelerationSensor.getAcceleration().toString();
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
