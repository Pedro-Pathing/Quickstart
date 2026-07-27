package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* JADX INFO: loaded from: classes8.dex */
class MrI2cRangeSensorAccess extends HardwareAccess<ModernRoboticsI2cRangeSensor> {
    private final ModernRoboticsI2cRangeSensor mrI2cRangeSensor;

    MrI2cRangeSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, ModernRoboticsI2cRangeSensor.class);
        this.mrI2cRangeSensor = (ModernRoboticsI2cRangeSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {ModernRoboticsI2cRangeSensor.class}, methodName = {"getLightDetected"})
    public double getLightDetected() {
        try {
            startBlockExecution(BlockType.GETTER, ".LightDetected");
            return this.mrI2cRangeSensor.getLightDetected();
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
    @Block(classes = {ModernRoboticsI2cRangeSensor.class}, methodName = {"getRawLightDetected"})
    public double getRawLightDetected() {
        try {
            startBlockExecution(BlockType.GETTER, ".RawLightDetected");
            return this.mrI2cRangeSensor.getRawLightDetected();
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
    @Block(classes = {ModernRoboticsI2cRangeSensor.class}, methodName = {"getRawLightDetectedMax"})
    public double getRawLightDetectedMax() {
        try {
            startBlockExecution(BlockType.GETTER, ".RawLightDetectedMax");
            return this.mrI2cRangeSensor.getRawLightDetectedMax();
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
    @Block(classes = {ModernRoboticsI2cRangeSensor.class}, methodName = {"rawUltrasonic"})
    public double getRawUltrasonic() {
        try {
            startBlockExecution(BlockType.GETTER, ".RawUltrasonic");
            return this.mrI2cRangeSensor.rawUltrasonic();
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
    @Block(classes = {ModernRoboticsI2cRangeSensor.class}, methodName = {"rawOptical"})
    public double getRawOptical() {
        try {
            startBlockExecution(BlockType.GETTER, ".RawOptical");
            return this.mrI2cRangeSensor.rawOptical();
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
    @Block(classes = {ModernRoboticsI2cRangeSensor.class}, methodName = {"cmUltrasonic"})
    public double getCmUltrasonic() {
        try {
            startBlockExecution(BlockType.GETTER, ".CmUltrasonic");
            return this.mrI2cRangeSensor.cmUltrasonic();
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
    @Block(classes = {ModernRoboticsI2cRangeSensor.class}, methodName = {"cmOptical"})
    public double getCmOptical() {
        try {
            startBlockExecution(BlockType.GETTER, ".CmOptical");
            return this.mrI2cRangeSensor.cmOptical();
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
    @Block(classes = {ModernRoboticsI2cRangeSensor.class}, methodName = {"getDistance"})
    public double getDistance(String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getDistance");
            DistanceUnit distanceUnit = (DistanceUnit) checkArg(distanceUnitString, DistanceUnit.class, "unit");
            if (distanceUnit != null) {
                return this.mrI2cRangeSensor.getDistance(distanceUnit);
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
    @Block(classes = {ModernRoboticsI2cRangeSensor.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress7Bit(int i2cAddr7Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress7Bit");
            this.mrI2cRangeSensor.setI2cAddress(I2cAddr.create7bit(i2cAddr7Bit));
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
    @Block(classes = {ModernRoboticsI2cRangeSensor.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress7Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress7Bit");
            I2cAddr i2cAddr = this.mrI2cRangeSensor.getI2cAddress();
            if (i2cAddr != null) {
                return i2cAddr.get7Bit();
            }
            endBlockExecution();
            return 0;
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
    @Block(classes = {ModernRoboticsI2cRangeSensor.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress8Bit(int i2cAddr8Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress8Bit");
            this.mrI2cRangeSensor.setI2cAddress(I2cAddr.create8bit(i2cAddr8Bit));
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
    @Block(classes = {ModernRoboticsI2cRangeSensor.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress8Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress8Bit");
            I2cAddr i2cAddr = this.mrI2cRangeSensor.getI2cAddress();
            if (i2cAddr != null) {
                return i2cAddr.get8Bit();
            }
            endBlockExecution();
            return 0;
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
