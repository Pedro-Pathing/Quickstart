package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* JADX INFO: loaded from: classes8.dex */
class ColorRangeSensorAccess extends HardwareAccess<ColorRangeSensor> {
    private final ColorRangeSensor colorRangeSensor;

    ColorRangeSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, ColorRangeSensor.class);
        this.colorRangeSensor = (ColorRangeSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"red"})
    public int getRed() {
        try {
            startBlockExecution(BlockType.GETTER, ".Red");
            return this.colorRangeSensor.red();
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
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"green"})
    public int getGreen() {
        try {
            startBlockExecution(BlockType.GETTER, ".Green");
            return this.colorRangeSensor.green();
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
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"blue"})
    public int getBlue() {
        try {
            startBlockExecution(BlockType.GETTER, ".Blue");
            return this.colorRangeSensor.blue();
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
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"alpha"})
    public int getAlpha() {
        try {
            startBlockExecution(BlockType.GETTER, ".Alpha");
            return this.colorRangeSensor.alpha();
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
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"argb"})
    public int getArgb() {
        try {
            startBlockExecution(BlockType.GETTER, ".Argb");
            return this.colorRangeSensor.argb();
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
    @Block(classes = {LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress7Bit(int i2cAddr7Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress7Bit");
            this.colorRangeSensor.setI2cAddress(I2cAddr.create7bit(i2cAddr7Bit));
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
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress7Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress7Bit");
            I2cAddr i2cAddr = this.colorRangeSensor.getI2cAddress();
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
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress8Bit(int i2cAddr8Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress8Bit");
            this.colorRangeSensor.setI2cAddress(I2cAddr.create8bit(i2cAddr8Bit));
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
    @Block(classes = {LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress8Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress8Bit");
            I2cAddr i2cAddr = this.colorRangeSensor.getI2cAddress();
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

    @JavascriptInterface
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"getLightDetected"})
    public double getLightDetected() {
        try {
            startBlockExecution(BlockType.GETTER, ".LightDetected");
            return this.colorRangeSensor.getLightDetected();
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
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"getRawLightDetected"})
    public double getRawLightDetected() {
        try {
            startBlockExecution(BlockType.GETTER, ".RawLightDetected");
            return this.colorRangeSensor.getRawLightDetected();
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
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"getRawLightDetectedMax"})
    public double getRawLightDetectedMax() {
        try {
            startBlockExecution(BlockType.GETTER, ".RawLightDetectedMax");
            return this.colorRangeSensor.getRawLightDetectedMax();
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
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"getDistance"})
    public double getDistance(String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getDistance");
            DistanceUnit distanceUnit = (DistanceUnit) checkArg(distanceUnitString, DistanceUnit.class, "unit");
            if (distanceUnit != null) {
                return this.colorRangeSensor.getDistance(distanceUnit);
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
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"getNormalizedColors"})
    public String getNormalizedColors() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getNormalizedColors");
            NormalizedRGBA color = this.colorRangeSensor.getNormalizedColors();
            return "{ \"Red\":" + color.red + ", \"Green\":" + color.green + ", \"Blue\":" + color.blue + ", \"Alpha\":" + color.alpha + ", \"Color\":" + color.toColor() + " }";
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
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"setGain"})
    public void setGain(float gain) {
        try {
            startBlockExecution(BlockType.SETTER, ".Gain");
            this.colorRangeSensor.setGain(gain);
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
    @Block(classes = {ColorRangeSensor.class, LynxI2cColorRangeSensor.class, RevColorSensorV3.class}, methodName = {"getGain"})
    public float getGain() {
        try {
            startBlockExecution(BlockType.GETTER, ".Gain");
            return this.colorRangeSensor.getGain();
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
