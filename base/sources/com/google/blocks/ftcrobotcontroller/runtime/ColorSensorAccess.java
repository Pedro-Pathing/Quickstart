package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/* JADX INFO: loaded from: classes8.dex */
class ColorSensorAccess extends HardwareAccess<ColorSensor> {
    private final ColorSensor colorSensor;

    ColorSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, ColorSensor.class);
        this.colorSensor = (ColorSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {AdafruitI2cColorSensor.class, ColorSensor.class, ModernRoboticsI2cColorSensor.class}, methodName = {"red"})
    public int getRed() {
        try {
            startBlockExecution(BlockType.GETTER, ".Red");
            return this.colorSensor.red();
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
    @Block(classes = {AdafruitI2cColorSensor.class, ColorSensor.class, ModernRoboticsI2cColorSensor.class}, methodName = {"green"})
    public int getGreen() {
        try {
            startBlockExecution(BlockType.GETTER, ".Green");
            return this.colorSensor.green();
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
    @Block(classes = {AdafruitI2cColorSensor.class, ColorSensor.class, ModernRoboticsI2cColorSensor.class}, methodName = {"blue"})
    public int getBlue() {
        try {
            startBlockExecution(BlockType.GETTER, ".Blue");
            return this.colorSensor.blue();
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
    @Block(classes = {AdafruitI2cColorSensor.class, ColorSensor.class, ModernRoboticsI2cColorSensor.class}, methodName = {"alpha"})
    public int getAlpha() {
        try {
            startBlockExecution(BlockType.GETTER, ".Alpha");
            return this.colorSensor.alpha();
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
    @Block(classes = {AdafruitI2cColorSensor.class, ColorSensor.class, ModernRoboticsI2cColorSensor.class}, methodName = {"argb"})
    public int getArgb() {
        try {
            startBlockExecution(BlockType.GETTER, ".Argb");
            return this.colorSensor.argb();
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
    @Block(classes = {AdafruitI2cColorSensor.class, ColorSensor.class, ModernRoboticsI2cColorSensor.class}, methodName = {"enableLed"})
    public void enableLed(boolean enable) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".enableLed");
            this.colorSensor.enableLed(enable);
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
    @Block(classes = {ModernRoboticsI2cColorSensor.class}, methodName = {"enableLight"})
    public void enableLight(boolean enable) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".enableLight");
            if (this.colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) this.colorSensor).enableLight(enable);
            } else {
                reportWarning("This ColorSensor is not a SwitchableLight.");
            }
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
    @Block(classes = {AdafruitI2cColorSensor.class, Light.class, ModernRoboticsI2cColorSensor.class}, methodName = {"isLightOn"})
    public boolean isLightOn() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isLightOn");
            if (this.colorSensor instanceof Light) {
                return ((Light) this.colorSensor).isLightOn();
            }
            reportWarning("This ColorSensor is not a Light.");
            endBlockExecution();
            return false;
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
    @Block(classes = {AdafruitI2cColorSensor.class, ColorSensor.class, ModernRoboticsI2cColorSensor.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress7Bit(int i2cAddr7Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress7Bit");
            this.colorSensor.setI2cAddress(I2cAddr.create7bit(i2cAddr7Bit));
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
    @Block(classes = {AdafruitI2cColorSensor.class, ColorSensor.class, ModernRoboticsI2cColorSensor.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress7Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress7Bit");
            I2cAddr i2cAddr = this.colorSensor.getI2cAddress();
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
    @Block(classes = {AdafruitI2cColorSensor.class, ColorSensor.class, ModernRoboticsI2cColorSensor.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress8Bit(int i2cAddr8Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress8Bit");
            this.colorSensor.setI2cAddress(I2cAddr.create8bit(i2cAddr8Bit));
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
    @Block(classes = {AdafruitI2cColorSensor.class, ColorSensor.class, ModernRoboticsI2cColorSensor.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress8Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress8Bit");
            I2cAddr i2cAddr = this.colorSensor.getI2cAddress();
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
    @Block(classes = {AdafruitI2cColorSensor.class, ModernRoboticsI2cColorSensor.class}, methodName = {"toString"})
    public String toText() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            return this.colorSensor.toString();
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
    @Block(classes = {AdafruitI2cColorSensor.class, ModernRoboticsI2cColorSensor.class, NormalizedColorSensor.class}, methodName = {"getNormalizedColors"})
    public String getNormalizedColors() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getNormalizedColors");
            if (this.colorSensor instanceof NormalizedColorSensor) {
                NormalizedRGBA color = ((NormalizedColorSensor) this.colorSensor).getNormalizedColors();
                return "{ \"Red\":" + color.red + ", \"Green\":" + color.green + ", \"Blue\":" + color.blue + ", \"Alpha\":" + color.alpha + ", \"Color\":" + color.toColor() + " }";
            }
            return "{ \"Red\":0, \"Green\":0, \"Blue\":0, \"Alpha\":0, \"Color\":0 }";
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
    @Block(classes = {AdafruitI2cColorSensor.class, ModernRoboticsI2cColorSensor.class, NormalizedColorSensor.class}, methodName = {"setGain"})
    public void setGain(float gain) {
        try {
            startBlockExecution(BlockType.SETTER, ".Gain");
            if (this.colorSensor instanceof NormalizedColorSensor) {
                ((NormalizedColorSensor) this.colorSensor).setGain(gain);
            }
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
    @Block(classes = {AdafruitI2cColorSensor.class, ModernRoboticsI2cColorSensor.class, NormalizedColorSensor.class}, methodName = {"getGain"})
    public float getGain() {
        try {
            startBlockExecution(BlockType.GETTER, ".Gain");
            if (this.colorSensor instanceof NormalizedColorSensor) {
                return ((NormalizedColorSensor) this.colorSensor).getGain();
            }
            endBlockExecution();
            return 0.0f;
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
