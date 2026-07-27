package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.andymark.AndyMarkColorSensor;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* JADX INFO: loaded from: classes8.dex */
class AndyMarkColorSensorAccess extends HardwareAccess<AndyMarkColorSensor> {
    private final AndyMarkColorSensor andyMarkColorSensor;

    AndyMarkColorSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, AndyMarkColorSensor.class);
        this.andyMarkColorSensor = (AndyMarkColorSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"alpha"})
    public int getAlpha() {
        try {
            startBlockExecution(BlockType.GETTER, ".Alpha");
            return this.andyMarkColorSensor.alpha();
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"argb"})
    public int getArgb() {
        try {
            startBlockExecution(BlockType.GETTER, ".Argb");
            return this.andyMarkColorSensor.argb();
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"red"})
    public int getRed() {
        try {
            startBlockExecution(BlockType.GETTER, ".Red");
            return this.andyMarkColorSensor.red();
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"green"})
    public int getGreen() {
        try {
            startBlockExecution(BlockType.GETTER, ".Green");
            return this.andyMarkColorSensor.green();
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"blue"})
    public int getBlue() {
        try {
            startBlockExecution(BlockType.GETTER, ".Blue");
            return this.andyMarkColorSensor.blue();
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"getLightDetected"})
    public double getLightDetected() {
        try {
            startBlockExecution(BlockType.GETTER, ".LightDetected");
            return this.andyMarkColorSensor.getLightDetected();
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress7Bit(int i2cAddr7Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress7Bit");
            this.andyMarkColorSensor.setI2cAddress(I2cAddr.create7bit(i2cAddr7Bit));
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress7Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress7Bit");
            I2cAddr i2cAddr = this.andyMarkColorSensor.getI2cAddress();
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress8Bit(int i2cAddr8Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress8Bit");
            this.andyMarkColorSensor.setI2cAddress(I2cAddr.create8bit(i2cAddr8Bit));
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress8Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress8Bit");
            I2cAddr i2cAddr = this.andyMarkColorSensor.getI2cAddress();
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"getDistance"})
    public double getDistance(String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getDistance");
            DistanceUnit distanceUnit = (DistanceUnit) checkArg(distanceUnitString, DistanceUnit.class, "unit");
            if (distanceUnit != null) {
                return this.andyMarkColorSensor.getDistance(distanceUnit);
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"getNormalizedColors"})
    public String getNormalizedColors() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getNormalizedColors");
            NormalizedRGBA color = this.andyMarkColorSensor.getNormalizedColors();
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"setProximityGain"})
    public void setProximityGain(String gainString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setProximityGain");
            AndyMarkColorSensor.ProximityGain gain = (AndyMarkColorSensor.ProximityGain) checkArg(gainString, AndyMarkColorSensor.ProximityGain.class, "gain");
            if (gain != null) {
                this.andyMarkColorSensor.setProximityGain(gain);
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"setProximityLedPulses"})
    public void setProximityLedPulses(int pulses) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setProximityLedPulses");
            this.andyMarkColorSensor.setProximityLedPulses(pulses);
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"setProximityLedPulseLength"})
    public void setProximityLedPulseLength(String pulseLengthString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setProximityLedPulseLength");
            AndyMarkColorSensor.ProximityPulseLength pulseLength = (AndyMarkColorSensor.ProximityPulseLength) checkArg(pulseLengthString, AndyMarkColorSensor.ProximityPulseLength.class, "pulseLength");
            if (pulseLength != null) {
                this.andyMarkColorSensor.setProximityLedPulseLength(pulseLength);
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
    @Block(classes = {AndyMarkColorSensor.class}, methodName = {"configureProximitySettings"})
    public void configureProximitySettings(String gainString, int pulses, String pulseLengthString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".configureProximitySettings");
            AndyMarkColorSensor.ProximityGain gain = (AndyMarkColorSensor.ProximityGain) checkArg(gainString, AndyMarkColorSensor.ProximityGain.class, "gain");
            AndyMarkColorSensor.ProximityPulseLength pulseLength = (AndyMarkColorSensor.ProximityPulseLength) checkArg(pulseLengthString, AndyMarkColorSensor.ProximityPulseLength.class, "pulseLength");
            if (gain != null && pulseLength != null) {
                this.andyMarkColorSensor.configureProximitySettings(gain, pulses, pulseLength);
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
}
