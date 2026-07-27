package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cIrSeekerSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

/* JADX INFO: loaded from: classes8.dex */
class IrSeekerSensorAccess extends HardwareAccess<IrSeekerSensor> {
    private final IrSeekerSensor irSeekerSensor;

    IrSeekerSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, IrSeekerSensor.class);
        this.irSeekerSensor = (IrSeekerSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = {"setSignalDetectedThreshold"})
    public void setSignalDetectedThreshold(double threshold) {
        try {
            startBlockExecution(BlockType.SETTER, ".SignalDetectedThreshold");
            this.irSeekerSensor.setSignalDetectedThreshold(threshold);
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
    @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = {"getSignalDetectedThreshold"})
    public double getSignalDetectedThreshold() {
        try {
            startBlockExecution(BlockType.GETTER, ".SignalDetectedThreshold");
            return this.irSeekerSensor.getSignalDetectedThreshold();
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
    @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = {"setMode"})
    public void setMode(String modeString) {
        try {
            startBlockExecution(BlockType.SETTER, ".Mode");
            IrSeekerSensor.Mode mode = (IrSeekerSensor.Mode) checkArg(modeString, IrSeekerSensor.Mode.class, "");
            if (mode != null) {
                this.irSeekerSensor.setMode(mode);
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
    @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = {"getMode"})
    public String getMode() {
        try {
            startBlockExecution(BlockType.GETTER, ".Mode");
            IrSeekerSensor.Mode mode = this.irSeekerSensor.getMode();
            if (mode != null) {
                return mode.toString();
            }
            return "";
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
    @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = {"signalDetected"})
    public boolean getIsSignalDetected() {
        try {
            startBlockExecution(BlockType.GETTER, ".IsSignalDetected");
            return this.irSeekerSensor.signalDetected();
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
    @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = {"getAngle"})
    public double getAngle() {
        try {
            startBlockExecution(BlockType.GETTER, ".Angle");
            return this.irSeekerSensor.getAngle();
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
    @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = {"getStrength"})
    public double getStrength() {
        try {
            startBlockExecution(BlockType.GETTER, ".Strength");
            return this.irSeekerSensor.getStrength();
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
    @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress7Bit(int i2cAddr7Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress7Bit");
            this.irSeekerSensor.setI2cAddress(I2cAddr.create7bit(i2cAddr7Bit));
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
    @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress7Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress7Bit");
            I2cAddr i2cAddr = this.irSeekerSensor.getI2cAddress();
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
    @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress8Bit(int i2cAddr8Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress8Bit");
            this.irSeekerSensor.setI2cAddress(I2cAddr.create8bit(i2cAddr8Bit));
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
    @Block(classes = {IrSeekerSensor.class, ModernRoboticsI2cIrSeekerSensorV3.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress8Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress8Bit");
            I2cAddr i2cAddr = this.irSeekerSensor.getI2cAddress();
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
