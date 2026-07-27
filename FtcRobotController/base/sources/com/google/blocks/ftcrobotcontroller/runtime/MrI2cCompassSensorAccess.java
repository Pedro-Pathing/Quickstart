package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;

/* JADX INFO: loaded from: classes8.dex */
class MrI2cCompassSensorAccess extends HardwareAccess<ModernRoboticsI2cCompassSensor> {
    private final ModernRoboticsI2cCompassSensor mrI2cCompassSensor;

    MrI2cCompassSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, ModernRoboticsI2cCompassSensor.class);
        this.mrI2cCompassSensor = (ModernRoboticsI2cCompassSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"getDirection"})
    public double getDirection() {
        try {
            startBlockExecution(BlockType.GETTER, ".Direction");
            return this.mrI2cCompassSensor.getDirection();
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
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress7Bit(int i2cAddr7Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress7Bit");
            this.mrI2cCompassSensor.setI2cAddress(I2cAddr.create7bit(i2cAddr7Bit));
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
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress7Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress7Bit");
            I2cAddr i2cAddr = this.mrI2cCompassSensor.getI2cAddress();
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
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress8Bit(int i2cAddr8Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress8Bit");
            this.mrI2cCompassSensor.setI2cAddress(I2cAddr.create8bit(i2cAddr8Bit));
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
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress8Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress8Bit");
            I2cAddr i2cAddr = this.mrI2cCompassSensor.getI2cAddress();
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
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"getAcceleration"})
    public double getXAccel() {
        try {
            startBlockExecution(BlockType.GETTER, ".XAccel");
            Acceleration acceleration = this.mrI2cCompassSensor.getAcceleration();
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
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"getAcceleration"})
    public double getYAccel() {
        try {
            startBlockExecution(BlockType.GETTER, ".YAccel");
            Acceleration acceleration = this.mrI2cCompassSensor.getAcceleration();
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
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"getAcceleration"})
    public double getZAccel() {
        try {
            startBlockExecution(BlockType.GETTER, ".ZAccel");
            Acceleration acceleration = this.mrI2cCompassSensor.getAcceleration();
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
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"getMagneticFlux"})
    public double getXMagneticFlux() {
        try {
            startBlockExecution(BlockType.GETTER, ".XMagneticFlux");
            MagneticFlux magneticFlux = this.mrI2cCompassSensor.getMagneticFlux();
            if (magneticFlux != null) {
                return magneticFlux.x;
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
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"getMagneticFlux"})
    public double getYMagneticFlux() {
        try {
            startBlockExecution(BlockType.GETTER, ".YMagneticFlux");
            MagneticFlux magneticFlux = this.mrI2cCompassSensor.getMagneticFlux();
            if (magneticFlux != null) {
                return magneticFlux.y;
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
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"getMagneticFlux"})
    public double getZMagneticFlux() {
        try {
            startBlockExecution(BlockType.GETTER, ".ZMagneticFlux");
            MagneticFlux magneticFlux = this.mrI2cCompassSensor.getMagneticFlux();
            if (magneticFlux != null) {
                return magneticFlux.z;
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
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"setMode"})
    public void setMode(String compassModeString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setMode");
            CompassSensor.CompassMode compassMode = (CompassSensor.CompassMode) checkArg(compassModeString, CompassSensor.CompassMode.class, "compassMode");
            if (compassMode != null) {
                this.mrI2cCompassSensor.setMode(compassMode);
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
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"isCalibrating"})
    public boolean isCalibrating() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isCalibrating");
            return this.mrI2cCompassSensor.isCalibrating();
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
    @Block(classes = {ModernRoboticsI2cCompassSensor.class}, methodName = {"calibrationFailed"})
    public boolean calibrationFailed() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".calibrationFailed");
            return this.mrI2cCompassSensor.calibrationFailed();
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
