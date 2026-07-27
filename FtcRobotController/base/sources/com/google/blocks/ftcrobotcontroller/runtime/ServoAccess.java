package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/* JADX INFO: loaded from: classes8.dex */
class ServoAccess extends HardwareAccess<ServoImplEx> {
    private final ServoImplEx servo;

    ServoAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, ServoImplEx.class);
        this.servo = (ServoImplEx) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {Servo.class}, methodName = {"setDirection"})
    public void setDirection(String directionString) {
        try {
            startBlockExecution(BlockType.SETTER, ".Direction");
            Servo.Direction direction = (Servo.Direction) checkArg(directionString, Servo.Direction.class, "");
            if (direction != null) {
                this.servo.setDirection(direction);
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
    @Block(classes = {Servo.class}, methodName = {"getDirection"})
    public String getDirection() {
        try {
            startBlockExecution(BlockType.GETTER, ".Direction");
            Servo.Direction direction = this.servo.getDirection();
            if (direction != null) {
                return direction.toString();
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
    @Block(classes = {Servo.class}, methodName = {"setPosition"})
    public void setPosition(double position) {
        try {
            startBlockExecution(BlockType.SETTER, ".Position");
            this.servo.setPosition(position);
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
    @Block(classes = {Servo.class}, methodName = {"getPosition"})
    public double getPosition() {
        try {
            startBlockExecution(BlockType.GETTER, ".Position");
            return this.servo.getPosition();
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
    @Block(classes = {Servo.class}, methodName = {"scaleRange"})
    public void scaleRange(double min, double max) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".scaleRange");
            this.servo.scaleRange(min, max);
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
    @Block(classes = {PwmControl.class}, methodName = {"setPwmEnable"})
    public void setPwmEnable() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setPwmEnable");
            this.servo.setPwmEnable();
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
    @Block(classes = {PwmControl.class}, methodName = {"setPwmDisable"})
    public void setPwmDisable() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setPwmDisable");
            this.servo.setPwmDisable();
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
    @Block(classes = {PwmControl.class}, methodName = {"isPwmEnabled"})
    public boolean isPwmEnabled() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isPwmEnabled");
            return this.servo.isPwmEnabled();
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
