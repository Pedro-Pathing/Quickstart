package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

/* JADX INFO: loaded from: classes8.dex */
class CRServoAccess extends HardwareAccess<CRServoImplEx> {
    private final CRServoImplEx crServo;

    CRServoAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, CRServoImplEx.class);
        this.crServo = (CRServoImplEx) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {CRServo.class}, methodName = {"setDirection"})
    public void setDirection(String directionString) {
        try {
            startBlockExecution(BlockType.SETTER, ".Direction");
            DcMotorSimple.Direction direction = (DcMotorSimple.Direction) checkArg(directionString, DcMotorSimple.Direction.class, "");
            if (direction != null) {
                this.crServo.setDirection(direction);
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
    @Block(classes = {CRServo.class}, methodName = {"getDirection"})
    public String getDirection() {
        try {
            startBlockExecution(BlockType.GETTER, ".Direction");
            DcMotorSimple.Direction direction = this.crServo.getDirection();
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
    @Block(classes = {CRServo.class}, methodName = {"setPower"})
    public void setPower(double power) {
        try {
            startBlockExecution(BlockType.SETTER, ".Power");
            this.crServo.setPower(power);
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
    @Block(classes = {CRServo.class}, methodName = {"getPower"})
    public double getPower() {
        try {
            startBlockExecution(BlockType.GETTER, ".Power");
            return this.crServo.getPower();
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
            this.crServo.setPwmEnable();
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
            this.crServo.setPwmDisable();
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
            return this.crServo.isPwmEnabled();
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
