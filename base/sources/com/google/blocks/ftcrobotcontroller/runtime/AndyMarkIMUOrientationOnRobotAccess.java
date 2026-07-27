package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.andymark.AndyMarkIMUOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

/* JADX INFO: loaded from: classes8.dex */
class AndyMarkIMUOrientationOnRobotAccess extends Access {
    AndyMarkIMUOrientationOnRobotAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "AndyMarkIMUOrientationOnRobot");
    }

    @JavascriptInterface
    @Block(classes = {AndyMarkIMUOrientationOnRobot.class}, constructor = true)
    public AndyMarkIMUOrientationOnRobot create1(String logoFacingDirectionString, String i2cPortFacingDirectionString) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            AndyMarkIMUOrientationOnRobot.LogoFacingDirection logoFacingDirection = (AndyMarkIMUOrientationOnRobot.LogoFacingDirection) checkArg(logoFacingDirectionString, AndyMarkIMUOrientationOnRobot.LogoFacingDirection.class, "logoFacingDirection");
            AndyMarkIMUOrientationOnRobot.I2cPortFacingDirection i2cPortFacingDirection = (AndyMarkIMUOrientationOnRobot.I2cPortFacingDirection) checkArg(i2cPortFacingDirectionString, AndyMarkIMUOrientationOnRobot.I2cPortFacingDirection.class, "i2cPortFacingDirection");
            if (logoFacingDirection != null && i2cPortFacingDirection != null) {
                return new AndyMarkIMUOrientationOnRobot(logoFacingDirection, i2cPortFacingDirection);
            }
            endBlockExecution();
            return null;
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
    @Block(classes = {AndyMarkIMUOrientationOnRobot.class}, constructor = true)
    public AndyMarkIMUOrientationOnRobot create2(Object orientationArg) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            Orientation orientation = checkOrientation(orientationArg, "rotation");
            if (orientation != null) {
                return new AndyMarkIMUOrientationOnRobot(orientation);
            }
            endBlockExecution();
            return null;
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
    @Block(classes = {AndyMarkIMUOrientationOnRobot.class}, constructor = true)
    public AndyMarkIMUOrientationOnRobot create3(Object quaternionArg) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            Quaternion quaternion = checkQuaternion(quaternionArg, "rotation");
            if (quaternion != null) {
                return new AndyMarkIMUOrientationOnRobot(quaternion);
            }
            endBlockExecution();
            return null;
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
    @Block(classes = {AndyMarkIMUOrientationOnRobot.class}, methodName = {"zyxOrientation"})
    public Orientation zyxOrientation(double z, double y, double x) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".zyxOrientation");
            return AndyMarkIMUOrientationOnRobot.zyxOrientation(z, y, x);
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
    @Block(classes = {AndyMarkIMUOrientationOnRobot.class}, methodName = {"xyzOrientation"})
    public Orientation xyzOrientation(double x, double y, double z) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".zyxOrientation");
            return AndyMarkIMUOrientationOnRobot.xyzOrientation(x, y, z);
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
