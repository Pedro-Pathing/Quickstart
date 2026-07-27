package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

/* JADX INFO: loaded from: classes8.dex */
class Rev9AxisImuOrientationOnRobotAccess extends Access {
    Rev9AxisImuOrientationOnRobotAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "Rev9AxisImuOrientationOnRobot");
    }

    @JavascriptInterface
    @Block(classes = {Rev9AxisImuOrientationOnRobot.class}, constructor = true)
    public Rev9AxisImuOrientationOnRobot create1(String logoFacingDirectionString, String i2cPortFacingDirectionString) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            Rev9AxisImuOrientationOnRobot.LogoFacingDirection logoFacingDirection = (Rev9AxisImuOrientationOnRobot.LogoFacingDirection) checkArg(logoFacingDirectionString, Rev9AxisImuOrientationOnRobot.LogoFacingDirection.class, "logoFacingDirection");
            Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection i2cPortFacingDirection = (Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection) checkArg(i2cPortFacingDirectionString, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.class, "i2cPortFacingDirection");
            if (logoFacingDirection != null && i2cPortFacingDirection != null) {
                return new Rev9AxisImuOrientationOnRobot(logoFacingDirection, i2cPortFacingDirection);
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
    @Block(classes = {Rev9AxisImuOrientationOnRobot.class}, constructor = true)
    public Rev9AxisImuOrientationOnRobot create2(Object orientationArg) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            Orientation orientation = checkOrientation(orientationArg, "rotation");
            if (orientation != null) {
                return new Rev9AxisImuOrientationOnRobot(orientation);
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
    @Block(classes = {Rev9AxisImuOrientationOnRobot.class}, constructor = true)
    public Rev9AxisImuOrientationOnRobot create3(Object quaternionArg) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            Quaternion quaternion = checkQuaternion(quaternionArg, "rotation");
            if (quaternion != null) {
                return new Rev9AxisImuOrientationOnRobot(quaternion);
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
    @Block(classes = {Rev9AxisImuOrientationOnRobot.class}, methodName = {"zyxOrientation"})
    public Orientation zyxOrientation(double z, double y, double x) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".zyxOrientation");
            return Rev9AxisImuOrientationOnRobot.zyxOrientation(z, y, x);
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
    @Block(classes = {Rev9AxisImuOrientationOnRobot.class}, methodName = {"xyzOrientation"})
    public Orientation xyzOrientation(double x, double y, double z) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".zyxOrientation");
            return Rev9AxisImuOrientationOnRobot.xyzOrientation(x, y, z);
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
