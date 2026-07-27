package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

/* JADX INFO: loaded from: classes8.dex */
class RevHubOrientationOnRobotAccess extends Access {
    RevHubOrientationOnRobotAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "RevHubOrientationOnRobot");
    }

    @JavascriptInterface
    @Block(classes = {RevHubOrientationOnRobot.class}, constructor = true)
    public RevHubOrientationOnRobot create1(String logoFacingDirectionString, String usbFacingDirectionString) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = (RevHubOrientationOnRobot.LogoFacingDirection) checkArg(logoFacingDirectionString, RevHubOrientationOnRobot.LogoFacingDirection.class, "logoFacingDirection");
            RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = (RevHubOrientationOnRobot.UsbFacingDirection) checkArg(usbFacingDirectionString, RevHubOrientationOnRobot.UsbFacingDirection.class, "usbFacingDirection");
            if (logoFacingDirection != null && usbFacingDirection != null) {
                return new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);
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
    @Block(classes = {RevHubOrientationOnRobot.class}, constructor = true)
    public RevHubOrientationOnRobot create2(Object orientationArg) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            Orientation orientation = checkOrientation(orientationArg, "rotation");
            if (orientation != null) {
                return new RevHubOrientationOnRobot(orientation);
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
    @Block(classes = {RevHubOrientationOnRobot.class}, constructor = true)
    public RevHubOrientationOnRobot create3(Object quaternionArg) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            Quaternion quaternion = checkQuaternion(quaternionArg, "rotation");
            if (quaternion != null) {
                return new RevHubOrientationOnRobot(quaternion);
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
    @Block(classes = {RevHubOrientationOnRobot.class}, methodName = {"zyxOrientation"})
    public Orientation zyxOrientation(double z, double y, double x) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".zyxOrientation");
            return RevHubOrientationOnRobot.zyxOrientation(z, y, x);
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
    @Block(classes = {RevHubOrientationOnRobot.class}, methodName = {"xyzOrientation"})
    public Orientation xyzOrientation(double x, double y, double z) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".zyxOrientation");
            return RevHubOrientationOnRobot.xyzOrientation(x, y, z);
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
