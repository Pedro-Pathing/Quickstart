package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/* JADX INFO: loaded from: classes8.dex */
class NavigationAccess extends Access {
    private final HardwareMap hardwareMap;

    NavigationAccess(BlocksOpMode blocksOpMode, String identifier, HardwareMap hardwareMap) {
        super(blocksOpMode, identifier, "");
        this.hardwareMap = hardwareMap;
    }

    @JavascriptInterface
    @Block(classes = {AngleUnit.class}, methodName = {"normalize"})
    public double angleUnit_normalize(double angle, String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, "AngleUnit", ".normalize");
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (angleUnit != null) {
                return angleUnit.normalize(angle);
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
    @Block(classes = {AngleUnit.class}, methodName = {"fromUnit"})
    public double angleUnit_convert(double angle, String fromAngleUnitString, String toAngleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, "AngleUnit", ".convert");
            AngleUnit fromAngleUnit = (AngleUnit) checkArg(fromAngleUnitString, AngleUnit.class, "from");
            AngleUnit toAngleUnit = (AngleUnit) checkArg(toAngleUnitString, AngleUnit.class, "to");
            if (fromAngleUnit != null && toAngleUnit != null) {
                return toAngleUnit.fromUnit(fromAngleUnit, angle);
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
    @Block(classes = {UnnormalizedAngleUnit.class}, methodName = {"fromUnit"})
    public double unnormalizedAngleUnit_convert(double angle, String fromAngleUnitString, String toAngleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, "UnnormalizedAngleUnit", ".convert");
            AngleUnit fromAngleUnit = (AngleUnit) checkArg(fromAngleUnitString, AngleUnit.class, "from");
            AngleUnit toAngleUnit = (AngleUnit) checkArg(toAngleUnitString, AngleUnit.class, "to");
            if (fromAngleUnit != null && toAngleUnit != null) {
                return toAngleUnit.getUnnormalized().fromUnit(fromAngleUnit.getUnnormalized(), angle);
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

    private BuiltinCameraDirection checkBuiltinCameraDirection(String cameraDirectionString) {
        return (BuiltinCameraDirection) checkArg(cameraDirectionString, BuiltinCameraDirection.class, "cameraDirection");
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public BuiltinCameraDirection getBuiltinCameraDirection(String cameraDirectionString) {
        return checkBuiltinCameraDirection(cameraDirectionString);
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public WebcamName getWebcamName(String webcamNameString) {
        return (WebcamName) this.hardwareMap.tryGet(WebcamName.class, webcamNameString);
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public CameraName createSwitchableCameraNameForAllWebcams() {
        return ClassFactory.createSwitchableCameraNameForAllWebcams(this.hardwareMap);
    }
}
