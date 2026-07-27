package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.lynx.LynxServoController;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/* JADX INFO: loaded from: classes8.dex */
class YawPitchRollAnglesAccess extends Access {
    YawPitchRollAnglesAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "YawPitchRollAngles");
    }

    @JavascriptInterface
    @Block(classes = {YawPitchRollAngles.class}, methodName = {"getYaw"})
    public double getYaw(Object yawPitchRollAnglesArg, String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getYaw");
            YawPitchRollAngles yawPitchRollAngles = checkYawPitchRollAngles(yawPitchRollAnglesArg);
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (yawPitchRollAngles != null && angleUnit != null) {
                return yawPitchRollAngles.getYaw(angleUnit);
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
    @Block(classes = {YawPitchRollAngles.class}, methodName = {"getPitch"})
    public double getPitch(Object yawPitchRollAnglesArg, String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getPitch");
            YawPitchRollAngles yawPitchRollAngles = checkYawPitchRollAngles(yawPitchRollAnglesArg);
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (yawPitchRollAngles != null && angleUnit != null) {
                return yawPitchRollAngles.getPitch(angleUnit);
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
    @Block(classes = {YawPitchRollAngles.class}, methodName = {"getRoll"})
    public double getRoll(Object yawPitchRollAnglesArg, String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getRoll");
            YawPitchRollAngles yawPitchRollAngles = checkYawPitchRollAngles(yawPitchRollAnglesArg);
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (yawPitchRollAngles != null && angleUnit != null) {
                return yawPitchRollAngles.getRoll(angleUnit);
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
    @Block(classes = {YawPitchRollAngles.class}, constructor = true)
    public YawPitchRollAngles create_withArgs(String angleUnitString, double yaw, double pitch, double roll, long acquisitionTime) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (angleUnit != null) {
                return new YawPitchRollAngles(angleUnit, yaw, pitch, roll, acquisitionTime);
            }
            endBlockExecution();
            return null;
        } finally {
        }
    }

    @JavascriptInterface
    @Block(classes = {YawPitchRollAngles.class}, constructor = true)
    public YawPitchRollAngles create_withArgs2(String angleUnitString, double yaw, double pitch, double roll) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (angleUnit != null) {
                return new YawPitchRollAngles(angleUnit, yaw, pitch, roll, 0L);
            }
            endBlockExecution();
            return null;
        } finally {
        }
    }

    @JavascriptInterface
    @Block(classes = {YawPitchRollAngles.class}, methodName = {"getAcquisitionTime"})
    public long getAcquisitionTime(Object yawPitchRollAnglesArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".AcquisitionTime");
            YawPitchRollAngles yawPitchRollAngles = checkYawPitchRollAngles(yawPitchRollAnglesArg);
            if (yawPitchRollAngles != null) {
                return yawPitchRollAngles.getAcquisitionTime();
            }
            endBlockExecution();
            return 0L;
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
    @Block(classes = {YawPitchRollAngles.class}, methodName = {"toString"})
    public String toText(Object yawPitchRollAnglesArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            YawPitchRollAngles yawPitchRollAngles = checkYawPitchRollAngles(yawPitchRollAnglesArg);
            if (yawPitchRollAngles != null) {
                return yawPitchRollAngles.toString();
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
}
