package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.lynx.LynxServoController;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/* JADX INFO: loaded from: classes8.dex */
class Pose2DAccess extends Access {
    Pose2DAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "Pose2D");
    }

    @JavascriptInterface
    @Block(classes = {Pose2D.class}, constructor = true)
    public Pose2D create(String distanceUnitString, double x, double y, String angleUnitString, double heading) {
        DistanceUnit distanceUnit;
        try {
            startBlockExecution(BlockType.CREATE, "");
            distanceUnit = checkDistanceUnit(distanceUnitString);
        } catch (Throwable th) {
            e = th;
        }
        try {
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (distanceUnit != null && angleUnit != null) {
                return new Pose2D(distanceUnit, x, y, angleUnit, heading);
            }
            endBlockExecution();
            return null;
        } catch (Throwable th2) {
            e = th2;
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {Pose2D.class}, methodName = {"getX"})
    public double getX(Object pose2DArg, String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getX");
            Pose2D pose2D = checkPose2D(pose2DArg);
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (pose2D != null && distanceUnit != null) {
                return pose2D.getX(distanceUnit);
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
    @Block(classes = {Pose2D.class}, methodName = {"getY"})
    public double getY(Object pose2DArg, String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getY");
            Pose2D pose2D = checkPose2D(pose2DArg);
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (pose2D != null && distanceUnit != null) {
                return pose2D.getY(distanceUnit);
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
    @Block(classes = {Pose2D.class}, methodName = {"getHeading"})
    public double getHeading(Object pose2DArg, String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getHeading");
            Pose2D pose2D = checkPose2D(pose2DArg);
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (pose2D != null && angleUnit != null) {
                return pose2D.getHeading(angleUnit);
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
    @Block(classes = {Pose2D.class}, methodName = {"toString"})
    public String toText(Object pose2DArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            Pose2D pose2D = checkPose2D(pose2DArg);
            if (pose2D != null) {
                return pose2D.toString();
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
