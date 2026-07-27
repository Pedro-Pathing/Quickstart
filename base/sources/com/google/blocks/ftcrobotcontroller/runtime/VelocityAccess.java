package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.lynx.LynxServoController;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/* JADX INFO: loaded from: classes8.dex */
class VelocityAccess extends Access {
    VelocityAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "Velocity");
    }

    private Velocity checkVelocity(Object velocityArg) {
        return (Velocity) checkArg(velocityArg, Velocity.class, "velocity");
    }

    @JavascriptInterface
    @Block(classes = {Velocity.class}, fieldName = {"unit"})
    public String getDistanceUnit(Object velocityArg) {
        DistanceUnit distanceUnit;
        try {
            startBlockExecution(BlockType.GETTER, ".DistanceUnit");
            Velocity velocity = checkVelocity(velocityArg);
            if (velocity != null && (distanceUnit = velocity.unit) != null) {
                return distanceUnit.toString();
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
    @Block(classes = {Velocity.class}, fieldName = {"xVeloc"})
    public double getXVeloc(Object velocityArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".XVeloc");
            Velocity velocity = checkVelocity(velocityArg);
            if (velocity != null) {
                return velocity.xVeloc;
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
    @Block(classes = {Velocity.class}, fieldName = {"yVeloc"})
    public double getYVeloc(Object velocityArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".YVeloc");
            Velocity velocity = checkVelocity(velocityArg);
            if (velocity != null) {
                return velocity.yVeloc;
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
    @Block(classes = {Velocity.class}, fieldName = {"zVeloc"})
    public double getZVeloc(Object velocityArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".ZVeloc");
            Velocity velocity = checkVelocity(velocityArg);
            if (velocity != null) {
                return velocity.zVeloc;
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
    @Block(classes = {Velocity.class}, fieldName = {"acquisitionTime"})
    public long getAcquisitionTime(Object velocityArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".AcquisitionTime");
            Velocity velocity = checkVelocity(velocityArg);
            if (velocity != null) {
                return velocity.acquisitionTime;
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
    @Block(classes = {Velocity.class}, constructor = true)
    public Velocity create() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new Velocity();
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
    @Block(classes = {Velocity.class}, constructor = true)
    public Velocity create_withArgs(String distanceUnitString, double xVeloc, double yVeloc, double zVeloc, long acquisitionTime) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (distanceUnit != null) {
                return new Velocity(distanceUnit, xVeloc, yVeloc, zVeloc, acquisitionTime);
            }
            endBlockExecution();
            return null;
        } finally {
        }
    }

    @JavascriptInterface
    @Block(classes = {Velocity.class}, methodName = {"toUnit"})
    public Velocity toDistanceUnit(Object velocityArg, String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toDistanceUnit");
            Velocity velocity = checkVelocity(velocityArg);
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (velocity != null && distanceUnit != null) {
                return velocity.toUnit(distanceUnit);
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
    @Block(classes = {Velocity.class}, methodName = {"toString"})
    public String toText(Object velocityArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            Velocity velocity = checkVelocity(velocityArg);
            if (velocity != null) {
                return velocity.toString();
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
