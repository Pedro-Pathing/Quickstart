package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/* JADX INFO: loaded from: classes8.dex */
class AngularVelocityAccess extends Access {
    AngularVelocityAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "AngularVelocity");
    }

    private AngularVelocity checkAngularVelocity(Object angularVelocityArg) {
        return (AngularVelocity) checkArg(angularVelocityArg, AngularVelocity.class, "angularVelocity");
    }

    @JavascriptInterface
    @Block(classes = {AngularVelocity.class}, fieldName = {"angleUnit"})
    public String getAngleUnit(Object angularVelocityArg) {
        UnnormalizedAngleUnit angleUnit;
        try {
            startBlockExecution(BlockType.GETTER, ".AngleUnit");
            AngularVelocity angularVelocity = checkAngularVelocity(angularVelocityArg);
            if (angularVelocity != null && (angleUnit = angularVelocity.angleUnit) != null) {
                return angleUnit.toString();
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
    @Block(classes = {AngularVelocity.class}, fieldName = {"xRotationRate"})
    public float getXRotationRate(Object angularVelocityArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".XRotationRate");
            AngularVelocity angularVelocity = checkAngularVelocity(angularVelocityArg);
            if (angularVelocity != null) {
                return angularVelocity.xRotationRate;
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {AngularVelocity.class}, fieldName = {"yRotationRate"})
    public float getYRotationRate(Object angularVelocityArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".YRotationRate");
            AngularVelocity angularVelocity = checkAngularVelocity(angularVelocityArg);
            if (angularVelocity != null) {
                return angularVelocity.yRotationRate;
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {AngularVelocity.class}, fieldName = {"zRotationRate"})
    public float getZRotationRate(Object angularVelocityArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".ZRotationRate");
            AngularVelocity angularVelocity = checkAngularVelocity(angularVelocityArg);
            if (angularVelocity != null) {
                return angularVelocity.zRotationRate;
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {AngularVelocity.class}, fieldName = {"acquisitionTime"})
    public long getAcquisitionTime(Object angularVelocityArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".AcquisitionTime");
            AngularVelocity angularVelocity = checkAngularVelocity(angularVelocityArg);
            if (angularVelocity != null) {
                return angularVelocity.acquisitionTime;
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
    @Block(classes = {AngularVelocity.class}, constructor = true)
    public AngularVelocity create() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new AngularVelocity();
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
    @Block(classes = {AngularVelocity.class}, constructor = true)
    public AngularVelocity create_withArgs(String angleUnitString, float xRotationRate, float yRotationRate, float zRotationRate, long acquisitionTime) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (angleUnit != null) {
                return new AngularVelocity(angleUnit, xRotationRate, yRotationRate, zRotationRate, acquisitionTime);
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
    @Block(classes = {AngularVelocity.class}, methodName = {"toAngleUnit"})
    public AngularVelocity toAngleUnit(Object angularVelocityArg, String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toAngleUnit");
            AngularVelocity angularVelocity = checkAngularVelocity(angularVelocityArg);
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (angularVelocity != null && angleUnit != null) {
                return angularVelocity.toAngleUnit(angleUnit);
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
    @Block(classes = {AngularVelocity.class}, fieldName = {"xRotationRate", "yRotationRate", "zRotationRate"})
    public float getRotationRate(Object angularVelocityArg, String axisString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getRotationRate");
            AngularVelocity angularVelocity = checkAngularVelocity(angularVelocityArg);
            Axis axis = (Axis) checkArg(axisString, Axis.class, "axis");
            if (angularVelocity != null && axis != null) {
                switch (axis) {
                    case X:
                        return angularVelocity.xRotationRate;
                    case Y:
                        return angularVelocity.yRotationRate;
                    case Z:
                        return angularVelocity.zRotationRate;
                    case UNKNOWN:
                        reportInvalidArg("axis", "Axis.X, Axis.Y, or Axis.Z");
                        break;
                }
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {AngularVelocity.class}, methodName = {"toString"})
    public String toText(Object angularVelocityArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            AngularVelocity angularVelocity = checkAngularVelocity(angularVelocityArg);
            if (angularVelocity != null) {
                return angularVelocity.toString();
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
