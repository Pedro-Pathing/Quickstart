package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.lynx.LynxServoController;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* JADX INFO: loaded from: classes8.dex */
class AccelerationAccess extends Access {
    AccelerationAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "Acceleration");
    }

    private Acceleration checkAcceleration(Object accelerationArg) {
        return (Acceleration) checkArg(accelerationArg, Acceleration.class, "acceleration");
    }

    @JavascriptInterface
    @Block(classes = {Acceleration.class}, fieldName = {"unit"})
    public String getDistanceUnit(Object accelerationArg) {
        DistanceUnit distanceUnit;
        try {
            startBlockExecution(BlockType.GETTER, ".DistanceUnit");
            Acceleration acceleration = checkAcceleration(accelerationArg);
            if (acceleration != null && (distanceUnit = acceleration.unit) != null) {
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
    @Block(classes = {Acceleration.class}, fieldName = {"xAccel"})
    public double getXAccel(Object accelerationArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".XAccel");
            Acceleration acceleration = checkAcceleration(accelerationArg);
            if (acceleration != null) {
                return acceleration.xAccel;
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
    @Block(classes = {Acceleration.class}, fieldName = {"yAccel"})
    public double getYAccel(Object accelerationArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".YAccel");
            Acceleration acceleration = checkAcceleration(accelerationArg);
            if (acceleration != null) {
                return acceleration.yAccel;
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
    @Block(classes = {Acceleration.class}, fieldName = {"zAccel"})
    public double getZAccel(Object accelerationArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".ZAccel");
            Acceleration acceleration = checkAcceleration(accelerationArg);
            if (acceleration != null) {
                return acceleration.zAccel;
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
    @Block(classes = {Acceleration.class}, fieldName = {"acquisitionTime"})
    public long getAcquisitionTime(Object accelerationArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".AcquisitionTime");
            Acceleration acceleration = checkAcceleration(accelerationArg);
            if (acceleration != null) {
                return acceleration.acquisitionTime;
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
    @Block(classes = {Acceleration.class}, constructor = true)
    public Acceleration create() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new Acceleration();
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
    @Block(classes = {Acceleration.class}, constructor = true)
    public Acceleration create_withArgs(String distanceUnitString, double xAccel, double yAccel, double zAccel, long acquisitionTime) {
        try {
            startBlockExecution(BlockType.CREATE, "");
        } catch (Throwable th) {
            e = th;
        }
        try {
            DistanceUnit distanceUnit = (DistanceUnit) checkArg(distanceUnitString, DistanceUnit.class, "distanceUnit");
            if (distanceUnit != null) {
                return new Acceleration(distanceUnit, xAccel, yAccel, zAccel, acquisitionTime);
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
    @Block(classes = {Acceleration.class}, methodName = {"fromGravity"})
    public Acceleration fromGravity(double gx, double gy, double gz, long acquisitionTime) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".fromGravity");
            return Acceleration.fromGravity(gx, gy, gz, acquisitionTime);
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
    @Block(classes = {Acceleration.class}, methodName = {"toUnit"})
    public Acceleration toDistanceUnit(Object accelerationArg, String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toDistanceUnit");
            Acceleration acceleration = checkAcceleration(accelerationArg);
            DistanceUnit distanceUnit = (DistanceUnit) checkArg(distanceUnitString, DistanceUnit.class, "distanceUnit");
            if (acceleration != null && distanceUnit != null) {
                return acceleration.toUnit(distanceUnit);
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
    @Block(classes = {Acceleration.class}, methodName = {"toString"})
    public String toText(Object accelerationArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            Acceleration acceleration = checkAcceleration(accelerationArg);
            if (acceleration != null) {
                return acceleration.toString();
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
