package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* JADX INFO: loaded from: classes8.dex */
class AndroidAccelerometerAccess extends Access {
    private final AndroidAccelerometer androidAccelerometer;

    AndroidAccelerometerAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "AndroidAccelerometer");
        this.androidAccelerometer = new AndroidAccelerometer();
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.Access
    void close() {
        this.androidAccelerometer.stopListening();
    }

    @JavascriptInterface
    public void setDistanceUnit(String distanceUnitString) {
        try {
            startBlockExecution(BlockType.SETTER, ".DistanceUnit");
            DistanceUnit distanceUnit = (DistanceUnit) checkArg(distanceUnitString, DistanceUnit.class, "");
            if (distanceUnit != null) {
                this.androidAccelerometer.setDistanceUnit(distanceUnit);
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
    public double getX() {
        try {
            startBlockExecution(BlockType.GETTER, ".X");
            return this.androidAccelerometer.getX();
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
    public double getY() {
        try {
            startBlockExecution(BlockType.GETTER, ".Y");
            return this.androidAccelerometer.getY();
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
    public double getZ() {
        try {
            startBlockExecution(BlockType.GETTER, ".Z");
            return this.androidAccelerometer.getZ();
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
    public Acceleration getAcceleration() {
        try {
            startBlockExecution(BlockType.GETTER, ".Acceleration");
            return this.androidAccelerometer.getAcceleration();
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
    public String getDistanceUnit() {
        try {
            startBlockExecution(BlockType.GETTER, ".DistanceUnit");
            return this.androidAccelerometer.getDistanceUnit().toString();
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
    public boolean isAvailable() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isAvailable");
            return this.androidAccelerometer.isAvailable();
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
    public void startListening() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".startListening");
            this.androidAccelerometer.startListening();
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
    public void stopListening() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".stopListening");
            this.androidAccelerometer.stopListening();
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
