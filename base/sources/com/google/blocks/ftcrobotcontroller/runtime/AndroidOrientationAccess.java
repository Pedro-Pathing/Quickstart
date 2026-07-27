package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.robotcore.external.android.AndroidOrientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* JADX INFO: loaded from: classes8.dex */
class AndroidOrientationAccess extends Access {
    private final AndroidOrientation androidOrientation;

    AndroidOrientationAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "AndroidOrientation");
        this.androidOrientation = new AndroidOrientation();
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.Access
    void close() {
        this.androidOrientation.stopListening();
    }

    @JavascriptInterface
    public void setAngleUnit(String angleUnitString) {
        try {
            startBlockExecution(BlockType.SETTER, ".AngleUnit");
            AngleUnit angleUnit = (AngleUnit) checkArg(angleUnitString, AngleUnit.class, "");
            if (angleUnit != null) {
                this.androidOrientation.setAngleUnit(angleUnit);
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
    public double getAzimuth() {
        try {
            startBlockExecution(BlockType.GETTER, ".Azimuth");
            return this.androidOrientation.getAzimuth();
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
    public double getPitch() {
        try {
            startBlockExecution(BlockType.GETTER, ".Pitch");
            return this.androidOrientation.getPitch();
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
    public double getRoll() {
        try {
            startBlockExecution(BlockType.GETTER, ".Roll");
            return this.androidOrientation.getRoll();
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
    public double getAngle() {
        try {
            startBlockExecution(BlockType.GETTER, ".Angle");
            return this.androidOrientation.getAngle();
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
    public double getMagnitude() {
        try {
            startBlockExecution(BlockType.GETTER, ".Magnitude");
            return this.androidOrientation.getMagnitude();
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
    public String getAngleUnit() {
        try {
            startBlockExecution(BlockType.GETTER, ".AngleUnit");
            return this.androidOrientation.getAngleUnit().toString();
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
            return this.androidOrientation.isAvailable();
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
            this.androidOrientation.startListening();
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
            this.androidOrientation.stopListening();
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
