package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

/* JADX INFO: loaded from: classes8.dex */
class AndroidGyroscopeAccess extends Access {
    private final AndroidGyroscope androidGyroscope;

    AndroidGyroscopeAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "AndroidGyroscope");
        this.androidGyroscope = new AndroidGyroscope();
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.Access
    void close() {
        this.androidGyroscope.stopListening();
    }

    @JavascriptInterface
    public void setAngleUnit(String angleUnitString) {
        try {
            startBlockExecution(BlockType.SETTER, ".AngleUnit");
            AngleUnit angleUnit = (AngleUnit) checkArg(angleUnitString, AngleUnit.class, "");
            if (angleUnit != null) {
                this.androidGyroscope.setAngleUnit(angleUnit);
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
    public float getX() {
        try {
            startBlockExecution(BlockType.GETTER, ".X");
            return this.androidGyroscope.getX();
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
    public float getY() {
        try {
            startBlockExecution(BlockType.GETTER, ".Y");
            return this.androidGyroscope.getY();
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
    public float getZ() {
        try {
            startBlockExecution(BlockType.GETTER, ".Z");
            return this.androidGyroscope.getZ();
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
    public AngularVelocity getAngularVelocity() {
        try {
            startBlockExecution(BlockType.GETTER, ".AngularVelocity");
            return this.androidGyroscope.getAngularVelocity();
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
            return this.androidGyroscope.getAngleUnit().toString();
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
            return this.androidGyroscope.isAvailable();
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
            this.androidGyroscope.startListening();
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
            this.androidGyroscope.stopListening();
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
