package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.lynx.LynxServoController;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;

/* JADX INFO: loaded from: classes8.dex */
class MagneticFluxAccess extends Access {
    MagneticFluxAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "MagneticFlux");
    }

    private MagneticFlux checkMagneticFlux(Object magneticFluxArg) {
        return (MagneticFlux) checkArg(magneticFluxArg, MagneticFlux.class, "magneticFlux");
    }

    @JavascriptInterface
    @Block(classes = {MagneticFlux.class}, fieldName = {"x"})
    public double getX(Object magneticFluxArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".X");
            MagneticFlux magneticFlux = checkMagneticFlux(magneticFluxArg);
            if (magneticFlux != null) {
                return magneticFlux.x;
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
    @Block(classes = {MagneticFlux.class}, fieldName = {"y"})
    public double getY(Object magneticFluxArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".Y");
            MagneticFlux magneticFlux = checkMagneticFlux(magneticFluxArg);
            if (magneticFlux != null) {
                return magneticFlux.y;
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
    @Block(classes = {MagneticFlux.class}, fieldName = {"z"})
    public double getZ(Object magneticFluxArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".Z");
            MagneticFlux magneticFlux = checkMagneticFlux(magneticFluxArg);
            if (magneticFlux != null) {
                return magneticFlux.z;
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
    @Block(classes = {MagneticFlux.class}, fieldName = {"acquisitionTime"})
    public long getAcquisitionTime(Object magneticFluxArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".AcquisitionTime");
            MagneticFlux magneticFlux = checkMagneticFlux(magneticFluxArg);
            if (magneticFlux != null) {
                return magneticFlux.acquisitionTime;
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
    @Block(classes = {MagneticFlux.class}, constructor = true)
    public MagneticFlux create() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new MagneticFlux();
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
    @Block(classes = {MagneticFlux.class}, constructor = true)
    public MagneticFlux create_withArgs(double x, double y, double z, long acquisitionTime) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new MagneticFlux(x, y, z, acquisitionTime);
        } finally {
        }
    }

    @JavascriptInterface
    @Block(classes = {MagneticFlux.class}, methodName = {"toString"})
    public String toText(Object magneticFluxArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            MagneticFlux magneticFlux = checkMagneticFlux(magneticFluxArg);
            if (magneticFlux != null) {
                return magneticFlux.toString();
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
