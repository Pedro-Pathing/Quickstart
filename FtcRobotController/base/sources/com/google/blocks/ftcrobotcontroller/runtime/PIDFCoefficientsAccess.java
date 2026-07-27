package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.onbotjava.RequestConditions;

/* JADX INFO: loaded from: classes8.dex */
class PIDFCoefficientsAccess extends Access {
    PIDFCoefficientsAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "PIDFCoefficients");
    }

    private PIDFCoefficients checkPIDFCoefficients(Object pidfCoefficientsArg) {
        return (PIDFCoefficients) checkArg(pidfCoefficientsArg, PIDFCoefficients.class, "pidfCoefficients");
    }

    @JavascriptInterface
    @Block(classes = {PIDFCoefficients.class}, constructor = true)
    public PIDFCoefficients create() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new PIDFCoefficients();
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
    @Block(classes = {PIDFCoefficients.class}, constructor = true)
    public PIDFCoefficients create_withPIDFAlgorithm(double p, double i, double d, double f, String algorithmString) {
        try {
            startBlockExecution(BlockType.CREATE, "");
        } catch (Throwable th) {
            e = th;
        }
        try {
            MotorControlAlgorithm algorithm = (MotorControlAlgorithm) checkArg(algorithmString, MotorControlAlgorithm.class, "algorithm");
            if (algorithm != null) {
                return new PIDFCoefficients(p, i, d, f, algorithm);
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
    @Block(classes = {PIDFCoefficients.class}, constructor = true)
    public PIDFCoefficients create_withPIDF(double p, double i, double d, double f) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new PIDFCoefficients(p, i, d, f);
        } finally {
        }
    }

    @JavascriptInterface
    @Block(classes = {PIDFCoefficients.class}, constructor = true)
    public PIDFCoefficients create_withPIDFCoefficients(Object pidfCoefficientsArg) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            PIDFCoefficients pidfCoefficients = checkPIDFCoefficients(pidfCoefficientsArg);
            if (pidfCoefficients != null) {
                return new PIDFCoefficients(pidfCoefficients);
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
    @Block(classes = {PIDFCoefficients.class}, fieldName = {"p"})
    public void setP(Object pidfCoefficientsArg, double p) {
        try {
            startBlockExecution(BlockType.SETTER, ".P");
            PIDFCoefficients pidfCoefficients = checkPIDFCoefficients(pidfCoefficientsArg);
            if (pidfCoefficients != null) {
                pidfCoefficients.p = p;
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
    @Block(classes = {PIDFCoefficients.class}, fieldName = {"p"})
    public double getP(Object pidfCoefficientsArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".P");
            PIDFCoefficients pidfCoefficients = checkPIDFCoefficients(pidfCoefficientsArg);
            if (pidfCoefficients != null) {
                return pidfCoefficients.p;
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
    @Block(classes = {PIDFCoefficients.class}, fieldName = {"i"})
    public void setI(Object pidfCoefficientsArg, double i) {
        try {
            startBlockExecution(BlockType.SETTER, ".I");
            PIDFCoefficients pidfCoefficients = checkPIDFCoefficients(pidfCoefficientsArg);
            if (pidfCoefficients != null) {
                pidfCoefficients.i = i;
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
    @Block(classes = {PIDFCoefficients.class}, fieldName = {"i"})
    public double getI(Object pidfCoefficientsArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".I");
            PIDFCoefficients pidfCoefficients = checkPIDFCoefficients(pidfCoefficientsArg);
            if (pidfCoefficients != null) {
                return pidfCoefficients.i;
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
    @Block(classes = {PIDFCoefficients.class}, fieldName = {"d"})
    public void setD(Object pidfCoefficientsArg, double d) {
        try {
            startBlockExecution(BlockType.SETTER, ".D");
            PIDFCoefficients pidfCoefficients = checkPIDFCoefficients(pidfCoefficientsArg);
            if (pidfCoefficients != null) {
                pidfCoefficients.d = d;
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
    @Block(classes = {PIDFCoefficients.class}, fieldName = {"d"})
    public double getD(Object pidfCoefficientsArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".D");
            PIDFCoefficients pidfCoefficients = checkPIDFCoefficients(pidfCoefficientsArg);
            if (pidfCoefficients != null) {
                return pidfCoefficients.d;
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
    @Block(classes = {PIDFCoefficients.class}, fieldName = {RequestConditions.REQUEST_KEY_FILE})
    public void setF(Object pidfCoefficientsArg, double f) {
        try {
            startBlockExecution(BlockType.SETTER, ".F");
            PIDFCoefficients pidfCoefficients = checkPIDFCoefficients(pidfCoefficientsArg);
            if (pidfCoefficients != null) {
                pidfCoefficients.f = f;
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
    @Block(classes = {PIDFCoefficients.class}, fieldName = {RequestConditions.REQUEST_KEY_FILE})
    public double getF(Object pidfCoefficientsArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".F");
            PIDFCoefficients pidfCoefficients = checkPIDFCoefficients(pidfCoefficientsArg);
            if (pidfCoefficients != null) {
                return pidfCoefficients.f;
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
    @Block(classes = {PIDFCoefficients.class}, fieldName = {"algorithm"})
    public void setAlgorithm(Object pidfCoefficientsArg, String algorithmString) {
        try {
            startBlockExecution(BlockType.SETTER, ".Algorithm");
            PIDFCoefficients pidfCoefficients = checkPIDFCoefficients(pidfCoefficientsArg);
            MotorControlAlgorithm algorithm = (MotorControlAlgorithm) checkArg(algorithmString, MotorControlAlgorithm.class, "");
            if (pidfCoefficients != null && algorithm != null) {
                pidfCoefficients.algorithm = algorithm;
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
    @Block(classes = {PIDFCoefficients.class}, fieldName = {"algorithm"})
    public String getAlgorithm(Object pidfCoefficientsArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".Algorithm");
            PIDFCoefficients pidfCoefficients = checkPIDFCoefficients(pidfCoefficientsArg);
            if (pidfCoefficients != null && pidfCoefficients.algorithm != null) {
                return pidfCoefficients.algorithm.toString();
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
    @Block(classes = {PIDFCoefficients.class}, methodName = {"toString"})
    public String toText(Object pidfCoefficientsArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            PIDFCoefficients pidfCoefficients = checkPIDFCoefficients(pidfCoefficientsArg);
            if (pidfCoefficients != null) {
                return pidfCoefficients.toString();
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
