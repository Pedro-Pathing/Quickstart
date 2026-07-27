package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

/* JADX INFO: loaded from: classes8.dex */
class QuaternionAccess extends Access {
    QuaternionAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "Quaternion");
    }

    @JavascriptInterface
    @Block(classes = {Quaternion.class}, fieldName = {"w"})
    public float getW(Object quaternionArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".W");
            Quaternion quaternion = checkQuaternion(quaternionArg);
            if (quaternion != null) {
                return quaternion.w;
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
    @Block(classes = {Quaternion.class}, fieldName = {"x"})
    public float getX(Object quaternionArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".X");
            Quaternion quaternion = checkQuaternion(quaternionArg);
            if (quaternion != null) {
                return quaternion.x;
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
    @Block(classes = {Quaternion.class}, fieldName = {"y"})
    public float getY(Object quaternionArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".Y");
            Quaternion quaternion = checkQuaternion(quaternionArg);
            if (quaternion != null) {
                return quaternion.y;
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
    @Block(classes = {Quaternion.class}, fieldName = {"z"})
    public float getZ(Object quaternionArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".Z");
            Quaternion quaternion = checkQuaternion(quaternionArg);
            if (quaternion != null) {
                return quaternion.z;
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
    @Block(classes = {Quaternion.class}, fieldName = {"acquisitionTime"})
    public long getAcquisitionTime(Object quaternionArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".AcquisitionTime");
            Quaternion quaternion = checkQuaternion(quaternionArg);
            if (quaternion != null) {
                return quaternion.acquisitionTime;
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
    @Block(classes = {Quaternion.class}, methodName = {"magnitude"})
    public float getMagnitude(Object quaternionArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".Magnitude");
            Quaternion quaternion = checkQuaternion(quaternionArg);
            if (quaternion != null) {
                return quaternion.magnitude();
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
    @Block(classes = {Quaternion.class}, constructor = true)
    public Quaternion create() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new Quaternion();
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
    @Block(classes = {Quaternion.class}, constructor = true)
    public Quaternion create_withArgs(float w, float x, float y, float z, long acquisitionTime) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new Quaternion(w, x, y, z, acquisitionTime);
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
    @Block(classes = {Quaternion.class}, constructor = true)
    public Quaternion create_withArgs2(float w, float x, float y, float z) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new Quaternion(w, x, y, z, 0L);
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
    @Block(classes = {Quaternion.class}, methodName = {"normalized"})
    public Quaternion normalized(Object quaternionArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".normalized");
            Quaternion quaternion = checkQuaternion(quaternionArg);
            if (quaternion != null) {
                return quaternion.normalized();
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
    @Block(classes = {Quaternion.class}, methodName = {"congugate"})
    public Quaternion congugate(Object quaternionArg) {
        return conjugate(quaternionArg);
    }

    @JavascriptInterface
    @Block(classes = {Quaternion.class}, methodName = {"conjugate"})
    public Quaternion conjugate(Object quaternionArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".conjugate");
            Quaternion quaternion = checkQuaternion(quaternionArg);
            if (quaternion != null) {
                return quaternion.conjugate();
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
    @Block(classes = {Quaternion.class}, methodName = {"toString"})
    public String toText(Object quaternionArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            Quaternion quaternion = checkQuaternion(quaternionArg);
            if (quaternion != null) {
                return quaternion.toString();
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
