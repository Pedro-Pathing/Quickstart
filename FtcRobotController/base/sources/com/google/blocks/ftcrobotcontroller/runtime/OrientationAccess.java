package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/* JADX INFO: loaded from: classes8.dex */
class OrientationAccess extends Access {
    OrientationAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "Orientation");
    }

    @JavascriptInterface
    @Block(classes = {Orientation.class}, fieldName = {"axesReference"})
    public String getAxesReference(Object orientationArg) {
        AxesReference axesReference;
        try {
            startBlockExecution(BlockType.GETTER, ".AxesReference");
            Orientation orientation = checkOrientation(orientationArg);
            if (orientation != null && (axesReference = orientation.axesReference) != null) {
                return axesReference.toString();
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
    @Block(classes = {Orientation.class}, fieldName = {"axesOrder"})
    public String getAxesOrder(Object orientationArg) {
        AxesOrder axesOrder;
        try {
            startBlockExecution(BlockType.GETTER, ".AxesOrder");
            Orientation orientation = checkOrientation(orientationArg);
            if (orientation != null && (axesOrder = orientation.axesOrder) != null) {
                return axesOrder.toString();
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
    @Block(classes = {Orientation.class}, fieldName = {"angleUnit"})
    public String getAngleUnit(Object orientationArg) {
        AngleUnit angleUnit;
        try {
            startBlockExecution(BlockType.GETTER, ".AngleUnit");
            Orientation orientation = checkOrientation(orientationArg);
            if (orientation != null && (angleUnit = orientation.angleUnit) != null) {
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
    @Block(classes = {Orientation.class}, fieldName = {"firstAngle"})
    public float getFirstAngle(Object orientationArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".FirstAngle");
            Orientation orientation = checkOrientation(orientationArg);
            if (orientation != null) {
                return orientation.firstAngle;
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
    @Block(classes = {Orientation.class}, fieldName = {"secondAngle"})
    public float getSecondAngle(Object orientationArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".SecondAngle");
            Orientation orientation = checkOrientation(orientationArg);
            if (orientation != null) {
                return orientation.secondAngle;
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
    @Block(classes = {Orientation.class}, fieldName = {"thirdAngle"})
    public float getThirdAngle(Object orientationArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".ThirdAngle");
            Orientation orientation = checkOrientation(orientationArg);
            if (orientation != null) {
                return orientation.thirdAngle;
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
    @Block(classes = {Orientation.class}, fieldName = {"acquisitionTime"})
    public long getAcquisitionTime(Object orientationArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".AcquisitionTime");
            Orientation orientation = checkOrientation(orientationArg);
            if (orientation != null) {
                return orientation.acquisitionTime;
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
    @Block(classes = {Orientation.class}, constructor = true)
    public Orientation create() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new Orientation();
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
    @Block(classes = {Orientation.class}, constructor = true)
    public Orientation create_withArgs(String axesReferenceString, String axesOrderString, String angleUnitString, float firstAngle, float secondAngle, float thirdAngle, long acquisitionTime) {
        AxesReference axesReference;
        try {
            startBlockExecution(BlockType.CREATE, "");
            axesReference = checkAxesReference(axesReferenceString);
        } catch (Throwable th) {
            e = th;
        }
        try {
            AxesOrder axesOrder = checkAxesOrder(axesOrderString);
            try {
                AngleUnit angleUnit = checkAngleUnit(angleUnitString);
                if (axesReference != null && axesOrder != null && angleUnit != null) {
                    return new Orientation(axesReference, axesOrder, angleUnit, firstAngle, secondAngle, thirdAngle, acquisitionTime);
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
        } catch (Throwable th3) {
            e = th3;
            this.blocksOpMode.handleFatalException(e);
            throw new AssertionError("impossible", e);
        }
    }

    @JavascriptInterface
    @Block(classes = {Orientation.class}, constructor = true)
    public Orientation create_withArgs2(String axesReferenceString, String axesOrderString, String angleUnitString, float firstAngle, float secondAngle, float thirdAngle) {
        AxesReference axesReference;
        try {
            startBlockExecution(BlockType.CREATE, "");
            axesReference = checkAxesReference(axesReferenceString);
        } catch (Throwable th) {
            e = th;
        }
        try {
            AxesOrder axesOrder = checkAxesOrder(axesOrderString);
            try {
                AngleUnit angleUnit = checkAngleUnit(angleUnitString);
                if (axesReference != null && axesOrder != null && angleUnit != null) {
                    return new Orientation(axesReference, axesOrder, angleUnit, firstAngle, secondAngle, thirdAngle, 0L);
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
        } catch (Throwable th3) {
            e = th3;
            this.blocksOpMode.handleFatalException(e);
            throw new AssertionError("impossible", e);
        }
    }

    @JavascriptInterface
    @Block(classes = {Orientation.class}, methodName = {"toAngleUnit"})
    public Orientation toAngleUnit(Object orientationArg, String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toAngleUnit");
            Orientation orientation = checkOrientation(orientationArg);
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (orientation != null && angleUnit != null) {
                return orientation.toAngleUnit(angleUnit);
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
    @Block(classes = {Orientation.class}, methodName = {"toAxesReference"})
    public Orientation toAxesReference(Object orientationArg, String axesReferenceString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toAxesReference");
            Orientation orientation = checkOrientation(orientationArg);
            AxesReference axesReference = checkAxesReference(axesReferenceString);
            if (orientation != null && axesReference != null) {
                return orientation.toAxesReference(axesReference);
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
    @Block(classes = {Orientation.class}, methodName = {"toAxesOrder"})
    public Orientation toAxesOrder(Object orientationArg, String axesOrderString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toAxesOrder");
            Orientation orientation = checkOrientation(orientationArg);
            AxesOrder axesOrder = checkAxesOrder(axesOrderString);
            if (orientation != null && axesOrder != null) {
                return orientation.toAxesOrder(axesOrder);
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
    @Block(classes = {Orientation.class}, methodName = {"toString"})
    public String toText(Object orientationArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            Orientation orientation = checkOrientation(orientationArg);
            if (orientation != null) {
                return orientation.toString();
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
    @Block(classes = {Orientation.class}, methodName = {"getRotationMatrix"})
    public OpenGLMatrix getRotationMatrix(Object orientationArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getRotationMatrix");
            Orientation orientation = checkOrientation(orientationArg);
            if (orientation != null) {
                return orientation.getRotationMatrix();
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
    @Block(classes = {Orientation.class}, methodName = {"getRotationMatrix"})
    public OpenGLMatrix getRotationMatrix_withArgs(String axesReferenceString, String axesOrderString, String angleUnitString, float firstAngle, float secondAngle, float thirdAngle) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getRotationMatrix");
            AxesReference axesReference = checkAxesReference(axesReferenceString);
            AxesOrder axesOrder = checkAxesOrder(axesOrderString);
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (axesReference != null && axesOrder != null && angleUnit != null) {
                return Orientation.getRotationMatrix(axesReference, axesOrder, angleUnit, firstAngle, secondAngle, thirdAngle);
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
    @Block(classes = {Orientation.class}, methodName = {"getOrientation"})
    public Orientation getOrientation(Object matrixArg, String axesReferenceString, String axesOrderString, String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getOrientation");
            MatrixF matrix = checkMatrixF(matrixArg);
            AxesReference axesReference = checkAxesReference(axesReferenceString);
            AxesOrder axesOrder = checkAxesOrder(axesOrderString);
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (matrix != null && axesReference != null && axesOrder != null && angleUnit != null) {
                return Orientation.getOrientation(matrix, axesReference, axesOrder, angleUnit);
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
}
