package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/* JADX INFO: loaded from: classes8.dex */
class OpenGLMatrixAccess extends Access {
    OpenGLMatrixAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "OpenGLMatrix");
    }

    @JavascriptInterface
    @Block(classes = {OpenGLMatrix.class}, constructor = true)
    public OpenGLMatrix create() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new OpenGLMatrix();
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
    @Block(classes = {OpenGLMatrix.class}, constructor = true)
    public OpenGLMatrix create_withMatrixF(Object matrixArg) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            MatrixF matrix = checkMatrixF(matrixArg);
            if (matrix != null) {
                return new OpenGLMatrix(matrix);
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"rotation"})
    public OpenGLMatrix rotation(String angleUnitString, float angle, float dx, float dy, float dz) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".rotation");
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (angleUnit != null) {
                return OpenGLMatrix.rotation(angleUnit, angle, dx, dy, dz);
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"rotation"})
    public OpenGLMatrix rotation_withAxesArgs(String axesReferenceString, String axesOrderString, String angleUnitString, float firstAngle, float secondAngle, float thirdAngle) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".rotation");
            AxesReference axesReference = checkAxesReference(axesReferenceString);
            AxesOrder axesOrder = checkAxesOrder(axesOrderString);
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (axesReference != null && axesOrder != null && angleUnit != null) {
                return OpenGLMatrix.rotation(axesReference, axesOrder, angleUnit, firstAngle, secondAngle, thirdAngle);
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"translation"})
    public OpenGLMatrix translation(float dx, float dy, float dz) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".translation");
            return OpenGLMatrix.translation(dx, dy, dz);
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"identityMatrix"})
    public OpenGLMatrix identityMatrix() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".identityMatrix");
            return OpenGLMatrix.identityMatrix();
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"scale"})
    public void scale_with3(Object matrixArg, float scaleX, float scaleY, float scaleZ) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".scale");
            OpenGLMatrix matrix = checkOpenGLMatrix(matrixArg);
            if (matrix != null) {
                matrix.scale(scaleX, scaleY, scaleZ);
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"scale"})
    public void scale_with1(Object matrixArg, float scale) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".scale");
            OpenGLMatrix matrix = checkOpenGLMatrix(matrixArg);
            if (matrix != null) {
                matrix.scale(scale);
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"translate"})
    public void translate(Object matrixArg, float dx, float dy, float dz) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".translate");
            OpenGLMatrix matrix = checkOpenGLMatrix(matrixArg);
            if (matrix != null) {
                matrix.translate(dx, dy, dz);
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"rotate"})
    public void rotate(Object matrixArg, String angleUnitString, float angle, float dx, float dy, float dz) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".rotate");
            OpenGLMatrix matrix = checkOpenGLMatrix(matrixArg);
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (matrix != null && angleUnit != null) {
                matrix.rotate(angleUnit, angle, dx, dy, dz);
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"rotate"})
    public void rotate_withAxesArgs(Object matrixArg, String axesReferenceString, String axesOrderString, String angleUnitString, float firstAngle, float secondAngle, float thirdAngle) {
        OpenGLMatrix matrix;
        try {
            startBlockExecution(BlockType.FUNCTION, ".rotate");
            matrix = checkOpenGLMatrix(matrixArg);
        } catch (Throwable th) {
            e = th;
        }
        try {
            AxesReference axesReference = checkAxesReference(axesReferenceString);
            try {
                AxesOrder axesOrder = checkAxesOrder(axesOrderString);
                try {
                    AngleUnit angleUnit = checkAngleUnit(angleUnitString);
                    if (matrix != null && axesReference != null && axesOrder != null && angleUnit != null) {
                        matrix.rotate(axesReference, axesOrder, angleUnit, firstAngle, secondAngle, thirdAngle);
                    }
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
        } catch (Throwable th4) {
            e = th4;
            this.blocksOpMode.handleFatalException(e);
            throw new AssertionError("impossible", e);
        }
    }

    @JavascriptInterface
    @Block(classes = {OpenGLMatrix.class}, methodName = {"scaled"})
    public OpenGLMatrix scaled_with3(Object matrixArg, float scaleX, float scaleY, float scaleZ) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".scaled");
            OpenGLMatrix matrix = checkOpenGLMatrix(matrixArg);
            if (matrix != null) {
                return matrix.scaled(scaleX, scaleY, scaleZ);
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"scaled"})
    public OpenGLMatrix scaled_with1(Object matrixArg, float scale) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".scaled");
            OpenGLMatrix matrix = checkOpenGLMatrix(matrixArg);
            if (matrix != null) {
                return matrix.scaled(scale);
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"translated"})
    public OpenGLMatrix translated(Object matrixArg, float dx, float dy, float dz) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".translated");
            OpenGLMatrix matrix = checkOpenGLMatrix(matrixArg);
            if (matrix != null) {
                return matrix.translated(dx, dy, dz);
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"rotated"})
    public OpenGLMatrix rotated(Object matrixArg, String angleUnitString, float angle, float dx, float dy, float dz) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".rotated");
            OpenGLMatrix matrix = checkOpenGLMatrix(matrixArg);
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (matrix != null && angleUnit != null) {
                return matrix.rotated(angleUnit, angle, dx, dy, dz);
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"rotated"})
    public OpenGLMatrix rotated_withAxesArgs(Object matrixArg, String axesReferenceString, String axesOrderString, String angleUnitString, float firstAngle, float secondAngle, float thirdAngle) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".rotated");
            OpenGLMatrix matrix = checkOpenGLMatrix(matrixArg);
            try {
                AxesReference axesReference = checkAxesReference(axesReferenceString);
                try {
                    AxesOrder axesOrder = checkAxesOrder(axesOrderString);
                    try {
                        AngleUnit angleUnit = checkAngleUnit(angleUnitString);
                        if (matrix != null && axesReference != null && axesOrder != null && angleUnit != null) {
                            return matrix.rotated(axesReference, axesOrder, angleUnit, firstAngle, secondAngle, thirdAngle);
                        }
                        endBlockExecution();
                        return null;
                    } catch (Throwable th) {
                        e = th;
                        try {
                            this.blocksOpMode.handleFatalException(e);
                            throw new AssertionError("impossible", e);
                        } finally {
                            endBlockExecution();
                        }
                    }
                } catch (Throwable th2) {
                    e = th2;
                    this.blocksOpMode.handleFatalException(e);
                    throw new AssertionError("impossible", e);
                }
            } catch (Throwable th3) {
                e = th3;
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            }
        } catch (Throwable th4) {
            e = th4;
        }
    }

    @JavascriptInterface
    @Block(classes = {OpenGLMatrix.class}, methodName = {"multiplied"})
    public OpenGLMatrix multiplied(Object matrix1Arg, Object matrix2Arg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".multiplied");
            OpenGLMatrix matrix1 = (OpenGLMatrix) checkArg(matrix1Arg, OpenGLMatrix.class, "matrix1");
            OpenGLMatrix matrix2 = (OpenGLMatrix) checkArg(matrix2Arg, OpenGLMatrix.class, "matrix2");
            if (matrix1 != null && matrix2 != null) {
                return matrix1.multiplied(matrix2);
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
    @Block(classes = {OpenGLMatrix.class}, methodName = {"multiply"})
    public void multiply(Object matrix1Arg, Object matrix2Arg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".multiply");
            OpenGLMatrix matrix1 = (OpenGLMatrix) checkArg(matrix1Arg, OpenGLMatrix.class, "matrix1");
            OpenGLMatrix matrix2 = (OpenGLMatrix) checkArg(matrix2Arg, OpenGLMatrix.class, "matrix2");
            if (matrix1 != null && matrix2 != null) {
                matrix1.multiply(matrix2);
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
}
