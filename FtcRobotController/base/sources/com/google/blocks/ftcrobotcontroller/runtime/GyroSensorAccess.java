package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.sun.tools.doclint.DocLint;
import java.util.Set;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/* JADX INFO: loaded from: classes8.dex */
class GyroSensorAccess extends HardwareAccess<GyroSensor> {
    private final GyroSensor gyroSensor;

    GyroSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, GyroSensor.class);
        this.gyroSensor = (GyroSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {GyroSensor.class, ModernRoboticsI2cGyro.class}, methodName = {"getHeading"})
    public int getHeading() {
        try {
            startBlockExecution(BlockType.GETTER, ".Heading");
            return this.gyroSensor.getHeading();
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
    @Block(classes = {ModernRoboticsI2cGyro.class}, methodName = {"setHeadingMode"})
    public void setHeadingMode(String headingModeString) {
        try {
            startBlockExecution(BlockType.SETTER, ".HeadingMode");
            ModernRoboticsI2cGyro.HeadingMode headingMode = (ModernRoboticsI2cGyro.HeadingMode) checkArg(headingModeString, ModernRoboticsI2cGyro.HeadingMode.class, "");
            if (headingMode != null) {
                if (this.gyroSensor instanceof ModernRoboticsI2cGyro) {
                    ((ModernRoboticsI2cGyro) this.gyroSensor).setHeadingMode(headingMode);
                } else {
                    reportWarning("This GyroSensor is not a ModernRoboticsI2cGyro.");
                }
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
    @Block(classes = {ModernRoboticsI2cGyro.class}, methodName = {"getHeadingMode"})
    public String getHeadingMode() {
        try {
            startBlockExecution(BlockType.GETTER, ".HeadingMode");
            if (this.gyroSensor instanceof ModernRoboticsI2cGyro) {
                ModernRoboticsI2cGyro.HeadingMode headingMode = ((ModernRoboticsI2cGyro) this.gyroSensor).getHeadingMode();
                if (headingMode != null) {
                    return headingMode.toString();
                }
            } else {
                reportWarning("This GyroSensor is not a ModernRoboticsI2cGyro.");
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
    @Block(classes = {ModernRoboticsI2cGyro.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress7Bit(int i2cAddr7Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress7Bit");
            if (this.gyroSensor instanceof ModernRoboticsI2cGyro) {
                ((ModernRoboticsI2cGyro) this.gyroSensor).setI2cAddress(I2cAddr.create7bit(i2cAddr7Bit));
            } else {
                reportWarning("This GyroSensor is not a ModernRoboticsI2cGyro.");
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
    @Block(classes = {ModernRoboticsI2cGyro.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress7Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress7Bit");
            if (this.gyroSensor instanceof ModernRoboticsI2cGyro) {
                I2cAddr i2cAddr = ((ModernRoboticsI2cGyro) this.gyroSensor).getI2cAddress();
                if (i2cAddr != null) {
                    return i2cAddr.get7Bit();
                }
            } else {
                reportWarning("This GyroSensor is not a ModernRoboticsI2cGyro.");
            }
            endBlockExecution();
            return 0;
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
    @Block(classes = {ModernRoboticsI2cGyro.class}, methodName = {"setI2cAddress"})
    public void setI2cAddress8Bit(int i2cAddr8Bit) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cAddress8Bit");
            if (this.gyroSensor instanceof ModernRoboticsI2cGyro) {
                ((ModernRoboticsI2cGyro) this.gyroSensor).setI2cAddress(I2cAddr.create8bit(i2cAddr8Bit));
            } else {
                reportWarning("This GyroSensor is not a ModernRoboticsI2cGyro.");
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
    @Block(classes = {ModernRoboticsI2cGyro.class}, methodName = {"getI2cAddress"})
    public int getI2cAddress8Bit() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress8Bit");
            if (this.gyroSensor instanceof ModernRoboticsI2cGyro) {
                I2cAddr i2cAddr = ((ModernRoboticsI2cGyro) this.gyroSensor).getI2cAddress();
                if (i2cAddr != null) {
                    return i2cAddr.get8Bit();
                }
            } else {
                reportWarning("This GyroSensor is not a ModernRoboticsI2cGyro.");
            }
            endBlockExecution();
            return 0;
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
    @Block(classes = {ModernRoboticsI2cGyro.class}, methodName = {"getIntegratedZValue"})
    public int getIntegratedZValue() {
        try {
            startBlockExecution(BlockType.GETTER, ".IntegratedZValue");
            if (this.gyroSensor instanceof ModernRoboticsI2cGyro) {
                return ((ModernRoboticsI2cGyro) this.gyroSensor).getIntegratedZValue();
            }
            reportWarning("This GyroSensor is not a ModernRoboticsI2cGyro.");
            endBlockExecution();
            return 0;
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
    @Block(classes = {GyroSensor.class, ModernRoboticsI2cGyro.class}, methodName = {"rawX"})
    public int getRawX() {
        try {
            startBlockExecution(BlockType.GETTER, ".RawX");
            return this.gyroSensor.rawX();
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
    @Block(classes = {GyroSensor.class, ModernRoboticsI2cGyro.class}, methodName = {"rawY"})
    public int getRawY() {
        try {
            startBlockExecution(BlockType.GETTER, ".RawY");
            return this.gyroSensor.rawY();
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
    @Block(classes = {GyroSensor.class, ModernRoboticsI2cGyro.class}, methodName = {"rawZ"})
    public int getRawZ() {
        try {
            startBlockExecution(BlockType.GETTER, ".RawZ");
            return this.gyroSensor.rawZ();
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
    @Block(classes = {GyroSensor.class, ModernRoboticsI2cGyro.class}, methodName = {"getRotationFraction"})
    public double getRotationFraction() {
        try {
            startBlockExecution(BlockType.GETTER, ".RotationFraction");
            return this.gyroSensor.getRotationFraction();
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
    @Block(classes = {GyroSensor.class, ModernRoboticsI2cGyro.class}, methodName = {"calibrate"})
    public void calibrate() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".calibrate");
            this.gyroSensor.calibrate();
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
    @Block(classes = {GyroSensor.class, ModernRoboticsI2cGyro.class}, methodName = {"isCalibrating"})
    public boolean isCalibrating() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isCalibrating");
            return this.gyroSensor.isCalibrating();
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
    @Block(classes = {GyroSensor.class, ModernRoboticsI2cGyro.class}, methodName = {"resetZAxisIntegrator"})
    public void resetZAxisIntegrator() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".resetZAxisIntegrator");
            this.gyroSensor.resetZAxisIntegrator();
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
    @Block(classes = {Gyroscope.class, ModernRoboticsI2cGyro.class}, methodName = {"getAngularVelocityAxes"})
    public String getAngularVelocityAxes() {
        try {
            startBlockExecution(BlockType.GETTER, ".AngularVelocityAxes");
            if (this.gyroSensor instanceof Gyroscope) {
                Set<Axis> axes = ((Gyroscope) this.gyroSensor).getAngularVelocityAxes();
                StringBuilder sb = new StringBuilder();
                sb.append("[");
                String delimiter = "";
                for (Axis axis : axes) {
                    sb.append(delimiter).append("\"").append(axis.toString()).append("\"");
                    delimiter = DocLint.TAGS_SEPARATOR;
                }
                sb.append("]");
                return sb.toString();
            }
            reportWarning("This GyroSensor is not a Gyroscope.");
            return "[]";
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
    @Block(classes = {Gyroscope.class, ModernRoboticsI2cGyro.class}, methodName = {"getAngularVelocity"})
    public AngularVelocity getAngularVelocity(String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getAngularVelocity");
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (angleUnit == null) {
                endBlockExecution();
                return null;
            }
            if (this.gyroSensor instanceof Gyroscope) {
                return ((Gyroscope) this.gyroSensor).getAngularVelocity(angleUnit);
            }
            reportWarning("This GyroSensor is not a Gyroscope.");
            return new AngularVelocity(angleUnit, 0.0f, 0.0f, 0.0f, 0L);
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
    @Block(classes = {OrientationSensor.class, ModernRoboticsI2cGyro.class}, methodName = {"getAngularOrientationAxes"})
    public String getAngularOrientationAxes() {
        try {
            startBlockExecution(BlockType.GETTER, ".AngularOrientationAxes");
            if (this.gyroSensor instanceof OrientationSensor) {
                Set<Axis> axes = ((OrientationSensor) this.gyroSensor).getAngularOrientationAxes();
                StringBuilder sb = new StringBuilder();
                sb.append("[");
                String delimiter = "";
                for (Axis axis : axes) {
                    sb.append(delimiter).append("\"").append(axis.toString()).append("\"");
                    delimiter = DocLint.TAGS_SEPARATOR;
                }
                sb.append("]");
                return sb.toString();
            }
            reportWarning("This GyroSensor is not a OrientationSensor.");
            return "[]";
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
    @Block(classes = {OrientationSensor.class, ModernRoboticsI2cGyro.class}, methodName = {"getAngularOrientation"})
    public Orientation getAngularOrientation(String axesReferenceString, String axesOrderString, String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getAngularOrientation");
            AxesReference axesReference = checkAxesReference(axesReferenceString);
            try {
                AxesOrder axesOrder = checkAxesOrder(axesOrderString);
                try {
                    AngleUnit angleUnit = checkAngleUnit(angleUnitString);
                    if (axesReference == null || axesOrder == null || angleUnit == null) {
                        endBlockExecution();
                        return null;
                    }
                    if (this.gyroSensor instanceof OrientationSensor) {
                        return ((OrientationSensor) this.gyroSensor).getAngularOrientation(axesReference, axesOrder, angleUnit);
                    }
                    reportWarning("This GyroSensor is not a OrientationSensor.");
                    return new Orientation(axesReference, axesOrder, angleUnit, 0.0f, 0.0f, 0.0f, 0L);
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
        }
    }
}
