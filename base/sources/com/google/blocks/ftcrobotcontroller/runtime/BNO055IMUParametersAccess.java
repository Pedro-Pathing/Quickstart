package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.I2cAddr;

/* JADX INFO: loaded from: classes8.dex */
class BNO055IMUParametersAccess extends Access {

    enum Algorithm {
        NAIVE,
        JUST_LOGGING
    }

    BNO055IMUParametersAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "IMU-BNO055.Parameters");
    }

    @JavascriptInterface
    @Block(classes = {BNO055IMU.Parameters.class}, constructor = true)
    public BNO055IMU.Parameters create() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new BNO055IMU.Parameters();
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"accelUnit"})
    public void setAccelUnit(Object parametersArg, String accelUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setAccelUnit");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            BNO055IMU.AccelUnit accelUnit = (BNO055IMU.AccelUnit) checkArg(accelUnitString, BNO055IMU.AccelUnit.class, "accelUnit");
            if (parameters != null && accelUnit != null) {
                parameters.accelUnit = accelUnit;
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"accelUnit"})
    public String getAccelUnit(Object parametersArg) {
        BNO055IMU.AccelUnit accelUnit;
        try {
            startBlockExecution(BlockType.GETTER, ".AccelUnit");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null && (accelUnit = parameters.accelUnit) != null) {
                return accelUnit.toString();
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"accelerationIntegrationAlgorithm"})
    public void setAccelerationIntegrationAlgorithm(Object parametersArg, String algorithmString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setAccelerationIntegrationAlgorithm");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            Algorithm algorithm = (Algorithm) checkArg(algorithmString, Algorithm.class, "accelerationIntegrationAlgorithm");
            if (parameters != null && algorithm != null) {
                switch (algorithm) {
                    case NAIVE:
                        parameters.accelerationIntegrationAlgorithm = null;
                        break;
                    case JUST_LOGGING:
                        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                        break;
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"accelerationIntegrationAlgorithm"})
    public String getAccelerationIntegrationAlgorithm(Object parametersArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".AccelerationIntegrationAlgorithm");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null) {
                BNO055IMU.AccelerationIntegrator accelerationIntegrator = parameters.accelerationIntegrationAlgorithm;
                if (accelerationIntegrator != null && !(accelerationIntegrator instanceof NaiveAccelerationIntegrator)) {
                    if (accelerationIntegrator instanceof JustLoggingAccelerationIntegrator) {
                        return "JUST_LOGGING";
                    }
                }
                return "NAIVE";
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"angleUnit"})
    public void setAngleUnit(Object parametersArg, String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setAngleUnit");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            BNO055IMU.AngleUnit angleUnit = (BNO055IMU.AngleUnit) checkArg(angleUnitString, BNO055IMU.AngleUnit.class, "angleUnit");
            if (parameters != null && angleUnit != null) {
                parameters.angleUnit = angleUnit;
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"angleUnit"})
    public String getAngleUnit(Object parametersArg) {
        BNO055IMU.AngleUnit angleUnit;
        try {
            startBlockExecution(BlockType.GETTER, ".AngleUnit");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null && (angleUnit = parameters.angleUnit) != null) {
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"calibrationDataFile"})
    public void setCalibrationDataFile(Object parametersArg, String calibrationDataFile) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setCalibrationDataFile");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null) {
                parameters.calibrationDataFile = calibrationDataFile;
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"calibrationDataFile"})
    public String getCalibrationDataFile(Object parametersArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".CalibrationDataFile");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null) {
                String calibrationDataFile = parameters.calibrationDataFile;
                if (calibrationDataFile != null) {
                    return calibrationDataFile;
                }
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"i2cAddr"})
    public void setI2cAddress7Bit(Object parametersArg, int i2cAddr7Bit) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setI2cAddress7Bit");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null) {
                parameters.i2cAddr = I2cAddr.create7bit(i2cAddr7Bit);
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"i2cAddr"})
    public int getI2cAddress7Bit(Object parametersArg) {
        I2cAddr i2cAddr;
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress7Bit");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null && (i2cAddr = parameters.i2cAddr) != null) {
                return i2cAddr.get7Bit();
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"i2cAddr"})
    public void setI2cAddress8Bit(Object parametersArg, int i2cAddr8Bit) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setI2cAddress8Bit");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null) {
                parameters.i2cAddr = I2cAddr.create8bit(i2cAddr8Bit);
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"i2cAddr"})
    public int getI2cAddress8Bit(Object parametersArg) {
        I2cAddr i2cAddr;
        try {
            startBlockExecution(BlockType.GETTER, ".I2cAddress8Bit");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null && (i2cAddr = parameters.i2cAddr) != null) {
                return i2cAddr.get8Bit();
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"loggingEnabled"})
    public void setLoggingEnabled(Object parametersArg, boolean loggingEnabled) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setLoggingEnabled");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null) {
                parameters.loggingEnabled = loggingEnabled;
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"loggingEnabled"})
    public boolean getLoggingEnabled(Object parametersArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".LoggingEnabled");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null) {
                return parameters.loggingEnabled;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"loggingTag"})
    public void setLoggingTag(Object parametersArg, String loggingTag) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setLoggingTag");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null) {
                parameters.loggingTag = loggingTag;
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"loggingTag"})
    public String getLoggingTag(Object parametersArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".LoggingTag");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null) {
                String loggingTag = parameters.loggingTag;
                if (loggingTag != null) {
                    return loggingTag;
                }
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"mode"})
    public void setSensorMode(Object parametersArg, String sensorModeString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setSensorMode");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            BNO055IMU.SensorMode sensorMode = (BNO055IMU.SensorMode) checkArg(sensorModeString, BNO055IMU.SensorMode.class, "sensorMode");
            if (parameters != null && sensorMode != null) {
                parameters.mode = sensorMode;
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"mode"})
    public String getSensorMode(Object parametersArg) {
        BNO055IMU.SensorMode sensorMode;
        try {
            startBlockExecution(BlockType.GETTER, ".SensorMode");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null && (sensorMode = parameters.mode) != null) {
                return sensorMode.toString();
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"temperatureUnit"})
    public void setTempUnit(Object parametersArg, String tempUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setTempUnit");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            BNO055IMU.TempUnit tempUnit = (BNO055IMU.TempUnit) checkArg(tempUnitString, BNO055IMU.TempUnit.class, "tempUnit");
            if (parameters != null && tempUnit != null) {
                parameters.temperatureUnit = tempUnit;
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
    @Block(classes = {BNO055IMU.Parameters.class}, fieldName = {"temperatureUnit"})
    public String getTempUnit(Object parametersArg) {
        BNO055IMU.TempUnit tempUnit;
        try {
            startBlockExecution(BlockType.GETTER, ".TempUnit");
            BNO055IMU.Parameters parameters = checkBNO055IMUParameters(parametersArg);
            if (parameters != null && (tempUnit = parameters.temperatureUnit) != null) {
                return tempUnit.toString();
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
