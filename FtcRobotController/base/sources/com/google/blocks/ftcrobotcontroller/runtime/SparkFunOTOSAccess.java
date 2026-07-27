package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

/* JADX INFO: loaded from: classes8.dex */
class SparkFunOTOSAccess extends HardwareAccess<SparkFunOTOS> {
    private final SparkFunOTOS sparkFunOTOS;

    SparkFunOTOSAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, SparkFunOTOS.class);
        this.sparkFunOTOS = (SparkFunOTOS) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getImuCalibrationProgress"})
    public int getImuCalibrationProgress() {
        try {
            startBlockExecution(BlockType.GETTER, ".ImuCalibrationProgress");
            return this.sparkFunOTOS.getImuCalibrationProgress();
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getLinearScalar"})
    public double getLinearScalar() {
        try {
            startBlockExecution(BlockType.GETTER, ".LinearScalar");
            return this.sparkFunOTOS.getLinearScalar();
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"setLinearScalar"})
    public boolean setLinearScalar(double scalar) {
        try {
            startBlockExecution(BlockType.SETTER, ".LinearScalar");
            return this.sparkFunOTOS.setLinearScalar(scalar);
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getAngularScalar"})
    public double getAngularScalar() {
        try {
            startBlockExecution(BlockType.GETTER, ".AngularScalar");
            return this.sparkFunOTOS.getAngularScalar();
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"setAngularScalar"})
    public boolean setAngularScalar(double scalar) {
        try {
            startBlockExecution(BlockType.SETTER, ".AngularScalar");
            return this.sparkFunOTOS.setAngularScalar(scalar);
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getLinearUnit"})
    public String getLinearUnit() {
        try {
            startBlockExecution(BlockType.GETTER, ".LinearUnit");
            DistanceUnit distanceUnit = this.sparkFunOTOS.getLinearUnit();
            if (distanceUnit != null) {
                return distanceUnit.toString();
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"setLinearUnit"})
    public void setLinearUnit(String distanceUnitString) {
        try {
            startBlockExecution(BlockType.SETTER, ".LinearUnit");
            DistanceUnit distanceUnit = (DistanceUnit) checkArg(distanceUnitString, DistanceUnit.class, "");
            if (distanceUnit != null) {
                this.sparkFunOTOS.setLinearUnit(distanceUnit);
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getAngularUnit"})
    public String getAngularUnit() {
        try {
            startBlockExecution(BlockType.GETTER, ".AngularUnit");
            AngleUnit angleUnit = this.sparkFunOTOS.getAngularUnit();
            if (angleUnit != null) {
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"setAngularUnit"})
    public void setAngularUnit(String angleUnitString) {
        try {
            startBlockExecution(BlockType.SETTER, ".AngularUnit");
            AngleUnit angleUnit = (AngleUnit) checkArg(angleUnitString, AngleUnit.class, "");
            if (angleUnit != null) {
                this.sparkFunOTOS.setAngularUnit(angleUnit);
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getStatus"})
    public String getStatus() {
        try {
            startBlockExecution(BlockType.GETTER, ".Status");
            return SimpleGson.getInstance().toJson(this.sparkFunOTOS.getStatus());
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getOffset"})
    public String getOffset() {
        try {
            startBlockExecution(BlockType.GETTER, ".Offset");
            return SimpleGson.getInstance().toJson(this.sparkFunOTOS.getOffset());
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"setOffset"})
    public void setOffset(String json) {
        try {
            startBlockExecution(BlockType.SETTER, ".Offset");
            SparkFunOTOS.Pose2D pose = (SparkFunOTOS.Pose2D) SimpleGson.getInstance().fromJson(json, SparkFunOTOS.Pose2D.class);
            this.sparkFunOTOS.setOffset(pose);
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getPosition"})
    public String getPosition() {
        try {
            startBlockExecution(BlockType.GETTER, ".Position");
            return SimpleGson.getInstance().toJson(this.sparkFunOTOS.getPosition());
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"setPosition"})
    public void setPosition(String json) {
        try {
            startBlockExecution(BlockType.SETTER, ".Position");
            SparkFunOTOS.Pose2D pose = (SparkFunOTOS.Pose2D) SimpleGson.getInstance().fromJson(json, SparkFunOTOS.Pose2D.class);
            this.sparkFunOTOS.setPosition(pose);
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getAcceleration"})
    public String getAcceleration() {
        try {
            startBlockExecution(BlockType.GETTER, ".Acceleration");
            return SimpleGson.getInstance().toJson(this.sparkFunOTOS.getAcceleration());
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getVelocity"})
    public String getVelocity() {
        try {
            startBlockExecution(BlockType.GETTER, ".Velocity");
            return SimpleGson.getInstance().toJson(this.sparkFunOTOS.getVelocity());
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getPositionStdDev"})
    public String getPositionStdDev() {
        try {
            startBlockExecution(BlockType.GETTER, ".PositionStdDev");
            return SimpleGson.getInstance().toJson(this.sparkFunOTOS.getPositionStdDev());
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getAccelerationStdDev"})
    public String getAccelerationStdDev() {
        try {
            startBlockExecution(BlockType.GETTER, ".AccelerationStdDev");
            return SimpleGson.getInstance().toJson(this.sparkFunOTOS.getAccelerationStdDev());
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getVelocityStdDev"})
    public String getVelocityStdDev() {
        try {
            startBlockExecution(BlockType.GETTER, ".VelocityStdDev");
            return SimpleGson.getInstance().toJson(this.sparkFunOTOS.getVelocityStdDev());
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getSignalProcessConfig"})
    public String getSignalProcessConfig() {
        try {
            startBlockExecution(BlockType.GETTER, ".SignalProcessConfig");
            return SimpleGson.getInstance().toJson(this.sparkFunOTOS.getSignalProcessConfig());
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"setSignalProcessConfig"})
    public void setSignalProcessConfig(String json) {
        try {
            startBlockExecution(BlockType.SETTER, ".SignalProcessConfig");
            SparkFunOTOS.SignalProcessConfig signalProcessConfig = (SparkFunOTOS.SignalProcessConfig) SimpleGson.getInstance().fromJson(json, SparkFunOTOS.SignalProcessConfig.class);
            this.sparkFunOTOS.setSignalProcessConfig(signalProcessConfig);
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getPosVelAcc"})
    public String getPosVelAcc(String json1, String json2, String json3) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getPosVelAcc");
            SimpleGson.getInstance().fromJson(json1, SparkFunOTOS.Pose2D.class);
            SimpleGson.getInstance().fromJson(json2, SparkFunOTOS.Pose2D.class);
            SimpleGson.getInstance().fromJson(json3, SparkFunOTOS.Pose2D.class);
            SparkFunOTOS.Pose2D[] posVelAcc = new SparkFunOTOS.Pose2D[3];
            for (int i = 0; i < posVelAcc.length; i++) {
                posVelAcc[i] = new SparkFunOTOS.Pose2D();
            }
            this.sparkFunOTOS.getPosVelAcc(posVelAcc[0], posVelAcc[1], posVelAcc[2]);
            return SimpleGson.getInstance().toJson(posVelAcc);
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getPosVelAccStdDev"})
    public String getPosVelAccStdDev(String json1, String json2, String json3) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getPosVelAccStdDev");
            SimpleGson.getInstance().fromJson(json1, SparkFunOTOS.Pose2D.class);
            SimpleGson.getInstance().fromJson(json2, SparkFunOTOS.Pose2D.class);
            SimpleGson.getInstance().fromJson(json3, SparkFunOTOS.Pose2D.class);
            SparkFunOTOS.Pose2D[] posVelAccStdDev = new SparkFunOTOS.Pose2D[3];
            for (int i = 0; i < posVelAccStdDev.length; i++) {
                posVelAccStdDev[i] = new SparkFunOTOS.Pose2D();
            }
            this.sparkFunOTOS.getPosVelAccStdDev(posVelAccStdDev[0], posVelAccStdDev[1], posVelAccStdDev[2]);
            return SimpleGson.getInstance().toJson(posVelAccStdDev);
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getPosVelAccAndStdDev"})
    public String getPosVelAccAndStdDev(String json1, String json2, String json3, String json4, String json5, String json6) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getPosVelAccAndStdDev");
            SimpleGson.getInstance().fromJson(json1, SparkFunOTOS.Pose2D.class);
            SimpleGson.getInstance().fromJson(json2, SparkFunOTOS.Pose2D.class);
            SimpleGson.getInstance().fromJson(json3, SparkFunOTOS.Pose2D.class);
            SimpleGson.getInstance().fromJson(json4, SparkFunOTOS.Pose2D.class);
            SimpleGson.getInstance().fromJson(json5, SparkFunOTOS.Pose2D.class);
            SimpleGson.getInstance().fromJson(json6, SparkFunOTOS.Pose2D.class);
            SparkFunOTOS.Pose2D[] posVelAccAndStdDev = new SparkFunOTOS.Pose2D[6];
            for (int i = 0; i < posVelAccAndStdDev.length; i++) {
                posVelAccAndStdDev[i] = new SparkFunOTOS.Pose2D();
            }
            this.sparkFunOTOS.getPosVelAccAndStdDev(posVelAccAndStdDev[0], posVelAccAndStdDev[1], posVelAccAndStdDev[2], posVelAccAndStdDev[3], posVelAccAndStdDev[4], posVelAccAndStdDev[5]);
            return SimpleGson.getInstance().toJson(posVelAccAndStdDev);
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"getVersionInfo"})
    public String getVersionInfo(String json1, String json2) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getVersionInfo");
            SimpleGson.getInstance().fromJson(json1, SparkFunOTOS.Version.class);
            SimpleGson.getInstance().fromJson(json2, SparkFunOTOS.Version.class);
            SparkFunOTOS.Version[] versionInfo = new SparkFunOTOS.Version[2];
            for (int i = 0; i < versionInfo.length; i++) {
                versionInfo[i] = new SparkFunOTOS.Version();
            }
            this.sparkFunOTOS.getVersionInfo(versionInfo[0], versionInfo[1]);
            return SimpleGson.getInstance().toJson(versionInfo);
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"resetTracking"})
    public void resetTracking() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".resetTracking");
            this.sparkFunOTOS.resetTracking();
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"begin"})
    public boolean begin() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".begin");
            return this.sparkFunOTOS.begin();
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"calibrateImu"})
    public boolean calibrateImu() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".calibrateImu");
            return this.sparkFunOTOS.calibrateImu();
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"calibrateImu"})
    public boolean calibrateImu_withArgs(int numSamples, boolean waitUntilDone) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".calibrateImu");
            return this.sparkFunOTOS.calibrateImu(numSamples, waitUntilDone);
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"isConnected"})
    public boolean isConnected() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isConnected");
            return this.sparkFunOTOS.isConnected();
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
    @Block(classes = {SparkFunOTOS.class}, methodName = {"selfTest"})
    public boolean selfTest() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".selfTest");
            return this.sparkFunOTOS.selfTest();
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
