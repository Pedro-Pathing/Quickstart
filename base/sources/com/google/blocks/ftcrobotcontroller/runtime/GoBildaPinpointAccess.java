package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/* JADX INFO: loaded from: classes8.dex */
class GoBildaPinpointAccess extends HardwareAccess<GoBildaPinpointDriver> {
    private final GoBildaPinpointDriver goBildaPinpoint;

    GoBildaPinpointAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, GoBildaPinpointDriver.class);
        this.goBildaPinpoint = (GoBildaPinpointDriver) this.hardwareDevice;
    }

    private GoBildaPinpointDriver.GoBildaOdometryPods checkGoBildaOdometryPods(String goBildaOdometryPodsString) {
        return (GoBildaPinpointDriver.GoBildaOdometryPods) checkArg(goBildaOdometryPodsString, GoBildaPinpointDriver.GoBildaOdometryPods.class, "pods");
    }

    private GoBildaPinpointDriver.EncoderDirection checkEncoderDirection(String encoderDirectionString, String socketName) {
        return (GoBildaPinpointDriver.EncoderDirection) checkArg(encoderDirectionString, GoBildaPinpointDriver.EncoderDirection.class, socketName);
    }

    private GoBildaPinpointDriver.ReadData checkReadData(String readDataString) {
        return (GoBildaPinpointDriver.ReadData) checkArg(readDataString, GoBildaPinpointDriver.ReadData.class, "readData");
    }

    @JavascriptInterface
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"setYawScalar"})
    public void setYawScalar(double yawOffset) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setYawScalar");
            this.goBildaPinpoint.setYawScalar(yawOffset);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getYawScalar"})
    public float getYawScalar() {
        try {
            startBlockExecution(BlockType.GETTER, ".YawScalar");
            return this.goBildaPinpoint.getYawScalar();
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getDeviceID"})
    public int getDeviceID() {
        try {
            startBlockExecution(BlockType.GETTER, ".DeviceID");
            return this.goBildaPinpoint.getDeviceID();
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getDeviceVersion"})
    public int getDeviceVersion() {
        try {
            startBlockExecution(BlockType.GETTER, ".DeviceVersion");
            return this.goBildaPinpoint.getDeviceVersion();
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getLoopTime"})
    public int getLoopTime() {
        try {
            startBlockExecution(BlockType.GETTER, ".LoopTime");
            return this.goBildaPinpoint.getLoopTime();
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getEncoderX"})
    public int getEncoderX() {
        try {
            startBlockExecution(BlockType.GETTER, ".EncoderX");
            return this.goBildaPinpoint.getEncoderX();
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getEncoderY"})
    public int getEncoderY() {
        try {
            startBlockExecution(BlockType.GETTER, ".EncoderY");
            return this.goBildaPinpoint.getEncoderY();
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getFrequency"})
    public double getFrequency() {
        try {
            startBlockExecution(BlockType.GETTER, ".Frequency");
            return this.goBildaPinpoint.getFrequency();
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getDeviceStatus"})
    public String getDeviceStatus() {
        try {
            startBlockExecution(BlockType.GETTER, ".DeviceStatus");
            GoBildaPinpointDriver.DeviceStatus deviceStatus = this.goBildaPinpoint.getDeviceStatus();
            if (deviceStatus != null) {
                return deviceStatus.toString();
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getPosition"})
    public Pose2D getPosition() {
        try {
            startBlockExecution(BlockType.GETTER, ".Position");
            return this.goBildaPinpoint.getPosition();
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"update"})
    public void update() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".update");
            this.goBildaPinpoint.update();
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"update"})
    public void update_withReadData(String readDataString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".update");
            GoBildaPinpointDriver.ReadData readData = checkReadData(readDataString);
            if (readData != null) {
                this.goBildaPinpoint.update(readData);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"setOffsets"})
    public void setOffsets(double xOffset, double yOffset, String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setOffsets");
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (distanceUnit != null) {
                this.goBildaPinpoint.setOffsets(xOffset, yOffset, distanceUnit);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"recalibrateIMU"})
    public void recalibrateIMU() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".recalibrateIMU");
            this.goBildaPinpoint.recalibrateIMU();
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"resetPosAndIMU"})
    public void resetPosAndIMU() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".resetPosAndIMU");
            this.goBildaPinpoint.resetPosAndIMU();
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"setEncoderDirections"})
    public void setEncoderDirections(String xEncoderString, String yEncoderString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setEncoderDirections");
            GoBildaPinpointDriver.EncoderDirection xEncoder = checkEncoderDirection(xEncoderString, "xEncoder");
            GoBildaPinpointDriver.EncoderDirection yEncoder = checkEncoderDirection(yEncoderString, "yEncoder");
            if (xEncoder != null && yEncoder != null) {
                this.goBildaPinpoint.setEncoderDirections(xEncoder, yEncoder);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"setEncoderResolution"})
    public void setEncoderResolution_withPods(String podsString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setEncoderResolution");
            GoBildaPinpointDriver.GoBildaOdometryPods pods = checkGoBildaOdometryPods(podsString);
            if (pods != null) {
                this.goBildaPinpoint.setEncoderResolution(pods);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"setEncoderResolution"})
    public void setEncoderResolution_withTicks(double ticksPerUnit, String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setEncoderResolution");
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (distanceUnit != null) {
                this.goBildaPinpoint.setEncoderResolution(ticksPerUnit, distanceUnit);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"setPosition"})
    public void setPosition(Object pose2DArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setPosition");
            Pose2D pose2D = checkPose2D(pose2DArg, "position");
            if (pose2D != null) {
                this.goBildaPinpoint.setPosition(pose2D);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"setPosX"})
    public void setPosX(double posX, String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setPosX");
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (distanceUnit != null) {
                this.goBildaPinpoint.setPosX(posX, distanceUnit);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"setPosY"})
    public void setPosY(double posY, String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setPosY");
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (distanceUnit != null) {
                this.goBildaPinpoint.setPosY(posY, distanceUnit);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"setHeading"})
    public void setHeading(double heading, String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setHeading");
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (angleUnit != null) {
                this.goBildaPinpoint.setHeading(heading, angleUnit);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getPosX"})
    public double getPosX(String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getPosX");
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (distanceUnit != null) {
                return this.goBildaPinpoint.getPosX(distanceUnit);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getPosY"})
    public double getPosY(String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getPosY");
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (distanceUnit != null) {
                return this.goBildaPinpoint.getPosY(distanceUnit);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getHeading"})
    public double getHeading_withAngleUnit(String angleUnitString) {
        try {
            startBlockExecution(BlockType.GETTER, ".Heading");
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (angleUnit != null) {
                return this.goBildaPinpoint.getHeading(angleUnit);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getHeading"})
    public double getHeading_withUnnormalizedAngleUnit(String unnormalizedAngleUnitString) {
        try {
            startBlockExecution(BlockType.GETTER, ".Heading");
            UnnormalizedAngleUnit unnormalizedAngleUnit = checkUnnormalizedAngleUnit(unnormalizedAngleUnitString);
            if (unnormalizedAngleUnit != null) {
                return this.goBildaPinpoint.getHeading(unnormalizedAngleUnit);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getVelX"})
    public double getVelX(String distanceUnitString) {
        try {
            startBlockExecution(BlockType.GETTER, ".VelX");
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (distanceUnit != null) {
                return this.goBildaPinpoint.getVelX(distanceUnit);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getVelY"})
    public double getVelY(String distanceUnitString) {
        try {
            startBlockExecution(BlockType.GETTER, ".VelY");
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (distanceUnit != null) {
                return this.goBildaPinpoint.getVelY(distanceUnit);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getHeadingVelocity"})
    public double getHeadingVelocity_withUnnormalizedAngleUnit(String unnormalizedAngleUnitString) {
        try {
            startBlockExecution(BlockType.GETTER, ".HeadingVelocity");
            UnnormalizedAngleUnit unnormalizedAngleUnit = checkUnnormalizedAngleUnit(unnormalizedAngleUnitString);
            if (unnormalizedAngleUnit != null) {
                return this.goBildaPinpoint.getHeadingVelocity(unnormalizedAngleUnit);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getXOffset"})
    public float getXOffset(String distanceUnitString) {
        try {
            startBlockExecution(BlockType.GETTER, ".XOffset");
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (distanceUnit != null) {
                return this.goBildaPinpoint.getXOffset(distanceUnit);
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
    @Block(classes = {GoBildaPinpointDriver.class}, methodName = {"getYOffset"})
    public float getYOffset(String distanceUnitString) {
        try {
            startBlockExecution(BlockType.GETTER, ".YOffset");
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (distanceUnit != null) {
                return this.goBildaPinpoint.getYOffset(distanceUnit);
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
}
