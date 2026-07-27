package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/* JADX INFO: loaded from: classes8.dex */
class ImuAccess extends HardwareAccess<IMU> {
    private final IMU imu;

    ImuAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, IMU.class);
        this.imu = (IMU) this.hardwareDevice;
    }

    private IMU.Parameters checkImuParameters(Object parametersArg) {
        return (IMU.Parameters) checkArg(parametersArg, IMU.Parameters.class, "parameters");
    }

    @JavascriptInterface
    @Block(classes = {IMU.class}, methodName = {"initialize"})
    public void initialize(Object parametersArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".initialize");
            IMU.Parameters parameters = checkImuParameters(parametersArg);
            if (parameters != null && !this.imu.initialize(parameters)) {
                reportWarning("IMU initialization failed");
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
    @Block(classes = {IMU.class}, methodName = {"getRobotAngularVelocity"})
    public AngularVelocity getRobotAngularVelocity(String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getRobotAngularVelocity");
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (angleUnit != null) {
                return this.imu.getRobotAngularVelocity(angleUnit);
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
    @Block(classes = {IMU.class}, methodName = {"getRobotOrientation"})
    public Orientation getRobotOrientation(String axesReferenceString, String axesOrderString, String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getRobotOrientation");
            AxesReference axesReference = checkAxesReference(axesReferenceString);
            AxesOrder axesOrder = checkAxesOrder(axesOrderString);
            AngleUnit angleUnit = checkAngleUnit(angleUnitString);
            if (axesReference != null && axesOrder != null && angleUnit != null) {
                return this.imu.getRobotOrientation(axesReference, axesOrder, angleUnit);
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
    @Block(classes = {IMU.class}, methodName = {"getRobotOrientationAsQuaternion"})
    public Quaternion getRobotOrientationAsQuaternion() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getRobotOrientationAsQuaternion");
            return this.imu.getRobotOrientationAsQuaternion();
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
    @Block(classes = {IMU.class}, methodName = {"getRobotYawPitchRollAngles"})
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getRobotYawPitchRollAngles");
            return this.imu.getRobotYawPitchRollAngles();
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
    @Block(classes = {IMU.class}, methodName = {"resetYaw"})
    public void resetYaw() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".resetYaw");
            this.imu.resetYaw();
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
