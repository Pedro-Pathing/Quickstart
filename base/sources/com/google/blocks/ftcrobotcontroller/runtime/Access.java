package com.google.blocks.ftcrobotcontroller.runtime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.HashSet;
import java.util.Locale;
import java.util.Set;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

/* JADX INFO: loaded from: classes8.dex */
public abstract class Access {
    protected static final String DEFAULT_CAMERA_MONTIOR_FEEDBACK_STRING = "DEFAULT";
    protected final String blockFirstName;
    protected final BlocksOpMode blocksOpMode;
    private final String identifier;
    private final Set<String> warningsReported = new HashSet();

    protected Access(BlocksOpMode blocksOpMode, String identifier, String blockFirstName) {
        this.blocksOpMode = blocksOpMode;
        this.identifier = identifier;
        this.blockFirstName = blockFirstName;
    }

    void close() {
    }

    protected final void startBlockExecution(BlockType blockType, String blockLastName) {
        this.blocksOpMode.startBlockExecution(blockType, this.blockFirstName, blockLastName);
    }

    protected final void startBlockExecution(BlockType blockType, String blockFirstNameOverride, String blockLastName) {
        this.blocksOpMode.startBlockExecution(blockType, blockFirstNameOverride, blockLastName);
    }

    protected final void endBlockExecution() {
        this.blocksOpMode.endBlockExecution();
    }

    protected final void handleObsoleteBlockExecution(BlockType blockType, String blockLastName) {
        startBlockExecution(blockType, blockLastName);
        reportWarning("This block is obsolete.");
    }

    protected final void handleObsoleteBlockExecution(BlockType blockType, String blockFirstNameOverride, String blockLastName) {
        startBlockExecution(blockType, blockFirstNameOverride, blockLastName);
        reportWarning("This block is obsolete.");
    }

    protected AngleUnit checkAngleUnit(String angleUnitString) {
        return (AngleUnit) checkArg(angleUnitString, AngleUnit.class, "angleUnit");
    }

    protected AxesOrder checkAxesOrder(String axesOrderString) {
        return (AxesOrder) checkArg(axesOrderString, AxesOrder.class, "axesOrder");
    }

    protected AxesReference checkAxesReference(String axesReferenceString) {
        return (AxesReference) checkArg(axesReferenceString, AxesReference.class, "axesReference");
    }

    protected DistanceUnit checkDistanceUnit(String distanceUnitString) {
        return (DistanceUnit) checkArg(distanceUnitString, DistanceUnit.class, "distanceUnit");
    }

    protected RevBlinkinLedDriver.BlinkinPattern checkBlinkinPattern(String blinkinPatternString) {
        return (RevBlinkinLedDriver.BlinkinPattern) checkArg(blinkinPatternString, RevBlinkinLedDriver.BlinkinPattern.class, "blinkinPattern");
    }

    protected BNO055IMU.Parameters checkBNO055IMUParameters(Object parametersArg) {
        return (BNO055IMU.Parameters) checkArg(parametersArg, BNO055IMU.Parameters.class, "parameters");
    }

    protected ImageRegion checkImageRegion(Object imageRegionArg) {
        return (ImageRegion) checkArg(imageRegionArg, ImageRegion.class, "roi");
    }

    protected Orientation checkOrientation(Object orientationArg) {
        return (Orientation) checkArg(orientationArg, Orientation.class, "orientation");
    }

    protected Orientation checkOrientation(Object orientationArg, String socketName) {
        return (Orientation) checkArg(orientationArg, Orientation.class, socketName);
    }

    protected Pose2D checkPose2D(Object pose2DArg) {
        return (Pose2D) checkArg(pose2DArg, Pose2D.class, "pose2D");
    }

    protected Pose2D checkPose2D(Object pose2DArg, String socketName) {
        return (Pose2D) checkArg(pose2DArg, Pose2D.class, socketName);
    }

    protected Position checkPosition(Object positionArg) {
        return (Position) checkArg(positionArg, Position.class, "position");
    }

    protected Quaternion checkQuaternion(Object quaternionArg) {
        return (Quaternion) checkArg(quaternionArg, Quaternion.class, "quaternion");
    }

    protected Quaternion checkQuaternion(Object quaternionArg, String socketName) {
        return (Quaternion) checkArg(quaternionArg, Quaternion.class, socketName);
    }

    protected MatrixF checkMatrixF(Object matrixArg) {
        return (MatrixF) checkArg(matrixArg, MatrixF.class, "matrix");
    }

    protected OpenGLMatrix checkOpenGLMatrix(Object matrixArg) {
        return (OpenGLMatrix) checkArg(matrixArg, OpenGLMatrix.class, "matrix");
    }

    protected UnnormalizedAngleUnit checkUnnormalizedAngleUnit(String unnormalizedAngleUnitString) {
        return (UnnormalizedAngleUnit) checkArg(unnormalizedAngleUnitString, UnnormalizedAngleUnit.class, "unnormalizedAngleUnit");
    }

    protected VectorF checkVectorF(Object vectorArg) {
        return (VectorF) checkArg(vectorArg, VectorF.class, "vector");
    }

    protected YawPitchRollAngles checkYawPitchRollAngles(Object yawPitchRollAnglesArg) {
        return (YawPitchRollAngles) checkArg(yawPitchRollAnglesArg, YawPitchRollAngles.class, "yawPitchRollAngles");
    }

    protected final <T> T checkArg(Object arg, Class<T> clazz, String socketName) {
        if (!clazz.isInstance(arg)) {
            reportInvalidArg(socketName, getTypeFromClass(clazz));
            return null;
        }
        return clazz.cast(arg);
    }

    protected final <T extends Enum<T>> T checkArg(String str, Class<T> cls, String str2) {
        if (str == null) {
            reportInvalidArg(str2, cls.getSimpleName());
            return null;
        }
        try {
            return (T) Enum.valueOf(cls, str);
        } catch (IllegalArgumentException e) {
            try {
                return (T) Enum.valueOf(cls, str.toUpperCase(Locale.ENGLISH));
            } catch (IllegalArgumentException e2) {
                reportInvalidArg(str2, cls.getSimpleName());
                return null;
            }
        }
    }

    protected final void reportInvalidArg(String socketName, String expectedType) {
        if (socketName != null && !socketName.isEmpty()) {
            reportWarning("Incorrect block plugged into the %s socket of the block labeled \"%s\". Expected %s.", socketName, this.blocksOpMode.getFullBlockLabel(), expectedType);
        } else {
            reportWarning("Incorrect block plugged into a socket of the block labeled \"%s\". Expected %s.", this.blocksOpMode.getFullBlockLabel(), expectedType);
        }
    }

    protected final void reportWarning(String message) {
        reportWarning("Warning while (or after) executing the block labeled \"%s\". %s", this.blocksOpMode.getFullBlockLabel(), message);
    }

    protected final void reportHardwareError(String message) {
        reportWarning("Error while initializing hardware items. %s", this.blocksOpMode.getFullBlockLabel(), message);
    }

    private final void reportWarning(String format, Object... args) {
        String message = String.format(format, args);
        RobotLog.ww("Blocks", message);
        if (this.warningsReported.add(message)) {
            RobotLog.addGlobalWarningMessage(message);
        }
    }

    protected static final String getTypeFromClass(Class clazz) {
        String type = clazz.getSimpleName();
        while (clazz.getEnclosingClass() != null) {
            clazz = clazz.getEnclosingClass();
            type = clazz.getSimpleName() + "." + type;
        }
        return type;
    }

    protected static String toJson(Object o) {
        if (o == null) {
            return "null";
        }
        return SimpleGson.getInstance().toJson(o);
    }

    protected static <T> T fromJson(String str, Class<T> cls) {
        if (str == null || str.equals("null") || str.equals("undefined")) {
            return null;
        }
        return (T) SimpleGson.getInstance().fromJson(str, (Class) cls);
    }
}
