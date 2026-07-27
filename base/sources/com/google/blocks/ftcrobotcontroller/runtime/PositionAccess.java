package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.lynx.LynxServoController;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/* JADX INFO: loaded from: classes8.dex */
class PositionAccess extends Access {
    PositionAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "Position");
    }

    @JavascriptInterface
    @Block(classes = {Position.class}, fieldName = {"unit"})
    public String getDistanceUnit(Object positionArg) {
        DistanceUnit distanceUnit;
        try {
            startBlockExecution(BlockType.GETTER, ".DistanceUnit");
            Position position = checkPosition(positionArg);
            if (position != null && (distanceUnit = position.unit) != null) {
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
    @Block(classes = {Position.class}, fieldName = {"x"})
    public double getX(Object positionArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".X");
            Position position = checkPosition(positionArg);
            if (position != null) {
                return position.x;
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
    @Block(classes = {Position.class}, fieldName = {"y"})
    public double getY(Object positionArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".Y");
            Position position = checkPosition(positionArg);
            if (position != null) {
                return position.y;
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
    @Block(classes = {Position.class}, fieldName = {"z"})
    public double getZ(Object positionArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".Z");
            Position position = checkPosition(positionArg);
            if (position != null) {
                return position.z;
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
    @Block(classes = {Position.class}, fieldName = {"acquisitionTime"})
    public long getAcquisitionTime(Object positionArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".AcquisitionTime");
            Position position = checkPosition(positionArg);
            if (position != null) {
                return position.acquisitionTime;
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
    @Block(classes = {Position.class}, constructor = true)
    public Position create() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new Position();
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
    @Block(classes = {Position.class}, constructor = true)
    public Position create_withArgs(String distanceUnitString, double x, double y, double z, long acquisitionTime) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (distanceUnit != null) {
                return new Position(distanceUnit, x, y, z, acquisitionTime);
            }
            endBlockExecution();
            return null;
        } finally {
        }
    }

    @JavascriptInterface
    @Block(classes = {Position.class}, methodName = {"toUnit"})
    public Position toDistanceUnit(Object positionArg, String distanceUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toDistanceUnit");
            Position position = checkPosition(positionArg);
            DistanceUnit distanceUnit = checkDistanceUnit(distanceUnitString);
            if (position != null && distanceUnit != null) {
                return position.toUnit(distanceUnit);
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
    @Block(classes = {Position.class}, methodName = {"toString"})
    public String toText(Object positionArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            Position position = checkPosition(positionArg);
            if (position != null) {
                return position.toString();
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
