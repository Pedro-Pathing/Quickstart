package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/* JADX INFO: loaded from: classes8.dex */
class DcMotorAccess extends HardwareAccess<DcMotor> {
    private final DcMotor dcMotor;

    DcMotorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, DcMotor.class);
        this.dcMotor = (DcMotor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"setDirection"})
    public void setDirection(String directionString) {
        try {
            startBlockExecution(BlockType.SETTER, ".Direction");
            DcMotorSimple.Direction direction = (DcMotorSimple.Direction) checkArg(directionString, DcMotorSimple.Direction.class, "");
            if (direction != null) {
                this.dcMotor.setDirection(direction);
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"getDirection"})
    public String getDirection() {
        try {
            startBlockExecution(BlockType.GETTER, ".Direction");
            DcMotorSimple.Direction direction = this.dcMotor.getDirection();
            if (direction != null) {
                return direction.toString();
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"setPower"})
    public void setPower(double power) {
        try {
            startBlockExecution(BlockType.SETTER, ".Power");
            this.dcMotor.setPower(power);
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"getPower"})
    public double getPower() {
        try {
            startBlockExecution(BlockType.GETTER, ".Power");
            return this.dcMotor.getPower();
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
    @Deprecated
    public void setMaxSpeed(double maxSpeed) {
        try {
            startBlockExecution(BlockType.SETTER, ".MaxSpeed");
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
    @Deprecated
    public int getMaxSpeed() {
        try {
            startBlockExecution(BlockType.GETTER, ".MaxSpeed");
            endBlockExecution();
            return 0;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } catch (Throwable e2) {
                endBlockExecution();
                throw e2;
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"setZeroPowerBehavior"})
    public void setZeroPowerBehavior(String zeroPowerBehaviorString) {
        try {
            startBlockExecution(BlockType.SETTER, ".ZeroPowerBehavior");
            DcMotor.ZeroPowerBehavior zeroPowerBehavior = (DcMotor.ZeroPowerBehavior) checkArg(zeroPowerBehaviorString, DcMotor.ZeroPowerBehavior.class, "");
            if (zeroPowerBehavior != null) {
                this.dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"getZeroPowerBehavior"})
    public String getZeroPowerBehavior() {
        try {
            startBlockExecution(BlockType.GETTER, ".ZeroPowerBehavior");
            DcMotor.ZeroPowerBehavior zeroPowerBehavior = this.dcMotor.getZeroPowerBehavior();
            if (zeroPowerBehavior != null) {
                return zeroPowerBehavior.toString();
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"getPowerFloat"})
    public boolean getPowerFloat() {
        try {
            startBlockExecution(BlockType.GETTER, ".PowerFloat");
            return this.dcMotor.getPowerFloat();
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"setTargetPosition"})
    public void setTargetPosition(double position) {
        try {
            startBlockExecution(BlockType.SETTER, ".TargetPosition");
            this.dcMotor.setTargetPosition((int) position);
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"getTargetPosition"})
    public int getTargetPosition() {
        try {
            startBlockExecution(BlockType.GETTER, ".TargetPosition");
            return this.dcMotor.getTargetPosition();
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"isBusy"})
    public boolean isBusy() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isBusy");
            return this.dcMotor.isBusy();
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"getCurrentPosition"})
    public int getCurrentPosition() {
        try {
            startBlockExecution(BlockType.GETTER, ".CurrentPosition");
            return this.dcMotor.getCurrentPosition();
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"setMode"})
    public void setMode(String runModeString) {
        try {
            startBlockExecution(BlockType.SETTER, ".Mode");
            DcMotor.RunMode runMode = (DcMotor.RunMode) checkArg(runModeString, DcMotor.RunMode.class, "");
            if (runMode != null) {
                this.dcMotor.setMode(runMode);
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"getMode"})
    public String getMode() {
        try {
            startBlockExecution(BlockType.GETTER, ".Mode");
            DcMotor.RunMode runMode = this.dcMotor.getMode();
            if (runMode != null) {
                return runMode.toString();
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
    @Deprecated
    public void setDualMaxSpeed(double maxSpeed1, Object otherArg, double maxSpeed2) {
        try {
            startBlockExecution(BlockType.SETTER, ".MaxSpeed");
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"setMode"})
    public void setDualMode(String runMode1String, Object otherArg, String runMode2String) {
        try {
            startBlockExecution(BlockType.SETTER, ".Mode");
            DcMotor.RunMode runMode1 = (DcMotor.RunMode) checkArg(runMode1String, DcMotor.RunMode.class, "first");
            DcMotor.RunMode runMode2 = (DcMotor.RunMode) checkArg(runMode2String, DcMotor.RunMode.class, "second");
            if (runMode1 != null && runMode2 != null && (otherArg instanceof DcMotorAccess)) {
                DcMotorAccess other = (DcMotorAccess) otherArg;
                this.dcMotor.setMode(runMode1);
                other.dcMotor.setMode(runMode2);
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"setPower"})
    public void setDualPower(double power1, Object otherArg, double power2) {
        try {
            startBlockExecution(BlockType.SETTER, ".Power");
            if (otherArg instanceof DcMotorAccess) {
                DcMotorAccess other = (DcMotorAccess) otherArg;
                this.dcMotor.setPower(power1);
                other.dcMotor.setPower(power2);
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"setTargetPosition"})
    public void setDualTargetPosition(double position1, Object otherArg, double position2) {
        try {
            startBlockExecution(BlockType.SETTER, ".TargetPosition");
            if (otherArg instanceof DcMotorAccess) {
                DcMotorAccess other = (DcMotorAccess) otherArg;
                this.dcMotor.setTargetPosition((int) position1);
                other.dcMotor.setTargetPosition((int) position2);
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
    @Block(classes = {DcMotorEx.class}, methodName = {"setTargetPositionTolerance"})
    public void setDualTargetPositionTolerance(double tolerance1, Object otherArg, double tolerance2) {
        try {
            startBlockExecution(BlockType.SETTER, ".TargetPositionTolerance");
            if (otherArg instanceof DcMotorAccess) {
                DcMotorAccess other = (DcMotorAccess) otherArg;
                if (this.dcMotor instanceof DcMotorEx) {
                    ((DcMotorEx) this.dcMotor).setTargetPositionTolerance((int) tolerance1);
                }
                if (other.dcMotor instanceof DcMotorEx) {
                    ((DcMotorEx) other.dcMotor).setTargetPositionTolerance((int) tolerance2);
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
    @Block(classes = {DcMotorEx.class}, methodName = {"setVelocity"})
    public void setDualVelocity(double velocity1, Object otherArg, double velocity2) {
        try {
            startBlockExecution(BlockType.SETTER, ".Velocity");
            if (otherArg instanceof DcMotorAccess) {
                DcMotorAccess other = (DcMotorAccess) otherArg;
                if (this.dcMotor instanceof DcMotorEx) {
                    ((DcMotorEx) this.dcMotor).setVelocity(velocity1);
                }
                if (other.dcMotor instanceof DcMotorEx) {
                    ((DcMotorEx) other.dcMotor).setVelocity(velocity2);
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
    @Block(classes = {DcMotor.class, DcMotorEx.class}, methodName = {"setZeroPowerBehavior"})
    public void setDualZeroPowerBehavior(String zeroPowerBehavior1String, Object otherArg, String zeroPowerBehavior2String) {
        try {
            startBlockExecution(BlockType.SETTER, ".ZeroPowerBehavior");
            DcMotor.ZeroPowerBehavior zeroPowerBehavior1 = (DcMotor.ZeroPowerBehavior) checkArg(zeroPowerBehavior1String, DcMotor.ZeroPowerBehavior.class, "first");
            DcMotor.ZeroPowerBehavior zeroPowerBehavior2 = (DcMotor.ZeroPowerBehavior) checkArg(zeroPowerBehavior2String, DcMotor.ZeroPowerBehavior.class, "second");
            if (zeroPowerBehavior1 != null && zeroPowerBehavior2 != null && (otherArg instanceof DcMotorAccess)) {
                DcMotorAccess other = (DcMotorAccess) otherArg;
                this.dcMotor.setZeroPowerBehavior(zeroPowerBehavior1);
                other.dcMotor.setZeroPowerBehavior(zeroPowerBehavior2);
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
    @Block(classes = {DcMotorEx.class}, methodName = {"setMotorEnable"})
    public void setMotorEnable() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setMotorEnable");
            if (this.dcMotor instanceof DcMotorEx) {
                ((DcMotorEx) this.dcMotor).setMotorEnable();
            } else {
                reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"setMotorDisable"})
    public void setMotorDisable() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setMotorDisable");
            if (this.dcMotor instanceof DcMotorEx) {
                ((DcMotorEx) this.dcMotor).setMotorDisable();
            } else {
                reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"isMotorEnabled"})
    public boolean isMotorEnabled() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isMotorEnabled");
            if (this.dcMotor instanceof DcMotorEx) {
                return ((DcMotorEx) this.dcMotor).isMotorEnabled();
            }
            reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"setVelocity"})
    public void setVelocity(double angularRate) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setVelocity");
            if (this.dcMotor instanceof DcMotorEx) {
                ((DcMotorEx) this.dcMotor).setVelocity(angularRate);
            } else {
                reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"setVelocity"})
    public void setVelocity_withAngleUnit(double angularRate, String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setVelocity");
            AngleUnit angleUnit = (AngleUnit) checkArg(angleUnitString, AngleUnit.class, "");
            if (angleUnit != null) {
                if (this.dcMotor instanceof DcMotorEx) {
                    ((DcMotorEx) this.dcMotor).setVelocity(angularRate, angleUnit);
                } else {
                    reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"getVelocity"})
    public double getVelocity() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getVelocity");
            if (this.dcMotor instanceof DcMotorEx) {
                return ((DcMotorEx) this.dcMotor).getVelocity();
            }
            reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"getVelocity"})
    public double getVelocity_withAngleUnit(String angleUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getVelocity");
            AngleUnit angleUnit = (AngleUnit) checkArg(angleUnitString, AngleUnit.class, "");
            if (angleUnit != null) {
                if (this.dcMotor instanceof DcMotorEx) {
                    return ((DcMotorEx) this.dcMotor).getVelocity(angleUnit);
                }
                reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"setVelocityPIDFCoefficients"})
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setVelocityPIDFCoefficients");
            if (this.dcMotor instanceof DcMotorEx) {
                ((DcMotorEx) this.dcMotor).setVelocityPIDFCoefficients(p, i, d, f);
            } else {
                reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"setPositionPIDFCoefficients"})
    public void setPositionPIDFCoefficients(double p) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setPositionPIDFCoefficients");
            if (this.dcMotor instanceof DcMotorEx) {
                ((DcMotorEx) this.dcMotor).setPositionPIDFCoefficients(p);
            } else {
                reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"setPIDFCoefficients"})
    public void setPIDFCoefficients(String runModeString, Object pidfCoefficientsArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setPIDFCoefficients");
            DcMotor.RunMode runMode = (DcMotor.RunMode) checkArg(runModeString, DcMotor.RunMode.class, "");
            PIDFCoefficients pidfCoefficients = (PIDFCoefficients) checkArg(pidfCoefficientsArg, PIDFCoefficients.class, "");
            if (runMode != null && pidfCoefficients != null) {
                if (this.dcMotor instanceof DcMotorEx) {
                    ((DcMotorEx) this.dcMotor).setPIDFCoefficients(runMode, pidfCoefficients);
                } else {
                    reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"getPIDFCoefficients"})
    public PIDFCoefficients getPIDFCoefficients(String runModeString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getPIDFCoefficients");
            DcMotor.RunMode runMode = (DcMotor.RunMode) checkArg(runModeString, DcMotor.RunMode.class, "");
            if (runMode != null) {
                if (this.dcMotor instanceof DcMotorEx) {
                    return ((DcMotorEx) this.dcMotor).getPIDFCoefficients(runMode);
                }
                reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"setTargetPositionTolerance"})
    public void setTargetPositionTolerance(int tolerance) {
        try {
            startBlockExecution(BlockType.SETTER, ".TargetPositionTolerance");
            if (this.dcMotor instanceof DcMotorEx) {
                ((DcMotorEx) this.dcMotor).setTargetPositionTolerance(tolerance);
            } else {
                reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"getTargetPositionTolerance"})
    public int getTargetPositionTolerance() {
        try {
            startBlockExecution(BlockType.GETTER, ".TargetPositionTolerance");
            if (this.dcMotor instanceof DcMotorEx) {
                return ((DcMotorEx) this.dcMotor).getTargetPositionTolerance();
            }
            reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"getCurrent"})
    public double getCurrent(String currentUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getCurrent");
            CurrentUnit currentUnit = (CurrentUnit) checkArg(currentUnitString, CurrentUnit.class, "");
            if (currentUnit != null) {
                if (this.dcMotor instanceof DcMotorEx) {
                    return ((DcMotorEx) this.dcMotor).getCurrent(currentUnit);
                }
                reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"getCurrentAlert"})
    public double getCurrentAlert(String currentUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getCurrentAlert");
            CurrentUnit currentUnit = (CurrentUnit) checkArg(currentUnitString, CurrentUnit.class, "");
            if (currentUnit != null) {
                if (this.dcMotor instanceof DcMotorEx) {
                    return ((DcMotorEx) this.dcMotor).getCurrentAlert(currentUnit);
                }
                reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"setCurrentAlert"})
    public void setCurrentAlert(double current, String currentUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setCurrentAlert");
            CurrentUnit currentUnit = (CurrentUnit) checkArg(currentUnitString, CurrentUnit.class, "");
            if (currentUnit != null) {
                if (this.dcMotor instanceof DcMotorEx) {
                    ((DcMotorEx) this.dcMotor).setCurrentAlert(current, currentUnit);
                } else {
                    reportWarning("This DcMotor is not a DcMotorEx.");
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
    @Block(classes = {DcMotorEx.class}, methodName = {"isOverCurrent"})
    public boolean isOverCurrent() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isOverCurrent");
            if (this.dcMotor instanceof DcMotorEx) {
                return ((DcMotorEx) this.dcMotor).isOverCurrent();
            }
            reportWarning("This DcMotor is not a DcMotorEx.");
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
}
