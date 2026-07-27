package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.lynx.LynxServoController;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;

/* JADX INFO: loaded from: classes8.dex */
class TemperatureAccess extends Access {
    TemperatureAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "Temperature");
    }

    private Temperature checkTemperature(Object temperatureArg) {
        return (Temperature) checkArg(temperatureArg, Temperature.class, "temperature");
    }

    private TempUnit checkTempUnit(String tempUnitString) {
        return (TempUnit) checkArg(tempUnitString, TempUnit.class, "tempUnit");
    }

    @JavascriptInterface
    @Block(classes = {Temperature.class}, fieldName = {"unit"})
    public String getTempUnit(Object temperatureArg) {
        TempUnit tempUnit;
        try {
            startBlockExecution(BlockType.GETTER, ".TempUnit");
            Temperature temperature = checkTemperature(temperatureArg);
            if (temperature != null && (tempUnit = temperature.unit) != null) {
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

    @JavascriptInterface
    @Block(classes = {Temperature.class}, fieldName = {"temperature"})
    public double getTemperature(Object temperatureArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".Temperature");
            Temperature temperature = checkTemperature(temperatureArg);
            if (temperature != null) {
                return temperature.temperature;
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
    @Block(classes = {Temperature.class}, fieldName = {"acquisitionTime"})
    public long getAcquisitionTime(Object temperatureArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".AcquisitionTime");
            Temperature temperature = checkTemperature(temperatureArg);
            if (temperature != null) {
                return temperature.acquisitionTime;
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
    @Block(classes = {Temperature.class}, constructor = true)
    public Temperature create() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new Temperature();
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
    @Block(classes = {Temperature.class}, constructor = true)
    public Temperature create_withArgs(String tempUnitString, double temperature, long acquisitionTime) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            TempUnit tempUnit = checkTempUnit(tempUnitString);
            if (tempUnit != null) {
                return new Temperature(tempUnit, temperature, acquisitionTime);
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
    @Block(classes = {Temperature.class}, methodName = {"toUnit"})
    public Temperature toTempUnit(Object temperatureArg, String tempUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toTempUnit");
            Temperature temperature = checkTemperature(temperatureArg);
            TempUnit tempUnit = checkTempUnit(tempUnitString);
            if (temperature != null && tempUnit != null) {
                return temperature.toUnit(tempUnit);
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
