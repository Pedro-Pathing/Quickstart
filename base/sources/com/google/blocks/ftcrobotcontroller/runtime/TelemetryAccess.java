package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* JADX INFO: loaded from: classes8.dex */
class TelemetryAccess extends Access {
    private final Telemetry telemetry;

    TelemetryAccess(BlocksOpMode blocksOpMode, String identifier, Telemetry telemetry) {
        super(blocksOpMode, identifier, "Telemetry");
        this.telemetry = telemetry;
    }

    @JavascriptInterface
    @Block(classes = {Telemetry.class}, methodName = {"addData"})
    public void addNumericData(String key, double data) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".addData");
            this.telemetry.addData(key, Double.valueOf(data));
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
    @Block(classes = {Telemetry.class}, methodName = {"addData"})
    public void addTextData(String key, String data) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".addData");
            if (data != null) {
                this.telemetry.addData(key, data);
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
    @Block(classes = {Telemetry.class}, methodName = {"addData"})
    public void addObjectData(String key, Object data) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".addData");
            this.telemetry.addData(key, "" + data);
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
    @Block(classes = {Telemetry.class}, methodName = {"addLine"})
    public void addLine(String text) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".addLine");
            this.telemetry.addLine(text);
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
    @Block(classes = {Telemetry.class}, methodName = {"update"})
    public void update() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".update");
            this.telemetry.update();
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
    @Block(classes = {Telemetry.class}, methodName = {"speak"})
    public void speakTextData(String data, String languageCode, String countryCode) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".speak");
            if (data != null) {
                this.telemetry.speak(data, languageCode, countryCode);
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
    @Block(classes = {Telemetry.class}, methodName = {"speak"})
    public void speakObjectData(Object data, String languageCode, String countryCode) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".speak");
            this.telemetry.speak("" + data, languageCode, countryCode);
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
    @Block(classes = {Telemetry.class}, methodName = {"setDisplayFormat"})
    public void setDisplayFormat(String displayFormatString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setDisplayFormat");
            Telemetry.DisplayFormat displayFormat = (Telemetry.DisplayFormat) checkArg(displayFormatString, Telemetry.DisplayFormat.class, "displayFormat");
            if (displayFormat != null) {
                this.telemetry.setDisplayFormat(displayFormat);
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
    @Block(classes = {Telemetry.class}, methodName = {"setNumDecimalPlaces"})
    public void setNumDecimalPlaces(int minDecimalPlaces, int maxDecimalPlaces) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setNumDecimalPlaces");
            this.telemetry.setNumDecimalPlaces(minDecimalPlaces, maxDecimalPlaces);
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
    @Block(classes = {Telemetry.class}, methodName = {"setMsTransmissionInterval"})
    public void setMsTransmissionInterval(int interval) {
        try {
            startBlockExecution(BlockType.SETTER, ".MsTransmissionInterval");
            this.telemetry.setMsTransmissionInterval(interval);
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
    @Block(classes = {Telemetry.class}, methodName = {"getMsTransmissionInterval"})
    public int getMsTransmissionInterval() {
        try {
            startBlockExecution(BlockType.GETTER, ".MsTransmissionInterval");
            return this.telemetry.getMsTransmissionInterval();
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
