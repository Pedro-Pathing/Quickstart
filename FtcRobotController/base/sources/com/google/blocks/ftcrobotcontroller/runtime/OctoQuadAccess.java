package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.sun.tools.doclint.DocLint;

/* JADX INFO: loaded from: classes8.dex */
class OctoQuadAccess extends HardwareAccess<OctoQuad> {
    private final OctoQuad octoQuad;

    OctoQuadAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, OctoQuad.class);
        this.octoQuad = (OctoQuad) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {OctoQuad.class}, methodName = {"getChipId"})
    public int getChipId() {
        try {
            startBlockExecution(BlockType.GETTER, ".ChipId");
            return this.octoQuad.getChipId();
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
    @Block(classes = {OctoQuad.class}, methodName = {"getFirmwareVersionString"})
    public String getFirmwareVersionString() {
        try {
            startBlockExecution(BlockType.GETTER, ".FirmwareVersionString");
            return this.octoQuad.getFirmwareVersionString();
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
    @Block(classes = {OctoQuad.class}, methodName = {"resetAllPositions"})
    public void resetAllPositions() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".resetAllPositions");
            this.octoQuad.resetAllPositions();
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
    @Block(classes = {OctoQuad.class}, methodName = {"resetSinglePosition"})
    public void resetSinglePosition(int index) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".resetSinglePosition");
            this.octoQuad.resetSinglePosition(index);
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
    @Block(classes = {OctoQuad.class}, methodName = {"setSingleEncoderDirection"})
    public void setSingleEncoderDirection(int index, String encoderDirectionString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setEncoderDirection");
            OctoQuad.EncoderDirection encoderDirection = (OctoQuad.EncoderDirection) checkArg(encoderDirectionString, OctoQuad.EncoderDirection.class, "direction");
            if (encoderDirection != null) {
                this.octoQuad.setSingleEncoderDirection(index, encoderDirection);
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
    @Block(classes = {OctoQuad.class}, methodName = {"getSingleEncoderDirection"})
    public String getSingleEncoderDirection(int index) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".SingleEncoderDirection");
            OctoQuad.EncoderDirection encoderDirection = this.octoQuad.getSingleEncoderDirection(index);
            if (encoderDirection != null) {
                return encoderDirection.toString();
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
    @Block(classes = {OctoQuad.class}, methodName = {"setAllVelocitySampleIntervals"})
    public void setAllVelocitySampleIntervals(int intervalMillis) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setAllVelocitySampleIntervals");
            this.octoQuad.setAllVelocitySampleIntervals(intervalMillis);
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
    @Block(classes = {OctoQuad.class}, methodName = {"setSingleVelocitySampleInterval"})
    public void setSingleVelocitySampleInterval(int index, int intervalMillis) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".SingleVelocitySampleInterval");
            this.octoQuad.setSingleVelocitySampleInterval(index, intervalMillis);
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
    @Block(classes = {OctoQuad.class}, methodName = {"getSingleVelocitySampleInterval"})
    public int getSingleVelocitySampleInterval(int index) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getSingleVelocitySampleInterval");
            return this.octoQuad.getSingleVelocitySampleInterval(index);
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
    @Block(classes = {OctoQuad.class}, methodName = {"setSingleChannelPulseWidthParams"})
    public void setSingleChannelPulseWidthParams(int index, int min_length_us, int max_length_us) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setSingleChannelPulseWidthParams");
            this.octoQuad.setSingleChannelPulseWidthParams(index, min_length_us, max_length_us);
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
    @Block(classes = {OctoQuad.class}, methodName = {"resetEverything"})
    public void resetEverything() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".resetEverything");
            this.octoQuad.resetEverything();
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
    @Block(classes = {OctoQuad.class}, methodName = {"setChannelBankConfig"})
    public void setChannelBankConfig(String channelBankConfigString) {
        try {
            startBlockExecution(BlockType.SETTER, ".ChannelBankConfig");
            OctoQuad.ChannelBankConfig channelBankConfig = (OctoQuad.ChannelBankConfig) checkArg(channelBankConfigString, OctoQuad.ChannelBankConfig.class, "");
            if (channelBankConfig != null) {
                this.octoQuad.setChannelBankConfig(channelBankConfig);
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
    @Block(classes = {OctoQuad.class}, methodName = {"getChannelBankConfig"})
    public String getChannelBankConfig() {
        try {
            startBlockExecution(BlockType.GETTER, ".ChannelBankConfig");
            OctoQuad.ChannelBankConfig channelBankConfig = this.octoQuad.getChannelBankConfig();
            if (channelBankConfig != null) {
                return channelBankConfig.toString();
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
    @Block(classes = {OctoQuad.class}, methodName = {"setI2cRecoveryMode"})
    public void setI2cRecoveryMode(String i2cRecoveryModeString) {
        try {
            startBlockExecution(BlockType.SETTER, ".I2cRecoveryMode");
            OctoQuad.I2cRecoveryMode i2cRecoveryMode = (OctoQuad.I2cRecoveryMode) checkArg(i2cRecoveryModeString, OctoQuad.I2cRecoveryMode.class, "");
            if (i2cRecoveryMode != null) {
                this.octoQuad.setI2cRecoveryMode(i2cRecoveryMode);
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
    @Block(classes = {OctoQuad.class}, methodName = {"getI2cRecoveryMode"})
    public String getI2cRecoveryMode() {
        try {
            startBlockExecution(BlockType.GETTER, ".I2cRecoveryMode");
            OctoQuad.I2cRecoveryMode i2cRecoveryMode = this.octoQuad.getI2cRecoveryMode();
            if (i2cRecoveryMode != null) {
                return i2cRecoveryMode.toString();
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
    @Block(classes = {OctoQuad.class}, methodName = {"saveParametersToFlash"})
    public void saveParametersToFlash() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".saveParametersToFlash");
            this.octoQuad.saveParametersToFlash();
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
    @Block(classes = {OctoQuad.class}, methodName = {"setCachingMode"})
    public void setCachingMode(String cachingModeString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setCachingMode");
            if (cachingModeString.equals("NONE")) {
                reportWarning("OctoQuad.CachingMode NONE is obsolete.");
            } else {
                OctoQuad.CachingMode cachingMode = (OctoQuad.CachingMode) checkArg(cachingModeString, OctoQuad.CachingMode.class, "mode");
                if (cachingMode != null) {
                    this.octoQuad.setCachingMode(cachingMode);
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
    @Block(classes = {OctoQuad.class}, methodName = {"refreshCache"})
    public void refreshCache() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".refreshCache");
            this.octoQuad.refreshCache();
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
    @Block(classes = {OctoQuad.class}, methodName = {"readSinglePosition_Caching"})
    public int readSinglePosition_Caching(int index) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".readSinglePosition_Caching");
            return this.octoQuad.readSinglePosition_Caching(index);
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
    @Block(classes = {OctoQuad.class}, methodName = {"readSingleVelocity_Caching"})
    public int readSingleVelocity_Caching(int index) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".readSingleVelocity_Caching");
            return this.octoQuad.readSingleVelocity_Caching(index);
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
    @Block(classes = {OctoQuad.class}, methodName = {"setSingleChannelPulseWidthTracksWrap"})
    public void setSingleChannelPulseWidthTracksWrap(int index, boolean trackWrap) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setSingleChannelPulseWidthTracksWrap");
            this.octoQuad.setSingleChannelPulseWidthTracksWrap(index, trackWrap);
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
    @Block(classes = {OctoQuad.class}, methodName = {"getSingleChannelPulseWidthTracksWrap"})
    public boolean getSingleChannelPulseWidthTracksWrap(int index) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getSingleChannelPulseWidthTracksWrap");
            return this.octoQuad.getSingleChannelPulseWidthTracksWrap(index);
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
    @Block(classes = {OctoQuad.class}, methodName = {"getLocalizerHeadingAxisChoice"})
    public String getLocalizerHeadingAxisChoice() {
        try {
            startBlockExecution(BlockType.GETTER, ".LocalizerYawAxis");
            OctoQuad.LocalizerYawAxis localizerYawAxis = this.octoQuad.getLocalizerHeadingAxisChoice();
            if (localizerYawAxis != null) {
                return localizerYawAxis.toString();
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
    @Block(classes = {OctoQuad.class}, methodName = {"getLocalizerStatus"})
    public String getLocalizerStatus() {
        try {
            startBlockExecution(BlockType.GETTER, ".LocalizerStatus");
            OctoQuad.LocalizerStatus localizerStatus = this.octoQuad.getLocalizerStatus();
            if (localizerStatus != null) {
                return localizerStatus.toString();
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
    @Block(classes = {OctoQuad.class}, methodName = {"setAllLocalizerParameters"})
    public void setAllLocalizerParameters(int portX, int portY, float ticksPerMM_x, float ticksPerMM_y, float tcpOffsetMM_X, float tcpOffsetMM_Y, float headingScalar, int velocityIntervalMs) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setAllLocalizerParameters");
            this.octoQuad.setAllLocalizerParameters(portX, portY, ticksPerMM_x, ticksPerMM_y, tcpOffsetMM_X, tcpOffsetMM_Y, headingScalar, velocityIntervalMs);
        } finally {
        }
    }

    @JavascriptInterface
    @Block(classes = {OctoQuad.class}, methodName = {"readLocalizerData"})
    public String readLocalizerData() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".readLocalizerData");
            OctoQuad.LocalizerDataBlock localizerDataBlock = this.octoQuad.readLocalizerData();
            return localizerDataBlockToJson(localizerDataBlock);
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    private static String localizerDataBlockToJson(OctoQuad.LocalizerDataBlock localizerDataBlock) {
        String originalJson = toJson(localizerDataBlock);
        if (!originalJson.endsWith("}")) {
            RobotLog.ww("OctoQuadAccess", "Unexpected: result from toJson(LocalizerDataBlock) does not end with '}'!");
            return originalJson;
        }
        String json = originalJson.substring(0, originalJson.length() - 1) + DocLint.TAGS_SEPARATOR + "\"isDataValid\":" + toJson(Boolean.valueOf(localizerDataBlock.isDataValid())) + "}";
        return json;
    }

    @JavascriptInterface
    @Block(classes = {OctoQuad.class}, methodName = {"setLocalizerPose"})
    public void setLocalizerPose(int posX_mm, int posY_mm, float heading_rad) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setLocalizerPose");
            this.octoQuad.setLocalizerPose(posX_mm, posY_mm, heading_rad);
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
    @Block(classes = {OctoQuad.class}, methodName = {"setLocalizerHeading"})
    public void setLocalizerHeading(float heading_rad) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setLocalizerHeading");
            this.octoQuad.setLocalizerHeading(heading_rad);
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
    @Block(classes = {OctoQuad.class}, methodName = {"resetLocalizerAndCalibrateIMU"})
    public void resetLocalizerAndCalibrateIMU() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".resetLocalizerAndCalibrateIMU");
            this.octoQuad.resetLocalizerAndCalibrateIMU();
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
