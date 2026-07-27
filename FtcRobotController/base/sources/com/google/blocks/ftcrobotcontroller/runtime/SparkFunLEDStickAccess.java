package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

/* JADX INFO: loaded from: classes8.dex */
class SparkFunLEDStickAccess extends HardwareAccess<SparkFunLEDStick> {
    private final SparkFunLEDStick sparkFunLEDStick;

    SparkFunLEDStickAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, SparkFunLEDStick.class);
        this.sparkFunLEDStick = (SparkFunLEDStick) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {SparkFunLEDStick.class}, methodName = {"setColor"})
    public void setColor_withPosition(int position, long longColor) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setColor");
            int color = (int) ((-1) & longColor);
            this.sparkFunLEDStick.setColor(position, color);
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
    @Block(classes = {SparkFunLEDStick.class}, methodName = {"setColor"})
    public void setColor(long longColor) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setColor");
            int color = (int) ((-1) & longColor);
            this.sparkFunLEDStick.setColor(color);
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
    @Block(classes = {SparkFunLEDStick.class}, methodName = {"setColors"})
    public void setColors(String jsonColors) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setColors");
            long[] longColors = (long[]) SimpleGson.getInstance().fromJson(jsonColors, long[].class);
            int[] colors = new int[longColors.length];
            for (int i = 0; i < colors.length; i++) {
                colors[i] = (int) (longColors[i] & (-1));
            }
            this.sparkFunLEDStick.setColors(colors);
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
    @Block(classes = {SparkFunLEDStick.class}, methodName = {"setBrightness"})
    public void setBrightness_withPosition(int position, int brightness) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setBrightness");
            this.sparkFunLEDStick.setBrightness(position, brightness);
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
    @Block(classes = {SparkFunLEDStick.class}, methodName = {"setBrightness"})
    public void setBrightness(int brightness) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setBrightness");
            this.sparkFunLEDStick.setBrightness(brightness);
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
    @Block(classes = {SparkFunLEDStick.class}, methodName = {"turnAllOff"})
    public void turnAllOff() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".turnAllOff");
            this.sparkFunLEDStick.turnAllOff();
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
