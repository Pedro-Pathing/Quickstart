package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

/* JADX INFO: loaded from: classes8.dex */
class HuskyLensAccess extends HardwareAccess<HuskyLens> {
    private final HuskyLens huskyLens;

    HuskyLensAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, HuskyLens.class);
        this.huskyLens = (HuskyLens) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {HuskyLens.class}, methodName = {"knock"})
    public boolean knock() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".knock");
            return this.huskyLens.knock();
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
    @Block(classes = {HuskyLens.class}, methodName = {"selectAlgorithm"})
    public void selectAlgorithm(String algorithmString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".selectAlgorithm");
            HuskyLens.Algorithm algorithm = (HuskyLens.Algorithm) checkArg(algorithmString, HuskyLens.Algorithm.class, "");
            if (algorithm != null) {
                this.huskyLens.selectAlgorithm(algorithm);
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
    @Block(classes = {HuskyLens.class}, methodName = {"blocks"})
    public String blocks() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".blocks");
            return SimpleGson.getInstance().toJson(this.huskyLens.blocks());
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
    @Block(classes = {HuskyLens.class}, methodName = {"blocks"})
    public String blocks_withId(int id) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".blocks");
            return SimpleGson.getInstance().toJson(this.huskyLens.blocks(id));
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
    @Block(classes = {HuskyLens.class}, methodName = {"arrows"})
    public String arrows() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".arrows");
            return SimpleGson.getInstance().toJson(this.huskyLens.arrows());
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
    @Block(classes = {HuskyLens.class}, methodName = {"arrows"})
    public String arrows_withId(int id) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".arrows");
            return SimpleGson.getInstance().toJson(this.huskyLens.arrows(id));
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    static String huskyLensBlockToText(String json) {
        HuskyLens.Block block = (HuskyLens.Block) SimpleGson.getInstance().fromJson(json, HuskyLens.Block.class);
        return block.toString();
    }

    static String huskyLensArrowToText(String json) {
        HuskyLens.Arrow arrow = (HuskyLens.Arrow) SimpleGson.getInstance().fromJson(json, HuskyLens.Arrow.class);
        return arrow.toString();
    }
}
