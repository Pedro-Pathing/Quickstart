package com.google.blocks.ftcrobotcontroller.runtime.obsolete;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.runtime.Access;
import com.google.blocks.ftcrobotcontroller.runtime.BlockType;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;

/* JADX INFO: loaded from: classes8.dex */
public class VuforiaLocalizerAccess extends Access {
    public VuforiaLocalizerAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "VuforiaLocalizer");
    }

    @JavascriptInterface
    public Object create(Object vuforiaLocalizerParameters) {
        handleObsoleteBlockExecution(BlockType.CREATE, "");
        return null;
    }

    @JavascriptInterface
    public Object loadTrackablesFromAsset(Object vuforiaLocalizerArg, String assetName) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".loadTrackablesFromAsset");
        return null;
    }

    @JavascriptInterface
    public Object loadTrackablesFromFile(Object vuforiaLocalizerArg, String absoluteFileName) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".loadTrackablesFromFile");
        return null;
    }
}
