package com.google.blocks.ftcrobotcontroller.runtime.obsolete;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.runtime.Access;
import com.google.blocks.ftcrobotcontroller.runtime.BlockType;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;

/* JADX INFO: loaded from: classes8.dex */
public class VuforiaTrackablesAccess extends Access {
    public VuforiaTrackablesAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "VuforiaTrackables");
    }

    @JavascriptInterface
    public int getSize(Object vuforiaTrackablesArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".Size");
        return 0;
    }

    @JavascriptInterface
    public String getName(Object vuforiaTrackablesArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".Name");
        return "";
    }

    @JavascriptInterface
    public Object getLocalizer(Object vuforiaTrackablesArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".Localizer");
        return null;
    }

    @JavascriptInterface
    public Object get(Object vuforiaTrackablesArg, int index) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".get");
        return null;
    }

    @JavascriptInterface
    public void setName(Object vuforiaTrackablesArg, String name) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setName");
    }

    @JavascriptInterface
    public void activate(Object vuforiaTrackablesArg) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".activate");
    }

    @JavascriptInterface
    public void deactivate(Object vuforiaTrackablesArg) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".deactivate");
    }
}
