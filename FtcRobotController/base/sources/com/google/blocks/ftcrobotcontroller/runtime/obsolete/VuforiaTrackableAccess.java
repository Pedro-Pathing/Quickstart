package com.google.blocks.ftcrobotcontroller.runtime.obsolete;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.runtime.Access;
import com.google.blocks.ftcrobotcontroller.runtime.BlockType;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;

/* JADX INFO: loaded from: classes8.dex */
public class VuforiaTrackableAccess extends Access {
    public VuforiaTrackableAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "VuforiaTrackable");
    }

    @JavascriptInterface
    public void setLocation(Object vuforiaTrackableArg, Object matrixArg) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setLocation");
    }

    @JavascriptInterface
    public Object getLocation(Object vuforiaTrackableArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".Location");
        return null;
    }

    @JavascriptInterface
    public void setUserData(Object vuforiaTrackableArg, Object userData) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setUserData");
    }

    @JavascriptInterface
    public Object getUserData(Object vuforiaTrackableArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".UserData");
        return null;
    }

    @JavascriptInterface
    public Object getTrackables(Object vuforiaTrackableArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".Trackables");
        return null;
    }

    @JavascriptInterface
    public void setName(Object vuforiaTrackableArg, String name) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setName");
    }

    @JavascriptInterface
    public String getName(Object vuforiaTrackableArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".Name");
        return "";
    }

    @JavascriptInterface
    public Object getListener(Object vuforiaTrackableArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".Listener");
        return null;
    }
}
