package com.google.blocks.ftcrobotcontroller.runtime.obsolete;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.runtime.Access;
import com.google.blocks.ftcrobotcontroller.runtime.BlockType;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;

/* JADX INFO: loaded from: classes8.dex */
public class VuforiaTrackableDefaultListenerAccess extends Access {
    public VuforiaTrackableDefaultListenerAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "VuforiaTrackableDefaultListener");
    }

    @JavascriptInterface
    public void setPhoneInformation(Object vuforiaTrackableDefaultListenerArg, Object phoneLocationOnRobotArg, String cameraDirectionString) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setPhoneInformation");
    }

    @JavascriptInterface
    public void setCameraLocationOnRobot(Object vuforiaTrackableDefaultListenerArg, String cameraNameString, Object cameraLocationOnRobotArg) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setCameraLocationOnRobot");
    }

    @JavascriptInterface
    public boolean isVisible(Object vuforiaTrackableDefaultListenerArg) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".isVisible");
        return false;
    }

    @JavascriptInterface
    public Object getUpdatedRobotLocation(Object vuforiaTrackableDefaultListenerArg) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".getUpdatedRobotLocation");
        return null;
    }

    @JavascriptInterface
    public Object getPose(Object vuforiaTrackableDefaultListenerArg) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".getPose");
        return null;
    }

    @JavascriptInterface
    public String getRelicRecoveryVuMark(Object vuforiaTrackableDefaultListenerArg) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".getRelicRecoveryVuMark");
        return "UNKNOWN";
    }
}
