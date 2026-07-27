package com.google.blocks.ftcrobotcontroller.runtime.obsolete;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;

/* JADX INFO: loaded from: classes8.dex */
public final class VuforiaSkyStoneAccess extends VuforiaBaseAccess {
    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void activate() {
        super.activate();
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void deactivate() {
        super.deactivate();
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ Object getVuforiaLocalizer() {
        return super.getVuforiaLocalizer();
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void initialize_withCameraDirection(String str, String str2, boolean z, boolean z2, String str3, float f, float f2, float f3, float f4, float f5, float f6, boolean z3) {
        super.initialize_withCameraDirection(str, str2, z, z2, str3, f, f2, f3, f4, f5, f6, z3);
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void initialize_withCameraDirection_2(String str, boolean z, boolean z2, String str2, float f, float f2, float f3, String str3, float f4, float f5, float f6, boolean z3) {
        super.initialize_withCameraDirection_2(str, z, z2, str2, f, f2, f3, str3, f4, f5, f6, z3);
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void initialize_withWebcam(String str, String str2, boolean z, boolean z2, String str3, float f, float f2, float f3, float f4, float f5, float f6, boolean z3) {
        super.initialize_withWebcam(str, str2, z, z2, str3, f, f2, f3, f4, f5, f6, z3);
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void initialize_withWebcam_2(String str, String str2, boolean z, boolean z2, String str3, float f, float f2, float f3, String str4, float f4, float f5, float f6, boolean z3) {
        super.initialize_withWebcam_2(str, str2, z, z2, str3, f, f2, f3, str4, f4, f5, f6, z3);
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void setActiveCamera(String str) {
        super.setActiveCamera(str);
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ String track(String str) {
        return super.track(str);
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ String trackPose(String str) {
        return super.trackPose(str);
    }

    public VuforiaSkyStoneAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "VuforiaSKYSTONE");
    }
}
