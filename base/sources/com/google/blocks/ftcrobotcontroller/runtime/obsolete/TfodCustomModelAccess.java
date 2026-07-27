package com.google.blocks.ftcrobotcontroller.runtime.obsolete;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.runtime.BlockType;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;

/* JADX INFO: loaded from: classes8.dex */
public final class TfodCustomModelAccess extends TfodBaseAccess {
    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void activate() {
        super.activate();
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void deactivate() {
        super.deactivate();
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ String getRecognitions() {
        return super.getRecognitions();
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void initialize(Object obj, float f, boolean z, boolean z2) {
        super.initialize(obj, f, z, z2);
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void initializeWithAllArgs(Object obj, float f, boolean z, boolean z2, boolean z3, boolean z4, int i, int i2, int i3, int i4, int i5, double d, float f2, float f3, float f4, float f5) {
        super.initializeWithAllArgs(obj, f, z, z2, z3, z4, i, i2, i3, i4, i5, d, f2, f3, f4, f5);
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void initializeWithIsModelTensorFlow2(Object obj, float f, boolean z, boolean z2, boolean z3) {
        super.initializeWithIsModelTensorFlow2(obj, f, z, z2, z3);
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void setClippingMargins(int i, int i2, int i3, int i4) {
        super.setClippingMargins(i, i2, i3, i4);
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodBaseAccess
    @JavascriptInterface
    public /* bridge */ /* synthetic */ void setZoom(double d, double d2) {
        super.setZoom(d, d2);
    }

    public TfodCustomModelAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "TensorFlowObjectDetectionCustomModel");
    }

    @JavascriptInterface
    public void setModelFromAsset(String assetName, String jsonLabels) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setModelFromAsset");
    }

    @JavascriptInterface
    public void setModelFromFile(String tfliteModelFilename, String jsonLabels) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setModelFromFile");
    }
}
