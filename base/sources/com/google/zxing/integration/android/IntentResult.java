package com.google.zxing.integration.android;

import android.content.Intent;

/* JADX INFO: loaded from: classes8.dex */
public final class IntentResult {
    private final String barcodeImagePath;
    private final String contents;
    private final String errorCorrectionLevel;
    private final String formatName;
    private final Integer orientation;
    private final Intent originalIntent;
    private final byte[] rawBytes;

    IntentResult() {
        this(null, null, null, null, null, null, null);
    }

    IntentResult(Intent intent) {
        this(null, null, null, null, null, null, intent);
    }

    IntentResult(String contents, String formatName, byte[] rawBytes, Integer orientation, String errorCorrectionLevel, String barcodeImagePath, Intent originalIntent) {
        this.contents = contents;
        this.formatName = formatName;
        this.rawBytes = rawBytes;
        this.orientation = orientation;
        this.errorCorrectionLevel = errorCorrectionLevel;
        this.barcodeImagePath = barcodeImagePath;
        this.originalIntent = originalIntent;
    }

    public String getContents() {
        return this.contents;
    }

    public String getFormatName() {
        return this.formatName;
    }

    public byte[] getRawBytes() {
        return this.rawBytes;
    }

    public Integer getOrientation() {
        return this.orientation;
    }

    public String getErrorCorrectionLevel() {
        return this.errorCorrectionLevel;
    }

    public String getBarcodeImagePath() {
        return this.barcodeImagePath;
    }

    public Intent getOriginalIntent() {
        return this.originalIntent;
    }

    public String toString() {
        int rawBytesLength = this.rawBytes == null ? 0 : this.rawBytes.length;
        return "Format: " + this.formatName + "\nContents: " + this.contents + "\nRaw bytes: (" + rawBytesLength + " bytes)\nOrientation: " + this.orientation + "\nEC level: " + this.errorCorrectionLevel + "\nBarcode image: " + this.barcodeImagePath + "\nOriginal intent: " + this.originalIntent + '\n';
    }
}
