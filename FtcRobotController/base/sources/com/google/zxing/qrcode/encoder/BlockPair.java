package com.google.zxing.qrcode.encoder;

/* JADX INFO: loaded from: classes8.dex */
final class BlockPair {
    private final byte[] dataBytes;
    private final byte[] errorCorrectionBytes;

    BlockPair(byte[] data, byte[] errorCorrection) {
        this.dataBytes = data;
        this.errorCorrectionBytes = errorCorrection;
    }

    public byte[] getDataBytes() {
        return this.dataBytes;
    }

    public byte[] getErrorCorrectionBytes() {
        return this.errorCorrectionBytes;
    }
}
