package com.google.zxing.aztec.encoder;

import com.google.zxing.common.BitArray;

/* JADX INFO: loaded from: classes8.dex */
final class SimpleToken extends Token {
    private final short bitCount;
    private final short value;

    SimpleToken(Token previous, int value, int bitCount) {
        super(previous);
        this.value = (short) value;
        this.bitCount = (short) bitCount;
    }

    @Override // com.google.zxing.aztec.encoder.Token
    void appendTo(BitArray bitArray, byte[] text) {
        bitArray.appendBits(this.value, this.bitCount);
    }

    public String toString() {
        int value = (this.value & ((1 << this.bitCount) - 1)) | (1 << this.bitCount);
        return "<" + Integer.toBinaryString((1 << this.bitCount) | value).substring(1) + '>';
    }
}
